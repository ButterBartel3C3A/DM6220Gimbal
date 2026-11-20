"""Yaw continuous rotation in native POS_VEL mode, pitch held in POS_VEL.

Both axes stay in the production path (0x100+ID, drive position -> velocity ->
current).  No MIT frames are used.

The yaw position target is an unwrapped multi-turn ramp; the yaw velocity limit
equals the requested speed.  The pitch target is the multi-turn position read
from ``p_m`` (0x50) before enable.

Safety:
- commanded |speed| <= 4.0 rad/s (hard cap), default 4.0
- position-derived speed abort, raw-velocity backstop
- HF vibration RMS, torque, temperature, feedback-timeout, pitch-drift aborts
- STOP file, duration limit, Ctrl+C
- restores the initial drive mode, CAN timeout and position gains on exit

Usage:
    python scripts/pos_vel_continuous_test.py --speed 4.0 --duration 20
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import signal
import struct
import time
from collections import deque
from pathlib import Path

from dm6220_gimbal.config import load_config
from dm6220_gimbal.control import LowPass, VelocityEstimator
from dm6220_gimbal.motor import CanRouter, DmMotor
from dm6220_gimbal.transport import SerialUsb2CanBus

LOG = logging.getLogger("pos_vel_continuous")

MAX_SPEED = 4.0
ABORT_SPEED = 4.4  # tripping threshold; commanded speed stays at 4.0
ABORT_RAW_SPEED = 50.0
ABORT_YAW_TORQUE = 1.0
ABORT_PITCH_TORQUE = 1.2
ABORT_TEMP_C = 75.0
ABORT_PITCH_DRIFT = 0.2
PMAX = 12.5


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gimbal.example.yaml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--speed", type=float, default=4.0, help="rad/s, sign sets direction")
    ap.add_argument("--duration", type=float, default=20.0, help="seconds")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--velocity-limit", type=float, default=None,
                    help="yaw velocity limit, defaults to |speed|")
    ap.add_argument("--pitch-velocity-limit", type=float, default=1.0)
    ap.add_argument("--vel-window", type=float, default=0.1)
    ap.add_argument("--vibration-limit", type=float, default=1.0)
    ap.add_argument("--vibration-window", type=float, default=0.2)
    ap.add_argument("--record-dir", default="recordings")
    return ap.parse_args()


def f32(raw: int) -> float:
    return struct.unpack("<f", int(raw).to_bytes(4, "little"))[0]


def write_float_reg(motor: DmMotor, rid: int, value: float) -> None:
    bits = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    if not motor.write_register(rid, bits, timeout=0.5):
        raise RuntimeError(f"{motor.name}: failed to write RID 0x{rid:02X}")


def read_float_reg(motor: DmMotor, rid: int) -> float:
    raw = motor.read_register(rid, timeout=0.75)
    if raw is None:
        raise RuntimeError(f"{motor.name}: failed to read RID 0x{rid:02X}")
    return f32(raw)


def main() -> int:
    args = parse_args()
    if abs(args.speed) > MAX_SPEED:
        raise SystemExit(f"speed {args.speed} exceeds hard limit {MAX_SPEED} rad/s")
    if args.speed == 0.0:
        raise SystemExit("speed must be non-zero")
    if not (0.0 < args.duration <= 120.0):
        raise SystemExit("duration must be in (0, 120] s")
    yaw_vlim = args.velocity_limit if args.velocity_limit is not None else abs(args.speed)
    if not (0.05 <= yaw_vlim <= MAX_SPEED):
        raise SystemExit(f"velocity_limit must be in [0.05, {MAX_SPEED}] rad/s")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = load_config(args.config)
    bus = SerialUsb2CanBus(
        port=args.port or cfg.can.port,
        baud=cfg.can.baud,
        can_bitrate=cfg.can.can_bitrate,
        set_can_baud=cfg.can.set_can_baud,
        send_heartbeat=False,
        auto_detect=cfg.can.auto_detect and args.port is None,
    )
    motors = {name: DmMotor(name, mcfg, bus) for name, mcfg in cfg.motors.items()}
    router = CanRouter(bus, motors)
    for m in motors.values():
        m.attach_router(router)
    router.start()

    rec_dir = Path(args.record_dir)
    rec_dir.mkdir(parents=True, exist_ok=True)
    stop_file = rec_dir / "STOP"
    stop_file.unlink(missing_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = rec_dir / f"pos_vel_continuous_{stamp}.csv"
    log_path = rec_dir / f"pos_vel_continuous_{stamp}.log"
    fh = logging.FileHandler(log_path, encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(fh)

    stop_event = False
    stops: list[str] = []

    def _sig(_s, _f):
        nonlocal stop_event
        stop_event = True
        stops.append("signal")

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    centers: dict[str, float] = {}
    initial_modes: dict[str, int | None] = {}
    orig_params: dict[str, dict[str, float]] = {}
    initial_timeout: dict[str, int | None] = {}
    tuned: dict[str, bool] = {}
    enabled = False
    yaw_target = centers.get("yaw", 0.0)
    csv_file = None
    writer = None
    max_tau = {"yaw": 0.0, "pitch": 0.0}
    max_vel = {"yaw": 0.0, "pitch": 0.0}
    try:
        for name, motor in motors.items():
            initial_modes[name] = motor.read_register(0x0A, timeout=0.75)
            initial_timeout[name] = motor.read_register(0x09, timeout=0.75)
            centers[name] = (
                cfg.motors[name].direction * read_float_reg(motor, 0x50)
                + cfg.motors[name].zero_offset_rad
            )
            orig_params[name] = {
                "kp_apr": read_float_reg(motor, 0x1B),
                "ki_apr": read_float_reg(motor, 0x1C),
            }
            motor.disable()
            if initial_modes[name] != 2:
                if not motor.set_control_mode(2):
                    raise RuntimeError(f"{name}: failed to enter POS_VEL mode")
                if motor.read_register(0x0A, timeout=0.5) != 2:
                    raise RuntimeError(f"{name}: POS_VEL readback failed")
            if motor.read_register(0x09, timeout=0.5) != 20000:
                motor.write_register(0x09, 20000, timeout=0.5)
            write_float_reg(motor, 0x1B, cfg.motors[name].position_kp_apr)
            write_float_reg(motor, 0x1C, cfg.motors[name].position_ki_apr)
            tuned[name] = True
            motor.clear_error()
            motor.enable()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(m.snapshot().status == 1 for m in motors.values()):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("motors did not enable")
        enabled = True
        for name, motor in motors.items():
            motor.send_pos_vel(centers[name], 0.0)
        time.sleep(0.05)

        yaw = motors["yaw"]
        pitch = motors["pitch"]
        yaw_target = centers["yaw"]
        pitch_target = centers["pitch"]
        # Wait for one fresh feedback pair before seeding the unwrapper.
        seed_deadline = time.monotonic() + 1.0
        while time.monotonic() < seed_deadline and not all(
            math.isfinite(m.snapshot().last_rx_age) for m in motors.values()
        ):
            time.sleep(0.01)

        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow([
            "t", "yaw_target", "yaw_pos", "yaw_vel", "yaw_vest", "yaw_tau", "yaw_t", "yaw_hf",
            "pitch_target", "pitch_pos", "pitch_vel", "pitch_tau", "pitch_t", "pitch_hf",
        ])
        est = {name: VelocityEstimator(args.vel_window, 10.0) for name in motors}
        hf_lp = {name: LowPass(10.0) for name in motors}
        hf_hist: dict[str, deque[tuple[float, float]]] = {name: deque() for name in motors}
        hf_value = {name: 0.0 for name in motors}
        unwrapped_pos = centers["yaw"]
        prev_raw_pos = yaw.snapshot().pos
        last_cycle = time.monotonic()
        last_print = 0.0
        last_csv = 0.0
        last_stop_check = 0.0
        t0 = time.monotonic()
        next_t = time.perf_counter()
        loop = 0
        LOG.info(
            "start: yaw speed=%+.3f rad/s vlim=%.2f from %+.3f, pitch hold %.3f "
            "abort_speed=%.2f yaw_tau=%.2f pitch_tau=%.2f",
            args.speed, yaw_vlim, centers["yaw"], centers["pitch"],
            ABORT_SPEED, ABORT_YAW_TORQUE, ABORT_PITCH_TORQUE,
        )

        while not stop_event:
            now = time.monotonic()
            t = now - t0
            if t >= args.duration:
                stops.append("duration")
                break
            if now - last_stop_check >= 0.2:
                last_stop_check = now
                if stop_file.exists():
                    stops.append("stop_file")
                    break

            dt = max(1e-6, now - last_cycle)
            last_cycle = now
            yaw_target += args.speed * dt
            for name, motor in motors.items():
                target = yaw_target if name == "yaw" else pitch_target
                vlim = yaw_vlim if name == "yaw" else args.pitch_velocity_limit
                motor.send_pos_vel(target, vlim)

            snaps = {name: motor.snapshot() for name, motor in motors.items()}
            for name, s in snaps.items():
                if name == "yaw":
                    delta = s.pos - prev_raw_pos
                    if delta > PMAX:
                        delta -= 2 * PMAX
                    elif delta < -PMAX:
                        delta += 2 * PMAX
                    unwrapped_pos += delta
                    prev_raw_pos = s.pos
                vest = est[name].update(s.pos if name != "yaw" else unwrapped_pos, now)
                max_vel[name] = max(max_vel[name], abs(vest))
                max_tau[name] = max(max_tau[name], abs(s.tau))
                lp = hf_lp[name].update(s.vel, dt)
                hf = s.vel - lp
                dq = hf_hist[name]
                dq.append((now, hf))
                while dq and now - dq[0][0] > args.vibration_window:
                    dq.popleft()
                hf_rms = 0.0
                if len(dq) >= 5 and now - dq[0][0] >= args.vibration_window * 0.75:
                    hf_rms = math.sqrt(sum(v * v for _, v in dq) / len(dq))
                hf_value[name] = hf_rms

                if not math.isfinite(s.last_rx_age) or s.last_rx_age > 0.1:
                    stops.append(f"{name}: feedback timeout")
                    stop_event = True
                elif s.status != 1:
                    stops.append(f"{name}: status={s.status}")
                    stop_event = True
                elif hf_rms > args.vibration_limit:
                    stops.append(f"{name}: HF vibration RMS {hf_rms:.3f} > {args.vibration_limit}")
                    stop_event = True
                elif abs(vest) > ABORT_SPEED:
                    stops.append(f"{name}: speed {vest:+.3f} > {ABORT_SPEED}")
                    stop_event = True
                elif abs(s.vel) > ABORT_RAW_SPEED:
                    stops.append(f"{name}: raw speed {s.vel:+.3f} > {ABORT_RAW_SPEED}")
                    stop_event = True
                elif max(s.t_mos, s.t_rotor) > ABORT_TEMP_C:
                    stops.append(f"{name}: temperature {s.t_mos}/{s.t_rotor}")
                    stop_event = True
                elif name == "yaw" and abs(s.tau) > ABORT_YAW_TORQUE:
                    stops.append(f"yaw: torque {s.tau:+.3f} > {ABORT_YAW_TORQUE}")
                    stop_event = True
                elif name == "pitch" and abs(s.tau) > ABORT_PITCH_TORQUE:
                    stops.append(f"pitch: torque {s.tau:+.3f} > {ABORT_PITCH_TORQUE}")
                    stop_event = True
                elif name == "pitch" and abs(s.pos - pitch_target) > ABORT_PITCH_DRIFT:
                    stops.append(f"pitch: drift {s.pos - pitch_target:+.3f} > {ABORT_PITCH_DRIFT}")
                    stop_event = True
            if stop_event:
                break

            if now - last_csv >= 0.01:
                last_csv = now
                y, p = snaps["yaw"], snaps["pitch"]
                writer.writerow([
                    f"{t:.4f}",
                    f"{yaw_target:.6f}", f"{unwrapped_pos:.6f}", f"{y.vel:.6f}",
                    f"{est['yaw'].value:.6f}", f"{y.tau:.6f}", f"{y.t_mos:.1f}", f"{hf_value['yaw']:.4f}",
                    f"{pitch_target:.6f}", f"{p.pos:.6f}", f"{p.vel:.6f}",
                    f"{p.tau:.6f}", f"{p.t_mos:.1f}", f"{hf_value['pitch']:.4f}",
                ])
                if loop % 50 == 0:
                    csv_file.flush()

            if now - last_print >= 1.0:
                last_print = now
                LOG.info(
                    "t=%5.1f yaw pos=%+.3f tgt=%+.3f vel=%+.3f tau=%+.3f hf=%.3f | "
                    "pitch pos=%+.3f err=%+.3f tau=%+.3f hf=%.3f | maxT %.2f/%.2f",
                    t, unwrapped_pos, yaw_target, est["yaw"].value, snaps["yaw"].tau, hf_value["yaw"],
                    snaps["pitch"].pos, snaps["pitch"].pos - pitch_target,
                    snaps["pitch"].tau, hf_value["pitch"],
                    max_tau["yaw"], max_tau["pitch"],
                )

            loop += 1
            next_t += 1.0 / args.rate
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

    finally:
        try:
            if enabled:
                for _ in range(3):
                    for motor in motors.values():
                        snap = motor.snapshot()
                        if math.isfinite(snap.last_rx_age):
                            motor.send_pos_vel(snap.pos, 0.0)
                    time.sleep(0.02)
                for name, motor in motors.items():
                    try:
                        motor.disable()
                        if tuned.get(name) and name in orig_params:
                            write_float_reg(motor, 0x1B, orig_params[name]["kp_apr"])
                            write_float_reg(motor, 0x1C, orig_params[name]["ki_apr"])
                        motor.write_register(0x09, int(initial_timeout[name] or 0), timeout=0.3)
                        if initial_modes.get(name) is not None:
                            motor.write_register(0x0A, int(initial_modes[name]), timeout=0.3)
                        motor.clear_error()
                    except Exception as exc:
                        LOG.warning("cleanup %s: %s", name, exc)
        finally:
            try:
                router.stop()
            except Exception:
                pass
            try:
                bus.shutdown()
            except Exception:
                pass
            if csv_file is not None:
                csv_file.close()
            logging.getLogger().removeHandler(fh)
            fh.close()
        LOG.info("stopped: %s", stops or ["unknown"])
        print(f"CSV: {csv_path}")
        print(f"LOG: {log_path}")
        print(f"max_tau: {max_tau}  max_vel: {max_vel}")
        if enabled:
            print(f"yaw travel: {yaw_target - centers['yaw']:+.3f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
