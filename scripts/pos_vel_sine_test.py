"""Dual-axis native POS_VEL sine test (0x100+ID), with HF detection.

Position target: center + A*sin(2*pi*t/T)
Velocity limit: constant (default 1.0 rad/s)

The drive runs position -> velocity -> current.  No MIT frames are used.

Safety:
- position-derived speed (0.98 rad/s), raw backstop (50 rad/s)
- HF raw-velocity RMS (10 Hz high-pass, 0.2 s) > 1.0 rad/s
- torque, position-deviation, temperature, feedback-timeout limits
- restores the initial drive mode and zeroes the CAN timeout on exit
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

LOG = logging.getLogger("pos_vel_sine")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gimbal.example.yaml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--amplitude", type=float, default=0.2, help="position amplitude rad")
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--velocity-limit", type=float, default=1.0, help="rad/s")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--speed-limit", type=float, default=0.98)
    ap.add_argument("--vel-window", type=float, default=0.1)
    ap.add_argument("--raw-speed-limit", type=float, default=50.0)
    ap.add_argument("--vibration-limit", type=float, default=1.0)
    ap.add_argument("--vibration-window", type=float, default=0.2)
    ap.add_argument("--torque-limit", type=float, default=2.0)
    ap.add_argument("--position-limit", type=float, default=0.6)
    ap.add_argument("--yaw-kp-apr", type=float, default=None,
                    help="temporary drive KP_APR (RID 0x1B), RAM only")
    ap.add_argument("--pitch-kp-apr", type=float, default=None)
    ap.add_argument("--yaw-ki-apr", type=float, default=None,
                    help="temporary drive KI_APR (RID 0x1C), RAM only")
    ap.add_argument("--pitch-ki-apr", type=float, default=None)
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
    if not (0.02 <= args.amplitude <= 0.5):
        raise SystemExit("amplitude must be in [0.02, 0.5] rad")
    if args.period < 1.0:
        raise SystemExit("period must be >= 1 s")
    if not (0.05 <= args.velocity_limit <= 1.0):
        raise SystemExit("velocity_limit must be in [0.05, 1] rad/s")
    if args.speed_limit > 1.0:
        raise SystemExit("speed_limit must be <= 1 rad/s")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = load_config(args.config)
    port = args.port or cfg.can.port
    bus = SerialUsb2CanBus(
        port=port, baud=cfg.can.baud, can_bitrate=cfg.can.can_bitrate,
        set_can_baud=cfg.can.set_can_baud, send_heartbeat=False,
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
    csv_path = rec_dir / f"pos_vel_sine_{stamp}.csv"
    log_path = rec_dir / f"pos_vel_sine_{stamp}.log"
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
    initial_modes: dict[str, int] = {}
    orig_params: dict[str, dict[str, float]] = {}
    tuned: dict[str, bool] = {}
    enabled = False
    csv_file = None
    writer = None
    max_tau = {"yaw": 0.0, "pitch": 0.0}
    max_vel = {"yaw": 0.0, "pitch": 0.0}
    max_err = {"yaw": 0.0, "pitch": 0.0}
    try:
        enabled = True
        for name, motor in motors.items():
            initial_modes[name] = motor.read_register(0x0A, timeout=0.75)
            raw = motor.read_register(0x50, timeout=0.75)
            centers[name] = cfg.motors[name].direction * f32(raw) + cfg.motors[name].zero_offset_rad
            try:
                orig_params[name] = {
                    "kp_apr": read_float_reg(motor, 0x1B),
                    "ki_apr": read_float_reg(motor, 0x1C),
                }
            except Exception as exc:
                LOG.warning("%s: could not read KP_APR/KI_APR: %s", name, exc)
            motor.disable()
            if initial_modes[name] != 2:
                if not motor.set_control_mode(2):
                    raise RuntimeError(f"{name}: failed to enter POS_VEL mode")
                if motor.read_register(0x0A, timeout=0.5) != 2:
                    raise RuntimeError(f"{name}: POS_VEL readback failed")
            want_kp = args.yaw_kp_apr if name == "yaw" else args.pitch_kp_apr
            want_ki = args.yaw_ki_apr if name == "yaw" else args.pitch_ki_apr
            if want_kp is None:
                want_kp = cfg.motors[name].position_kp_apr
            if want_ki is None:
                want_ki = cfg.motors[name].position_ki_apr
            if want_kp is not None:
                write_float_reg(motor, 0x1B, want_kp)
                tuned[name] = True
            if want_ki is not None:
                write_float_reg(motor, 0x1C, want_ki)
                tuned[name] = True
            if name in orig_params and (want_kp is not None or want_ki is not None):
                LOG.info(
                    "tune %s: KP_APR=%.3f KI_APR=%.3f (orig %.3f/%.3f)",
                    name, read_float_reg(motor, 0x1B), read_float_reg(motor, 0x1C),
                    orig_params[name]["kp_apr"], orig_params[name]["ki_apr"],
                )
            motor.clear_error()
            motor.write_register(0x09, 20000, timeout=0.5)
            motor.enable()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(m.snapshot().status == 1 for m in motors.values()):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("motors did not enable")
        # Pre-roll a POS_VEL hold so the first control cycle has fresh feedback.
        # (The motor only sends feedback in response to control frames.)
        for name, motor in motors.items():
            motor.send_pos_vel(centers[name], 0.0)
        time.sleep(0.05)

        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow([
            "t",
            "yaw_target", "yaw_pos", "yaw_err", "yaw_vlim", "yaw_vel", "yaw_vest", "yaw_tau", "yaw_t", "yaw_hf",
            "pitch_target", "pitch_pos", "pitch_err", "pitch_vlim", "pitch_vel", "pitch_vest", "pitch_tau", "pitch_t", "pitch_hf",
        ])
        est = {name: VelocityEstimator(args.vel_window, 10.0) for name in motors}
        hf_lp = {name: LowPass(10.0) for name in motors}
        hf_hist: dict[str, deque[tuple[float, float]]] = {name: deque() for name in motors}
        hf_value = {name: 0.0 for name in motors}
        last_cycle = time.monotonic()
        last_print = 0.0
        last_csv = 0.0
        last_stop_check = 0.0
        t0 = time.monotonic()
        next_t = time.perf_counter()
        loop = 0
        omega = 2.0 * math.pi / args.period
        LOG.info(
            "start: centers=%s A=%.3f period=%.2f vlim=%.2f speed_limit=%.2f "
            "vibration_limit=%.2f torque_limit=%.2f",
            centers, args.amplitude, args.period, args.velocity_limit,
            args.speed_limit, args.vibration_limit, args.torque_limit,
        )

        while not stop_event:
            now = time.monotonic()
            t = now - t0
            if args.duration > 0 and t >= args.duration:
                stops.append("duration")
                break
            if now - last_stop_check >= 0.2:
                last_stop_check = now
                if stop_file.exists():
                    stops.append("stop_file")
                    break

            env = min(1.0, t / 1.0)
            targets = {
                name: centers[name] + env * args.amplitude * math.sin(omega * t)
                for name in motors
            }
            for name, motor in motors.items():
                motor.send_pos_vel(targets[name], args.velocity_limit)

            snaps = {name: motor.snapshot() for name, motor in motors.items()}
            dt_loop = max(1e-6, now - last_cycle)
            last_cycle = now
            for name, s in snaps.items():
                vest = est[name].update(s.pos, now)
                err = targets[name] - s.pos
                max_vel[name] = max(max_vel[name], abs(vest))
                max_tau[name] = max(max_tau[name], abs(s.tau))
                max_err[name] = max(max_err[name], abs(err))
                lp = hf_lp[name].update(s.vel, dt_loop)
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
                elif abs(vest) > args.speed_limit:
                    stops.append(f"{name}: speed {vest:+.3f} > {args.speed_limit}")
                    stop_event = True
                elif abs(s.vel) > args.raw_speed_limit:
                    stops.append(f"{name}: raw speed {s.vel:+.3f} > {args.raw_speed_limit}")
                    stop_event = True
                elif abs(s.tau) > args.torque_limit:
                    stops.append(f"{name}: torque {s.tau:+.3f} > {args.torque_limit}")
                    stop_event = True
                elif abs(err) > args.position_limit:
                    stops.append(f"{name}: position error {err:+.3f} > {args.position_limit}")
                    stop_event = True
                elif max(s.t_mos, s.t_rotor) > 80.0:
                    stops.append(f"{name}: temperature {s.t_mos}/{s.t_rotor}")
                    stop_event = True
            if stop_event:
                break

            if now - last_csv >= 0.01:
                last_csv = now
                y, p = snaps["yaw"], snaps["pitch"]
                yt, pt = targets["yaw"], targets["pitch"]
                writer.writerow([
                    f"{t:.4f}",
                    f"{yt:.6f}", f"{y.pos:.6f}", f"{yt - y.pos:.6f}", f"{args.velocity_limit:.6f}",
                    f"{y.vel:.6f}", f"{est['yaw'].value:.6f}", f"{y.tau:.6f}", f"{y.t_mos:.1f}", f"{hf_value['yaw']:.4f}",
                    f"{pt:.6f}", f"{p.pos:.6f}", f"{pt - p.pos:.6f}", f"{args.velocity_limit:.6f}",
                    f"{p.vel:.6f}", f"{est['pitch'].value:.6f}", f"{p.tau:.6f}", f"{p.t_mos:.1f}", f"{hf_value['pitch']:.4f}",
                ])
                if loop % 50 == 0:
                    csv_file.flush()

            if now - last_print >= 1.0:
                last_print = now
                LOG.info(
                    "t=%5.1f yaw tgt=%+.3f pos=%+.3f err=%+.3f vel=%+.3f tau=%+.3f hf=%.2f | "
                    "pitch tgt=%+.3f pos=%+.3f err=%+.3f vel=%+.3f tau=%+.3f hf=%.2f | maxT %.2f/%.2f",
                    t, targets["yaw"], snaps["yaw"].pos, targets["yaw"] - snaps["yaw"].pos,
                    est["yaw"].value, snaps["yaw"].tau, hf_value["yaw"],
                    targets["pitch"], snaps["pitch"].pos, targets["pitch"] - snaps["pitch"].pos,
                    est["pitch"].value, snaps["pitch"].tau, hf_value["pitch"],
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
                        motor.send_pos_vel(motor.snapshot().pos, 0.0)
                    time.sleep(0.02)
                for name, motor in motors.items():
                    try:
                        motor.disable()
                        if tuned.get(name) and name in orig_params:
                            write_float_reg(motor, 0x1B, orig_params[name]["kp_apr"])
                            write_float_reg(motor, 0x1C, orig_params[name]["ki_apr"])
                            LOG.info("%s: restored KP_APR/KI_APR", name)
                        motor.write_register(0x09, 0, timeout=0.3)
                        motor.write_register(0x0A, 3, timeout=0.3)
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
        print(f"max_tau: {max_tau}  max_vel: {max_vel}  max_err: {max_err}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
