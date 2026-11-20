"""Dual-axis velocity-mode sine test (forward/reverse), with HF detection.

This is the velocity-mode counterpart of `mit_oscillation_test.py`: the driver
runs its internal velocity/current loops; we only send 0x200+ID float commands.

Safety:
- position-derived speed limit (default 0.98 rad/s)
- raw velocity backstop (50 rad/s) and HF RMS detector (10 Hz high-pass, 0.2 s)
- torque, position-deviation, temperature and feedback-timeout limits
- restores the initial mode and zeroes the CAN timeout on exit

Usage:
    python scripts/velocity_sine_test.py --port COM10 --duration 30
"""

from __future__ import annotations

import argparse
import csv
import json
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

LOG = logging.getLogger("vel_sine")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gimbal.example.yaml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--amplitude", type=float, default=0.314, help="velocity amplitude rad/s")
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--duration", type=float, default=30.0, help="0 = until stop")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--speed-limit", type=float, default=10.0, help="position-derived")
    ap.add_argument("--vel-window", type=float, default=0.1)
    ap.add_argument("--raw-speed-limit", type=float, default=50.0)
    ap.add_argument("--vibration-limit", type=float, default=1.0, help="HF RMS rad/s")
    ap.add_argument("--vibration-window", type=float, default=0.2)
    ap.add_argument("--torque-limit", type=float, default=2.0)
    ap.add_argument("--position-limit", type=float, default=0.6, help="rad from center")
    ap.add_argument("--record-dir", default="recordings")
    return ap.parse_args()


def f32(raw: int) -> float:
    return struct.unpack("<f", int(raw).to_bytes(4, "little"))[0]


def main() -> int:
    args = parse_args()
    if args.amplitude <= 0 or args.amplitude >= 10.0:
        raise SystemExit("amplitude must be in (0, 10) rad/s")
    if args.period < 1.0:
        raise SystemExit("period must be >= 1 s")
    if args.speed_limit > 10.0:
        raise SystemExit("speed_limit must be <= 10 rad/s")
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
    csv_path = rec_dir / f"vel_sine_{stamp}.csv"
    log_path = rec_dir / f"vel_sine_{stamp}.log"
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
    enabled = False
    csv_file = None
    writer = None
    max_tau = {"yaw": 0.0, "pitch": 0.0}
    max_vel = {"yaw": 0.0, "pitch": 0.0}
    max_dev = {"yaw": 0.0, "pitch": 0.0}
    try:
        enabled = True
        for name, motor in motors.items():
            initial_modes[name] = motor.read_register(0x0A, timeout=0.75)
            raw = motor.read_register(0x50, timeout=0.75)
            centers[name] = cfg.motors[name].direction * f32(raw) + cfg.motors[name].zero_offset_rad
            motor.disable()
            if initial_modes[name] != 3:
                if not motor.set_control_mode(3):
                    raise RuntimeError(f"{name}: failed to enter velocity mode")
                if motor.read_register(0x0A, timeout=0.5) != 3:
                    raise RuntimeError(f"{name}: velocity mode readback failed")
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

        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow([
            "t",
            "yaw_vcmd", "yaw_vel", "yaw_vest", "yaw_tau", "yaw_pos", "yaw_dev", "yaw_t", "yaw_hf",
            "pitch_vcmd", "pitch_vel", "pitch_vest", "pitch_tau", "pitch_pos", "pitch_dev", "pitch_t", "pitch_hf",
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
            "start: centers=%s amp=%.3f rad/s period=%.2f speed_limit=%.2f "
            "vibration_limit=%.2f torque_limit=%.2f",
            centers, args.amplitude, args.period, args.speed_limit,
            args.vibration_limit, args.torque_limit,
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
            vcmd = env * args.amplitude * math.sin(omega * t)
            for motor in motors.values():
                motor.send_velocity(vcmd)

            snaps = {name: motor.snapshot() for name, motor in motors.items()}
            dt_loop = max(1e-6, now - last_cycle)
            last_cycle = now
            for name, s in snaps.items():
                vest = est[name].update(s.pos, now)
                dev = s.pos - centers[name]
                max_vel[name] = max(max_vel[name], abs(vest))
                max_tau[name] = max(max_tau[name], abs(s.tau))
                max_dev[name] = max(max_dev[name], abs(dev))
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
                elif abs(dev) > args.position_limit:
                    stops.append(f"{name}: position drift {dev:+.3f} > {args.position_limit}")
                    stop_event = True
                elif max(s.t_mos, s.t_rotor) > 80.0:
                    stops.append(f"{name}: temperature {s.t_mos}/{s.t_rotor}")
                    stop_event = True
            if stop_event:
                break

            if now - last_csv >= 0.01:
                last_csv = now
                y, p = snaps["yaw"], snaps["pitch"]
                writer.writerow([
                    f"{t:.4f}",
                    f"{vcmd:.6f}", f"{y.vel:.6f}", f"{est['yaw'].value:.6f}", f"{y.tau:.6f}",
                    f"{y.pos:.6f}", f"{y.pos - centers['yaw']:.6f}", f"{y.t_mos:.1f}", f"{hf_value['yaw']:.4f}",
                    f"{vcmd:.6f}", f"{p.vel:.6f}", f"{est['pitch'].value:.6f}", f"{p.tau:.6f}",
                    f"{p.pos:.6f}", f"{p.pos - centers['pitch']:.6f}", f"{p.t_mos:.1f}", f"{hf_value['pitch']:.4f}",
                ])
                if loop % 50 == 0:
                    csv_file.flush()

            if now - last_print >= 1.0:
                last_print = now
                LOG.info(
                    "t=%5.1f vcmd=%+.3f | yaw vel=%+.3f vest=%+.3f dev=%+.3f tau=%+.3f hf=%.2f | "
                    "pitch vel=%+.3f vest=%+.3f dev=%+.3f tau=%+.3f hf=%.2f | maxT %.2f/%.2f",
                    t, vcmd,
                    snaps["yaw"].vel, est["yaw"].value, snaps["yaw"].pos - centers["yaw"],
                    snaps["yaw"].tau, hf_value["yaw"],
                    snaps["pitch"].vel, est["pitch"].value, snaps["pitch"].pos - centers["pitch"],
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
                for _ in range(5):
                    for motor in motors.values():
                        motor.send_velocity(0.0)
                    time.sleep(0.02)
                for name, motor in motors.items():
                    try:
                        motor.disable()
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
        print(f"max_tau: {max_tau}  max_vel: {max_vel}  max_dev: {max_dev}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
