"""Continuous dual-axis MIT oscillation + disturbance test.

Both axes follow the same sinusoidal position target around their current
positions (forward/reverse).  The MIT position loop resists external loads;
torque is allowed to rise (hard stop at 2 Nm) while speed is hard-limited to
0.9 rad/s using position-differentiated velocity.

The test runs until:
  * recordings/STOP appears (graceful, preferred),
  * --duration expires, or
  * a safety limit is hit.

Live tuning: edit recordings/tune.json while it runs:
    {"yaw_kp": 0.5, "yaw_kd": 0.1, "pitch_kp": 3.0, "pitch_kd": 0.3}

Usage:
    python scripts/mit_oscillation_test.py --port COM10
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

LOG = logging.getLogger("mit_osc")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gimbal.example.yaml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--amplitude", type=float, default=0.2, help="rad")
    ap.add_argument("--period", type=float, default=4.0, help="s")
    ap.add_argument("--yaw-kp", type=float, default=0.5)
    ap.add_argument("--yaw-kd", type=float, default=0.1)
    ap.add_argument("--pitch-kp", type=float, default=3.0)
    ap.add_argument("--pitch-kd", type=float, default=0.3)
    ap.add_argument("--speed-limit", type=float, default=0.98, help="rad/s hard stop (position-derived)")
    ap.add_argument("--vel-window", type=float, default=0.1, help="s, position-difference window")
    ap.add_argument("--raw-speed-limit", type=float, default=50.0, help="raw telemetry backstop")
    ap.add_argument("--vibration-limit", type=float, default=1.0, help="HF raw-velocity RMS, rad/s")
    ap.add_argument("--vibration-window", type=float, default=0.2, help="s")
    ap.add_argument("--torque-limit", type=float, default=2.0, help="N*m hard stop")
    ap.add_argument("--position-limit", type=float, default=0.6, help="rad from center")
    ap.add_argument("--duration", type=float, default=0.0, help="0 = until stop")
    ap.add_argument("--rate", type=float, default=500.0)
    ap.add_argument("--record-dir", default="recordings")
    return ap.parse_args()


def f32(raw: int) -> float:
    return struct.unpack("<f", int(raw).to_bytes(4, "little"))[0]


def main() -> int:
    args = parse_args()
    if not (0.02 <= args.amplitude <= 0.5):
        raise SystemExit("amplitude must be in [0.02, 0.5] rad")
    if not (1.0 <= args.period <= 20.0):
        raise SystemExit("period must be in [1, 20] s")
    if args.speed_limit > 1.0:
        raise SystemExit("speed_limit must be <= 1 rad/s")
    if args.vibration_limit <= 0 or args.vibration_window <= 0:
        raise SystemExit("vibration_limit/window must be positive")
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = load_config(args.config)
    port = args.port or cfg.can.port
    bus = SerialUsb2CanBus(
        port=port,
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
    tune_file = rec_dir / "tune.json"
    stop_file.unlink(missing_ok=True)
    tune_file.write_text(
        json.dumps(
            {
                "yaw_kp": args.yaw_kp,
                "yaw_kd": args.yaw_kd,
                "pitch_kp": args.pitch_kp,
                "pitch_kd": args.pitch_kd,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = rec_dir / f"mit_osc_{stamp}.csv"
    log_path = rec_dir / f"mit_osc_{stamp}.log"
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(file_handler)

    stops: list[str] = []
    stop_event = False

    def _signal(_sig, _frame):
        nonlocal stop_event
        stop_event = True
        stops.append("signal")

    signal.signal(signal.SIGINT, _signal)
    signal.signal(signal.SIGTERM, _signal)

    centers: dict[str, float] = {}
    initial_modes: dict[str, int] = {}
    enabled = False
    csv_file = None
    writer = None
    max_tau = {"yaw": 0.0, "pitch": 0.0}
    max_vel = {"yaw": 0.0, "pitch": 0.0}
    max_pos_err = {"yaw": 0.0, "pitch": 0.0}
    try:
        # Enable the cleanup path even if motor setup fails halfway through.
        enabled = True
        for name, motor in motors.items():
            initial_modes[name] = motor.read_register(0x0A, timeout=0.75)
            raw = motor.read_register(0x50, timeout=0.75)
            center_joint = cfg.motors[name].direction * f32(raw) + cfg.motors[name].zero_offset_rad
            centers[name] = center_joint
            motor.disable()
            if not motor.set_control_mode(1):
                raise RuntimeError(f"{name}: failed to enter MIT mode")
            motor.clear_error()
            motor.write_register(0x09, 20000, timeout=0.5)
            motor.enable()
        enabled = True
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(m.snapshot().status == 1 for m in motors.values()):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("motors did not enable")

        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "t",
                "yaw_target", "yaw_pos", "yaw_err", "yaw_vel_cmd", "yaw_vel", "yaw_vel_est", "yaw_tau", "yaw_t",
                "pitch_target", "pitch_pos", "pitch_err", "pitch_vel_cmd", "pitch_vel", "pitch_vel_est", "pitch_tau", "pitch_t",
                "yaw_hf", "pitch_hf",
                "yaw_kp", "yaw_kd", "pitch_kp", "pitch_kd",
            ]
        )

        yaw_kp, yaw_kd = args.yaw_kp, args.yaw_kd
        pitch_kp, pitch_kd = args.pitch_kp, args.pitch_kd
        last_tune = 0.0
        last_print = 0.0
        last_csv = 0.0
        last_stop_check = 0.0
        t0 = time.monotonic()
        period = args.rate
        next_t = time.perf_counter()
        loop = 0
        est = {name: VelocityEstimator(args.vel_window, 10.0) for name in motors}
        hf_lp = {name: LowPass(10.0) for name in motors}
        hf_hist: dict[str, deque[tuple[float, float]]] = {name: deque() for name in motors}
        hf_value = {name: 0.0 for name in motors}
        last_cycle = time.monotonic()
        LOG.info(
            "start: centers=%s amplitude=%.3f period=%.2f speed_limit=%.2f "
            "torque_limit=%.2f vibration_limit=%.2f (HF RMS %.2fs)",
            centers, args.amplitude, args.period, args.speed_limit,
            args.torque_limit, args.vibration_limit, args.vibration_window,
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
            if now - last_tune >= 0.5:
                last_tune = now
                try:
                    data = json.loads(tune_file.read_text(encoding="utf-8"))
                    yaw_kp = max(0.0, min(50.0, float(data.get("yaw_kp", yaw_kp))))
                    yaw_kd = max(0.0, min(5.0, float(data.get("yaw_kd", yaw_kd))))
                    pitch_kp = max(0.0, min(50.0, float(data.get("pitch_kp", pitch_kp))))
                    pitch_kd = max(0.0, min(5.0, float(data.get("pitch_kd", pitch_kd))))
                except Exception as exc:
                    LOG.warning("tune file read failed: %s", exc)

            omega = 2.0 * math.pi / args.period
            env = min(1.0, t / 1.0)
            offset = env * args.amplitude * math.sin(omega * t)
            vel_ff = env * args.amplitude * omega * math.cos(omega * t)
            targets = {
                "yaw": centers["yaw"] + offset,
                "pitch": centers["pitch"] + offset,
            }
            gains = {
                "yaw": (yaw_kp, yaw_kd),
                "pitch": (pitch_kp, pitch_kd),
            }
            for name, motor in motors.items():
                kp, kd = gains[name]
                motor.send_mit(targets[name], vel_ff, kp, kd, 0.0)

            snaps = {name: motor.snapshot() for name, motor in motors.items()}
            dt_loop = max(1e-6, now - last_cycle)
            last_cycle = now
            for name, s in snaps.items():
                v_est = est[name].update(s.pos, now)
                max_vel[name] = max(max_vel[name], abs(v_est))
                max_tau[name] = max(max_tau[name], abs(s.tau))
                err = targets[name] - s.pos
                max_pos_err[name] = max(max_pos_err[name], abs(err))

                # 10 Hz high-pass of the raw velocity + rolling RMS.  The 46 Hz
                # limit cycle is invisible in the 100 ms position-derived speed
                # but shows up clearly here.
                lp = hf_lp[name].update(s.vel, dt_loop)
                hf = s.vel - lp
                dq = hf_hist[name]
                dq.append((now, hf))
                while dq and now - dq[0][0] > args.vibration_window:
                    dq.popleft()
                if len(dq) >= 5 and now - dq[0][0] >= args.vibration_window * 0.75:
                    hf_rms = math.sqrt(sum(v * v for _, v in dq) / len(dq))
                else:
                    hf_rms = 0.0
                hf_value[name] = hf_rms

                if not math.isfinite(s.last_rx_age) or s.last_rx_age > 0.1:
                    stops.append(f"{name}: feedback timeout")
                    stop_event = True
                elif s.status != 1:
                    stops.append(f"{name}: status={s.status}")
                    stop_event = True
                elif hf_rms > args.vibration_limit:
                    stops.append(
                        f"{name}: HF vibration RMS {hf_rms:.3f} > {args.vibration_limit}"
                    )
                    stop_event = True
                elif abs(v_est) > args.speed_limit:
                    stops.append(f"{name}: speed {v_est:+.3f} > {args.speed_limit}")
                    stop_event = True
                elif abs(s.vel) > args.raw_speed_limit:
                    stops.append(f"{name}: raw speed {s.vel:+.3f} > {args.raw_speed_limit}")
                    stop_event = True
                elif abs(s.tau) > args.torque_limit:
                    stops.append(f"{name}: torque {s.tau:+.3f} > {args.torque_limit}")
                    stop_event = True
                elif abs(s.pos - centers[name]) > args.position_limit:
                    stops.append(f"{name}: position drift {s.pos - centers[name]:+.3f}")
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
                writer.writerow(
                    [
                        f"{t:.4f}",
                        f"{yt:.6f}", f"{y.pos:.6f}", f"{yt - y.pos:.6f}", f"{vel_ff:.6f}", f"{y.vel:.6f}", f"{est['yaw'].value:.6f}", f"{y.tau:.6f}", f"{y.t_mos:.1f}",
                        f"{pt:.6f}", f"{p.pos:.6f}", f"{pt - p.pos:.6f}", f"{vel_ff:.6f}", f"{p.vel:.6f}", f"{est['pitch'].value:.6f}", f"{p.tau:.6f}", f"{p.t_mos:.1f}",
                        f"{hf_value['yaw']:.4f}", f"{hf_value['pitch']:.4f}",
                        f"{yaw_kp:.3f}", f"{yaw_kd:.3f}", f"{pitch_kp:.3f}", f"{pitch_kd:.3f}",
                    ]
                )
                if loop % 50 == 0:
                    csv_file.flush()

            if now - last_print >= 1.0:
                last_print = now
                LOG.info(
                    "t=%5.1f yaw tgt=%+.3f pos=%+.3f err=%+.3f vel=%+.3f tau=%+.3f | "
                    "pitch tgt=%+.3f pos=%+.3f err=%+.3f vel=%+.3f tau=%+.3f | hf=%.2f/%.2f | kp/kd %.2f/%.2f %.2f/%.2f | maxT %.2f/%.2f",
                    t, targets["yaw"], snaps["yaw"].pos, targets["yaw"] - snaps["yaw"].pos,
                    est["yaw"].value, snaps["yaw"].tau,
                    targets["pitch"], snaps["pitch"].pos, targets["pitch"] - snaps["pitch"].pos,
                    est["pitch"].value, snaps["pitch"].tau,
                    hf_value["yaw"], hf_value["pitch"],
                    yaw_kp, yaw_kd, pitch_kp, pitch_kd, max_tau["yaw"], max_tau["pitch"],
                )

            loop += 1
            next_t += 1.0 / period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

    finally:
        try:
            if enabled:
                for _ in range(3):
                    for motor in motors.values():
                        motor.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
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
            if file_handler:
                logging.getLogger().removeHandler(file_handler)
                file_handler.close()
        LOG.info("stopped: %s", stops or ["unknown"])
        print(f"CSV: {csv_path}")
        print(f"LOG: {log_path}")
        print(f"max_tau: {max_tau}  max_vel: {max_vel}  max_pos_err: {max_pos_err}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
