"""Write the drive position-loop gains and save them to flash.

Uses the official DM_CAN "save all parameters to flash" command
(0x7FF, ``[id_lo, id_hi, 0xAA, 0, 0, 0, 0, 0]``) on the project's verified
serial USB2CAN path (cmd=0x01).  DMTool 2.1.9.4 cannot be used with the
current module firmware because it sends serial cmd=0x03.

The motors stay disabled for the whole run and are restored to their initial
control mode afterwards.  No motion command is sent.

Expected result (config/gimbal.example.yaml):
  yaw   KP_APR=20, KI_APR=0
  pitch KP_APR=54, KI_APR=0
"""

from __future__ import annotations

import argparse
import logging
import struct
import time

from dm6220_gimbal.config import load_config
from dm6220_gimbal.motor import CanRouter, DmMotor
from dm6220_gimbal.transport import SerialUsb2CanBus

LOG = logging.getLogger("store_params")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gimbal.example.yaml")
    ap.add_argument("--port", default=None)
    ap.add_argument("--no-store", action="store_true", help="write RAM only, skip flash save")
    ap.add_argument("--verify-delay", type=float, default=1.5, help="wait after flash save")
    return ap.parse_args()


def f32(raw: int) -> float:
    return struct.unpack("<f", int(raw).to_bytes(4, "little"))[0]


def bits(value: float) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def read_float(motor: DmMotor, rid: int, retries: int = 3) -> float:
    last: Exception | None = None
    for _ in range(retries):
        try:
            raw = motor.read_register(rid, timeout=0.6)
            if raw is not None:
                return f32(raw)
        except Exception as exc:  # timeout while the drive reboots
            last = exc
        time.sleep(0.3)
    raise RuntimeError(f"{motor.name}: failed to read RID 0x{rid:02X}: {last}")


def write_float(motor: DmMotor, rid: int, value: float) -> None:
    if not motor.write_register(rid, bits(value), timeout=0.6):
        raise RuntimeError(f"{motor.name}: failed to write RID 0x{rid:02X}")


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
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

    initial_modes: dict[str, int | None] = {}
    try:
        for name, motor in motors.items():
            initial_modes[name] = motor.read_register(0x0A, timeout=0.75)
            motor.disable()
            time.sleep(0.05)
            print(
                f"{name}: mode={initial_modes[name]} "
                f"MST=0x{motor.read_register(0x07, timeout=0.5):02X} "
                f"ESC=0x{motor.read_register(0x08, timeout=0.5):02X} "
                f"timeout={motor.read_register(0x09, timeout=0.5)}"
            )

        changed: dict[str, dict[str, tuple[float, float]]] = {}
        for name, motor in motors.items():
            mcfg = cfg.motors[name]
            before = {
                "kp_apr": read_float(motor, 0x1B),
                "ki_apr": read_float(motor, 0x1C),
            }
            target = {
                "kp_apr": mcfg.position_kp_apr,
                "ki_apr": mcfg.position_ki_apr,
            }
            for key, rid in (("kp_apr", 0x1B), ("ki_apr", 0x1C)):
                write_float(motor, rid, target[key])
                readback = read_float(motor, rid)
                if abs(readback - target[key]) > 1e-4:
                    raise RuntimeError(
                        f"{name}: RID 0x{rid:02X} readback {readback} != {target[key]}"
                    )
            changed[name] = {
                key: (before[key], target[key]) for key in target
            }
            print(
                f"{name}: RAM update "
                f"KP_APR {before['kp_apr']:.3f} -> {target['kp_apr']:.3f}, "
                f"KI_APR {before['ki_apr']:.3f} -> {target['ki_apr']:.3f}"
            )

        if args.no_store:
            print("--no-store: flash save skipped")
        else:
            for name, motor in motors.items():
                motor.disable()
                motor.store_parameters()
                print(f"{name}: flash save sent (0x7FF, id=0x{motor.can_id:02X}, 0xAA)")
            time.sleep(args.verify_delay)
            for name, motor in motors.items():
                kp = read_float(motor, 0x1B, retries=6)
                ki = read_float(motor, 0x1C, retries=6)
                expect = changed[name]
                ok = (
                    abs(kp - expect["kp_apr"][1]) < 1e-4
                    and abs(ki - expect["ki_apr"][1]) < 1e-4
                )
                print(
                    f"{name}: after save KP_APR={kp:.3f} KI_APR={ki:.3f} "
                    f"({'OK' if ok else 'MISMATCH'})"
                )

        for name, motor in motors.items():
            motor.clear_error()
            if initial_modes.get(name) is not None:
                motor.write_register(0x0A, int(initial_modes[name]), timeout=0.5)
            print(f"{name}: restored mode={initial_modes.get(name)}, disabled")
        return 0
    finally:
        router.stop()
        bus.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
