"""Yaw continuous rotation + pitch MIT hold (bring-up test).

Safety:
- yaw speed hard-limited to 1.0 rad/s; default 0.6 rad/s.
- aborts on over-speed, over-torque, over-temperature, lost feedback, or
  pitch position error.
- restores both motors to velocity mode (3) and disables them in all paths.

Usage:
    python scripts/yaw_continuous_test.py --port COM10 --speed 0.6 --duration 45
"""

from __future__ import annotations

import argparse
import math
import struct
import time

import serial

PMAX, VMAX, TMAX = 12.5, 45.0, 10.0
KP_MAX, KD_MAX = 500.0, 5.0

YAW_CAN_ID, YAW_MST_ID = 0x01, 0x11
PITCH_CAN_ID, PITCH_MST_ID = 0x02, 0x12

MAX_SPEED = 1.0
ABORT_SPEED = 0.95
ABORT_YAW_TORQUE = 0.5
ABORT_PITCH_TORQUE = 0.8
ABORT_TEMP_C = 60
ABORT_PITCH_ERROR = 0.15
LOOP_HZ = 20.0


def pack_frame(can_id: int, data: bytes, cmd: int = 0x01) -> bytes:
    if len(data) > 8:
        raise ValueError("classic CAN data must be <= 8 bytes")
    f = bytearray(30)
    f[0:2] = b"\x55\xAA"
    f[2] = 0x1E
    f[3] = cmd
    f[4:8] = (1).to_bytes(4, "little")
    f[8:10] = (10).to_bytes(2, "little")
    f[10] = 0x00
    f[11] = 0x00
    f[12] = 0x00
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[15] = 0x00
    f[16] = 0x00
    f[17] = 0x00
    f[18] = len(data)
    f[19:21] = b"\x00\x00"
    f[21:29] = data.ljust(8, b"\x00")
    f[29] = 0x00
    return bytes(f)


def read_frames(ser: serial.Serial, duration: float) -> list[bytes]:
    buf = bytearray()
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        buf.extend(ser.read(256))
    frames: list[bytes] = []
    while True:
        i = buf.find(b"\xAA")
        if i < 0 or len(buf) - i < 16:
            break
        pkt = bytes(buf[i : i + 16])
        if pkt[15] == 0x55:
            frames.append(pkt)
            del buf[: i + 16]
        else:
            del buf[: i + 1]
    return frames


def decode_feedback(pkt: bytes):
    d = pkt[7:15]
    status = d[0] >> 4
    motor_id = d[0] & 0x0F
    pos_u = (d[1] << 8) | d[2]
    vel_u = (d[3] << 4) | (d[4] >> 4)
    tau_u = ((d[4] & 0x0F) << 8) | d[5]
    pos = pos_u / 65535 * 2 * PMAX - PMAX
    vel = vel_u / 4095 * 2 * VMAX - VMAX
    tau = tau_u / 4095 * 2 * TMAX - TMAX
    return status, motor_id, pos, vel, tau, d[6], d[7]


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def encode_mit(pos: float, vel: float, kp: float, kd: float, tau: float) -> bytes:
    p = float_to_uint(pos, -PMAX, PMAX, 16)
    v = float_to_uint(vel, -VMAX, VMAX, 12)
    kpu = float_to_uint(kp, 0.0, KP_MAX, 12)
    kdu = float_to_uint(kd, 0.0, KD_MAX, 12)
    tu = float_to_uint(tau, -TMAX, TMAX, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kpu >> 8) & 0x0F),
            kpu & 0xFF,
            (kdu >> 4) & 0xFF,
            ((kdu & 0x0F) << 4) | ((tu >> 8) & 0x0F),
            tu & 0xFF,
        ]
    )


class Bus:
    def __init__(self, port: str):
        self.ser = serial.Serial(
            port, 921600, bytesize=8, parity="N", stopbits=1,
            timeout=0.01, write_timeout=0.1,
        )

    def send(self, can_id: int, data: bytes) -> list[bytes]:
        self.ser.reset_input_buffer()
        self.ser.write(pack_frame(can_id, data))
        self.ser.flush()
        return read_frames(self.ser, 0.035)

    def read_reg(self, can_id: int, rid: int, wait: float = 0.25):
        data = bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
        self.ser.reset_input_buffer()
        self.ser.write(pack_frame(0x7FF, data))
        self.ser.flush()
        for pkt in read_frames(self.ser, wait):
            if pkt[1] == 0x11:
                d = pkt[7:15]
                if (
                    d[0] == (can_id & 0xFF)
                    and d[1] == ((can_id >> 8) & 0xFF)
                    and d[2] == 0x33
                    and d[3] == rid
                ):
                    return int.from_bytes(d[4:8], "little")
        return None

    def write_reg(self, can_id: int, rid: int, value: int):
        data = bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x55, rid]) + value.to_bytes(4, "little")
        self.ser.reset_input_buffer()
        self.ser.write(pack_frame(0x7FF, data))
        self.ser.flush()
        read_frames(self.ser, 0.12)

    def close(self):
        self.ser.close()


def velocity(can_id: int, v: float) -> bytes:
    return pack_frame(0x200 + can_id, struct.pack("<f", v) + b"\x00\x00\x00\x00")


def enable_frame(can_id: int) -> bytes:
    return pack_frame(can_id, bytes.fromhex("FF FF FF FF FF FF FF FC"))


def disable_frame(can_id: int) -> bytes:
    return pack_frame(can_id, bytes.fromhex("FF FF FF FF FF FF FF FD"))


def latest_feedback(frames: list[bytes], motor_id: int):
    out = None
    for pkt in frames:
        if pkt[1] != 0x11:
            continue
        status, mid, pos, vel, tau, tm, tr = decode_feedback(pkt)
        if mid == motor_id:
            out = (status, pos, vel, tau, tm, tr)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--speed", type=float, default=0.6)
    ap.add_argument("--duration", type=float, default=45.0)
    args = ap.parse_args()

    speed = args.speed
    if abs(speed) > MAX_SPEED:
        raise SystemExit(f"speed {speed} exceeds hard limit {MAX_SPEED} rad/s")
    if args.duration <= 0 or args.duration > 180:
        raise SystemExit("duration must be in (0, 180] s")

    bus = Bus(args.port)
    yaw_mode = pitch_mode = None
    yaw_timeout = pitch_timeout = None
    pitch_pos = None
    yaw_enabled = pitch_enabled = False
    try:
        # Read current modes/positions.
        yaw_mode = bus.read_reg(YAW_CAN_ID, 0x0A)
        pitch_mode = bus.read_reg(PITCH_CAN_ID, 0x0A)
        yaw_pos_raw = bus.read_reg(YAW_CAN_ID, 0x50)
        pitch_pos_raw = bus.read_reg(PITCH_CAN_ID, 0x50)
        yaw_timeout = bus.read_reg(YAW_CAN_ID, 0x09)
        pitch_timeout = bus.read_reg(PITCH_CAN_ID, 0x09)
        if None in (yaw_mode, pitch_mode, yaw_pos_raw, pitch_pos_raw, yaw_timeout, pitch_timeout):
            raise RuntimeError("failed to read initial mode/position/timeout")
        pitch_pos = struct.unpack("<f", pitch_pos_raw.to_bytes(4, "little"))[0]
        print(
            f"initial: yaw_mode={yaw_mode} pitch_mode={pitch_mode} "
            f"pitch_pos={pitch_pos:.5f} timeout={yaw_timeout}/{pitch_timeout}"
        )

        # Disable both, then install a 500 ms CAN watchdog (RAM only).
        bus.send(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        bus.send(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        time.sleep(0.1)
        bus.write_reg(YAW_CAN_ID, 0x09, 20000)    # 20000 * 50 us = 1000 ms
        bus.write_reg(PITCH_CAN_ID, 0x09, 20000)
        if bus.read_reg(YAW_CAN_ID, 0x09) != 20000 or bus.read_reg(PITCH_CAN_ID, 0x09) != 20000:
            raise RuntimeError("failed to set CAN timeout watchdog")
        print("CAN timeout watchdog set to 1000 ms")
        bus.write_reg(PITCH_CAN_ID, 0x0A, 1)
        if bus.read_reg(PITCH_CAN_ID, 0x0A) != 1:
            raise RuntimeError("failed to switch pitch to MIT mode")
        bus.send(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FC"))
        pitch_enabled = True
        time.sleep(0.1)
        # Hold pitch at its initial position.
        fb = None
        for _ in range(10):
            fb = latest_feedback(
                bus.send(PITCH_CAN_ID, encode_mit(pitch_pos, 0.0, 1.0, 0.1, 0.0)),
                PITCH_CAN_ID,
            )
            if fb is not None and fb[0] == 1:
                break
            time.sleep(0.05)
        if fb is None or fb[0] != 1:
            raise RuntimeError(f"pitch MIT hold failed: {fb}")
        print(f"pitch hold: status={fb[0]} pos={fb[1]:.5f} vel={fb[2]:.4f} tau={fb[3]:.4f}")

        # Enable yaw in velocity mode.
        bus.send(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FC"))
        yaw_enabled = True
        time.sleep(0.1)

        start_pos = None
        prev_pos = None
        unwrapped = 0.0
        wraps = 0
        max_vel = 0.0
        max_tau = 0.0
        max_temp = 0
        last_yaw_rx = time.monotonic()
        last_yaw = None
        last_pm = None
        t0 = time.monotonic()
        next_print = t0
        next_pm = t0
        print(f"start yaw {speed:.3f} rad/s for {args.duration:.1f}s ...")

        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.duration:
                break

            frames = []
            frames += bus.send(0x200 + YAW_CAN_ID, struct.pack("<f", speed) + b"\x00\x00\x00\x00")
            frames += bus.send(PITCH_CAN_ID, encode_mit(pitch_pos, 0.0, 1.0, 0.1, 0.0))

            yaw_fb = latest_feedback(frames, YAW_CAN_ID)
            pitch_fb = latest_feedback(frames, PITCH_CAN_ID)

            if yaw_fb is not None:
                status, pos, vel, tau, tm, tr = yaw_fb
                last_yaw = yaw_fb
                last_yaw_rx = now
                max_vel = max(max_vel, abs(vel))
                max_tau = max(max_tau, abs(tau))
                max_temp = max(max_temp, tm, tr)
                if start_pos is None:
                    start_pos = prev_pos = pos
                else:
                    delta = pos - prev_pos
                    if delta > PMAX:
                        delta -= 2 * PMAX
                        wraps += 1
                    elif delta < -PMAX:
                        delta += 2 * PMAX
                        wraps += 1
                    unwrapped += delta
                    prev_pos = pos
                if status != 1:
                    raise RuntimeError(f"yaw status changed to {status}")
                if abs(vel) > ABORT_SPEED:
                    raise RuntimeError(f"yaw over-speed: {vel:.3f} rad/s")
                if abs(tau) > ABORT_YAW_TORQUE:
                    raise RuntimeError(f"yaw over-torque: {tau:.3f} Nm")
                if max(tm, tr) > ABORT_TEMP_C:
                    raise RuntimeError(f"yaw over-temperature: {tm}/{tr} C")

            if pitch_fb is not None:
                status, pos, vel, tau, tm, tr = pitch_fb
                if status != 1:
                    raise RuntimeError(f"pitch status changed to {status}")
                if abs(pos - pitch_pos) > ABORT_PITCH_ERROR:
                    raise RuntimeError(f"pitch drift: {pos - pitch_pos:.3f} rad")
                if abs(vel) > ABORT_SPEED:
                    raise RuntimeError(f"pitch over-speed: {vel:.3f} rad/s")
                if abs(tau) > ABORT_PITCH_TORQUE:
                    raise RuntimeError(f"pitch over-torque: {tau:.3f} Nm")
                if max(tm, tr) > ABORT_TEMP_C:
                    raise RuntimeError(f"pitch over-temperature: {tm}/{tr} C")

            if now - last_yaw_rx > 0.5:
                raise RuntimeError("yaw feedback lost")

            # Sample the absolute position register every 2 s.
            if now >= next_pm:
                pm_raw = bus.read_reg(YAW_CAN_ID, 0x50, wait=0.25)
                if pm_raw is not None:
                    last_pm = struct.unpack("<f", pm_raw.to_bytes(4, "little"))[0]
                next_pm = now + 2.0

            if now >= next_print:
                if last_yaw is None or prev_pos is None:
                    print(f"t={elapsed:5.1f}s waiting for yaw feedback ...")
                else:
                    print(
                        f"t={elapsed:5.1f}s yaw_pos={prev_pos:+.4f} raw  "
                        f"unwrapped={unwrapped:+.4f} rad ({unwrapped/(2*math.pi):+.2f} rev)  "
                        f"vel={last_yaw[2]:+.3f} tau={last_yaw[3]:+.4f} "
                        f"T={last_yaw[4]}/{last_yaw[5]}  "
                        f"p_m={last_pm if last_pm is None else f'{last_pm:+.4f}'}  wraps={wraps}"
                    )
                next_print = now + 1.0

            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - now)))

        # Keep pitch alive while stopping yaw.
        bus.send(PITCH_CAN_ID, encode_mit(pitch_pos, 0.0, 0.0, 0.0, 0.0))
        # Stop yaw.
        for _ in range(3):
            bus.send(0x200 + YAW_CAN_ID, struct.pack("<f", 0.0) + b"\x00\x00\x00\x00")
            bus.send(PITCH_CAN_ID, encode_mit(pitch_pos, 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.05)
        bus.send(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        yaw_enabled = False

        print(
            f"done: unwrapped={unwrapped:+.4f} rad ({unwrapped/(2*math.pi):+.2f} rev), "
            f"wraps={wraps}, max_vel={max_vel:.3f}, max_tau={max_tau:.4f}, max_temp={max_temp} C"
        )
        return 0
    finally:
        # Best-effort safe shutdown.
        try:
            if yaw_enabled:
                for _ in range(2):
                    bus.send(0x200 + YAW_CAN_ID, struct.pack("<f", 0.0) + b"\x00\x00\x00\x00")
                bus.send(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
            if pitch_enabled:
                bus.send(PITCH_CAN_ID, encode_mit(pitch_pos or 0.0, 0.0, 0.0, 0.0, 0.0))
                time.sleep(0.1)
                bus.send(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
            # Clear any latched communication-loss fault (0xD) caused by the
            # temporary CAN watchdog. This does not enable or move the motors.
            bus.send(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FB"))
            bus.send(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FB"))
        except Exception as e:
            print("cleanup warning:", e)
        try:
            if pitch_mode is not None:
                bus.write_reg(PITCH_CAN_ID, 0x0A, pitch_mode)
        except Exception as e:
            print("restore pitch mode warning:", e)
        try:
            if yaw_timeout is not None:
                bus.write_reg(YAW_CAN_ID, 0x09, yaw_timeout)
            if pitch_timeout is not None:
                bus.write_reg(PITCH_CAN_ID, 0x09, pitch_timeout)
        except Exception as e:
            print("restore CAN timeout warning:", e)
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
