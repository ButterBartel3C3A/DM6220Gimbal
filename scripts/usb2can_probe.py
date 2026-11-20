"""USB2CAN 串口封装探测（只发 0 rad/s 速度指令）。

2026-11-20 真机确认（DM-USBCU1 + 2×G6220）：
- 可用 TX：cmd=0x01, DLEN=18, CRC=0, CAN ID 为 2 字节放在字节 13..14。
- 模块回 `AA 12 ... 55` 发送成功；电机反馈为 `AA 11 ...`。
- 官方表里的 4 字节 CAN ID（11..14）在本固件上会把 ID 读成 0。

安全约束：
- 只发送速度模式下的 0 rad/s，绝不发送非零速度。
- 不使能/失能、不切换模式、不写参数。
- 目的：确认 55 AA 30 字节帧的偏移/命令/CRC，拿到第一帧反馈。

用法：
    python scripts/usb2can_probe.py COM10
"""

from __future__ import annotations

import struct
import sys
import time

import serial

BAUD = 921600
MAX_VELOCITY_RAD_S = 1.0  # 绝对上限；本脚本只发 0
CAN_ID_MOTOR1_VELOCITY = 0x201  # 0x200 + 0x01


def build_frame(
    can_id: int,
    data: bytes,
    *,
    cmd: int = 0x01,
    dlen_offset: int = 18,
    crc: int = 0x00,
    header: bytes = b"\x55\xAA",
    length: int = 0x1E,
) -> bytes:
    """按 DM_USB2CAN 30 字节模板构造一帧。"""
    if len(data) > 8:
        raise ValueError("classic CAN data must be <= 8 bytes")
    f = bytearray(30)
    f[0:2] = header
    f[2] = length
    f[3] = cmd
    f[4:8] = (1).to_bytes(4, "little")   # send count
    f[8:10] = (10).to_bytes(2, "little")  # interval ms
    f[10] = 0x00                          # standard id
    # 真机确认（2026-11-20）：本模块把 CAN ID 当 2 字节放在 13..14，
    # 11..12 必须为 0。按 4 字节写在 11..14 会被读成 ID=0。
    f[11] = 0x00
    f[12] = 0x00
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[15] = 0x00                          # data frame
    f[16] = 0x00
    f[17] = 0x00
    f[dlen_offset] = len(data)
    f[21:29] = data.ljust(8, b"\x00")
    f[29] = crc
    return bytes(f)


def read_frames(ser: serial.Serial, duration: float = 0.5) -> tuple[list[bytes], bytes]:
    """读 duration 秒，按 AA..55 切 16 字节帧。"""
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
    return frames, bytes(buf)


def describe(pkt: bytes) -> str:
    cmd = pkt[1]
    fmt = pkt[2]
    can_id = int.from_bytes(pkt[3:7], "little")
    dlen = fmt & 0x3F
    data = pkt[7:15][:dlen]
    return f"cmd=0x{cmd:02X} fmt=0x{fmt:02X} id=0x{can_id:03X} len={dlen} data={data.hex(' ')}"


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    zero_velocity = struct.pack("<f", 0.0) + b"\x00\x00\x00\x00"
    assert abs(0.0) <= MAX_VELOCITY_RAD_S

    variants = [
        ("handshake", 0x000, b"", dict(cmd=0x02)),
        ("cmd01 dlen18 crc0", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(cmd=0x01, dlen_offset=18)),
        ("cmd03 dlen18 crc0", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(cmd=0x03, dlen_offset=18)),
        ("cmd03 dlen20 crc0", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(cmd=0x03, dlen_offset=20)),
        ("cmd01 dlen20 crc0", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(cmd=0x01, dlen_offset=20)),
        ("cmd03 dlen18 sum", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(cmd=0x03, dlen_offset=18, crc=0x00)),
        ("header AA55", CAN_ID_MOTOR1_VELOCITY, zero_velocity, dict(header=b"\xAA\x55")),
    ]

    ser = serial.Serial(port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.05)
    print(f"opened {ser.name} @ {BAUD} 8N1")
    try:
        for name, can_id, payload, kwargs in variants:
            frame = build_frame(can_id, payload, **kwargs)
            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()
            frames, tail = read_frames(ser, 0.6)
            print(f"\n[{name}] tx={frame.hex(' ')}")
            print(f"  rx_frames={len(frames)} tail={tail.hex(' ') if tail else '(none)'}")
            for pkt in frames:
                print("  ", pkt.hex(" "), "->", describe(pkt))
            if frames:
                print("\nfirst response found; stopping.")
                return 0
        print("\nno response for any variant.")
        return 1
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
