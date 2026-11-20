"""DM6220 MIT protocol and serial USB2CAN framing."""

from __future__ import annotations

import struct
from dataclasses import dataclass

CAN_ID_REGISTER = 0x7FF
REG_READ = 0x33
REG_WRITE = 0x55
REG_STORE = 0xAA

SYSTEM_ENABLE = bytes.fromhex("FF FF FF FF FF FF FF FC")
SYSTEM_DISABLE = bytes.fromhex("FF FF FF FF FF FF FF FD")
SYSTEM_SET_ZERO = bytes.fromhex("FF FF FF FF FF FF FF FE")
SYSTEM_CLEAR_ERROR = bytes.fromhex("FF FF FF FF FF FF FF FB")

STATUS_NAMES = {
    0x0: "disabled",
    0x1: "enabled",
    0x2: "motor_encoder_error",
    0x3: "output_encoder_error",
    0x5: "encoder_read_error",
    0x7: "encoder_read_error_2",
    0x8: "over_voltage",
    0x9: "under_voltage",
    0xA: "over_current",
    0xB: "mos_over_temperature",
    0xC: "rotor_over_temperature",
    0xD: "communication_lost",
    0xE: "overload",
}


def status_name(status: int) -> str:
    return STATUS_NAMES.get(status, f"unknown_0x{status:X}")


@dataclass(frozen=True)
class MitLimits:
    p_max: float
    v_max: float
    t_max: float
    kp_max: float = 500.0
    kd_max: float = 5.0


@dataclass(frozen=True)
class MotorFeedback:
    status: int
    motor_id: int
    pos: float
    vel: float
    tau: float
    t_mos: float
    t_rotor: float
    raw_pos: int
    raw_vel: int
    raw_tau: int


@dataclass(frozen=True)
class Usb2CanFrame:
    cmd: int
    can_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    value = clamp(value, low, high)
    span = high - low
    if span <= 0:
        raise ValueError("high must be greater than low")
    return int((value - low) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    maximum = (1 << bits) - 1
    value = max(0, min(maximum, value))
    return low + value / maximum * (high - low)


def encode_mit(
    pos: float,
    vel: float,
    kp: float,
    kd: float,
    tau: float,
    limits: MitLimits,
) -> bytes:
    """Pack an MIT command into the 8-byte DM payload."""
    p = float_to_uint(pos, -limits.p_max, limits.p_max, 16)
    v = float_to_uint(vel, -limits.v_max, limits.v_max, 12)
    kp_u = float_to_uint(kp, 0.0, limits.kp_max, 12)
    kd_u = float_to_uint(kd, 0.0, limits.kd_max, 12)
    tau_u = float_to_uint(tau, -limits.t_max, limits.t_max, 12)
    return bytes(
        [
            (p >> 8) & 0xFF,
            p & 0xFF,
            (v >> 4) & 0xFF,
            ((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F),
            tau_u & 0xFF,
        ]
    )


def decode_feedback(data: bytes, limits: MitLimits) -> MotorFeedback:
    """Decode the 8-byte DM feedback payload."""
    if len(data) < 8:
        raise ValueError(f"feedback payload must be 8 bytes, got {len(data)}")
    status = data[0] >> 4
    motor_id = data[0] & 0x0F
    pos_u = (data[1] << 8) | data[2]
    vel_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    return MotorFeedback(
        status=status,
        motor_id=motor_id,
        pos=uint_to_float(pos_u, -limits.p_max, limits.p_max, 16),
        vel=uint_to_float(vel_u, -limits.v_max, limits.v_max, 12),
        tau=uint_to_float(tau_u, -limits.t_max, limits.t_max, 12),
        t_mos=float(data[6]),
        t_rotor=float(data[7]),
        raw_pos=pos_u,
        raw_vel=vel_u,
        raw_tau=tau_u,
    )


def encode_velocity(velocity_rad_s: float) -> bytes:
    """Velocity-mode payload (0x200 + can_id): float32 LE + 4 zero bytes."""
    return struct.pack("<f", velocity_rad_s) + b"\x00\x00\x00\x00"


def encode_pos_vel(position_rad: float, velocity_limit_rad_s: float) -> bytes:
    """Position-velocity payload (0x100 + can_id): float32 pos + float32 limit."""
    return struct.pack("<f", position_rad) + struct.pack("<f", abs(velocity_limit_rad_s))


def encode_register_read(can_id: int, rid: int) -> bytes:
    return bytes(
        [
            can_id & 0xFF,
            (can_id >> 8) & 0xFF,
            REG_READ,
            rid & 0xFF,
            0,
            0,
            0,
            0,
        ]
    )


def encode_register_write(can_id: int, rid: int, value: int) -> bytes:
    return bytes(
        [
            can_id & 0xFF,
            (can_id >> 8) & 0xFF,
            REG_WRITE,
            rid & 0xFF,
        ]
    ) + int(value).to_bytes(4, "little", signed=False)


def encode_store(can_id: int) -> bytes:
    return bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, REG_STORE, 0x01, 0, 0, 0, 0])


def pack_usb2can_frame(can_id: int, data: bytes, cmd: int = 0x01) -> bytes:
    """Pack one classic CAN frame into the 30-byte serial USB2CAN frame.

    Confirmed on 2026-11-20: cmd=0x01, CAN ID in bytes 13..14, DLEN in byte 18,
    CRC byte 0.  The 4-byte CAN ID layout from the public table was not accepted
    by the module firmware.
    """
    if not 0 <= can_id <= 0x7FF:
        raise ValueError(f"standard CAN ID out of range: 0x{can_id:X}")
    if len(data) > 8:
        raise ValueError("classic CAN data must be <= 8 bytes")
    f = bytearray(30)
    f[0:2] = b"\x55\xAA"
    f[2] = 0x1E
    f[3] = cmd & 0xFF
    f[4:8] = (1).to_bytes(4, "little")       # send once
    f[8:10] = (10).to_bytes(2, "little")      # interval ms
    f[10] = 0x00                              # standard frame
    f[11] = 0x00
    f[12] = 0x00
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[15] = 0x00                              # data frame
    f[16] = 0x00                              # idAcc
    f[17] = 0x00                              # dataAcc
    f[18] = len(data)
    f[19:21] = b"\x00\x00"
    f[21:29] = data.ljust(8, b"\x00")
    f[29] = 0x00
    return bytes(f)


def parse_usb2can_frames(buffer: bytearray) -> tuple[list[Usb2CanFrame], bytearray]:
    """Parse as many complete 16-byte RX frames as possible.

    Mutates `buffer` by removing consumed bytes; returns (frames, remaining).
    """
    frames: list[Usb2CanFrame] = []
    while True:
        i = buffer.find(b"\xAA")
        if i < 0:
            buffer.clear()
            break
        if i:
            del buffer[:i]
        if len(buffer) < 16:
            break
        pkt = bytes(buffer[:16])
        if pkt[15] != 0x55:
            del buffer[:1]
            continue
        del buffer[:16]
        fmt = pkt[2]
        dlen = min(fmt & 0x3F, 8)
        frames.append(
            Usb2CanFrame(
                cmd=pkt[1],
                can_id=int.from_bytes(pkt[3:7], "little"),
                data=pkt[7 : 7 + dlen],
                is_extended=bool(fmt & 0x40),
                is_remote=bool(fmt & 0x80),
            )
        )
    return frames, buffer
