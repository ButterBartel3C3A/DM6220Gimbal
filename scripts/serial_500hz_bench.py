"""500 Hz serial/CAN benchmark with both motors in native POS_VEL hold.

Safety:
- Only holds the current positions (no intentional motion).
- hard abort on |velocity| > 0.5 rad/s, |torque| > 0.3 Nm, status != 1.
- 1 s CAN watchdog installed during the test (RAM only), restored afterwards.
- both motors are disabled and returned to velocity mode (3) in all paths.

Usage:
    python scripts/serial_500hz_bench.py --port COM10 --cycles 1500
"""

from __future__ import annotations

import argparse
import statistics
import struct
import threading
import time

import serial

PMAX, VMAX, TMAX = 12.5, 45.0, 10.0
KP_MAX, KD_MAX = 500.0, 5.0
YAW_CAN_ID, YAW_MST_ID = 0x01, 0x11
PITCH_CAN_ID, PITCH_MST_ID = 0x02, 0x12

ABORT_SPEED = 0.5
ABORT_TORQUE = 0.3


def pack_frame(can_id: int, data: bytes, cmd: int = 0x01) -> bytes:
    if len(data) > 8:
        raise ValueError("classic CAN data must be <= 8 bytes")
    f = bytearray(30)
    f[0:2] = b"\x55\xAA"
    f[2] = 0x1E
    f[3] = cmd
    f[4:8] = (1).to_bytes(4, "little")
    f[8:10] = (10).to_bytes(2, "little")
    f[10] = 0
    f[11] = 0
    f[12] = 0
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[15] = 0
    f[16] = 0
    f[17] = 0
    f[18] = len(data)
    f[19:21] = b"\x00\x00"
    f[21:29] = data.ljust(8, b"\x00")
    f[29] = 0
    return bytes(f)


def f2u(x: float, xmin: float, xmax: float, bits: int) -> int:
    x = max(xmin, min(xmax, x))
    return int((x - xmin) * ((1 << bits) - 1) / (xmax - xmin))


def encode_mit(pos: float, vel: float, kp: float, kd: float, tau: float = 0.0) -> bytes:
    p = f2u(pos, -PMAX, PMAX, 16)
    v = f2u(vel, -VMAX, VMAX, 12)
    kpu = f2u(kp, 0.0, KP_MAX, 12)
    kdu = f2u(kd, 0.0, KD_MAX, 12)
    tu = f2u(tau, -TMAX, TMAX, 12)
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


def encode_pos_vel(pos: float, velocity_limit: float) -> bytes:
    return struct.pack("<f", pos) + struct.pack("<f", abs(velocity_limit))


def decode_feedback(pkt: bytes):
    d = pkt[7:15]
    status = d[0] >> 4
    mid = d[0] & 0x0F
    pu = (d[1] << 8) | d[2]
    vu = (d[3] << 4) | (d[4] >> 4)
    tu = ((d[4] & 0x0F) << 8) | d[5]
    return (
        status,
        mid,
        pu / 65535 * 2 * PMAX - PMAX,
        vu / 4095 * 2 * VMAX - VMAX,
        tu / 4095 * 2 * TMAX - TMAX,
        d[6],
        d[7],
    )


class RxThread(threading.Thread):
    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self.latest: dict[int, tuple] = {}
        self.max_vel = {YAW_CAN_ID: 0.0, PITCH_CAN_ID: 0.0}
        self.max_tau = {YAW_CAN_ID: 0.0, PITCH_CAN_ID: 0.0}
        self.rx_frames = {YAW_CAN_ID: 0, PITCH_CAN_ID: 0}
        self.acks = 0
        self.bad = 0
        self.abort_reason: str | None = None

    def run(self):
        buf = bytearray()
        while not self.stop_flag.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as e:
                self.abort_reason = f"serial read error: {e}"
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                i = buf.find(b"\xAA")
                if i < 0 or len(buf) - i < 16:
                    if i > 0:
                        del buf[:i]
                    break
                pkt = bytes(buf[i : i + 16])
                if pkt[15] != 0x55:
                    del buf[: i + 1]
                    self.bad += 1
                    continue
                del buf[: i + 16]
                cmd = pkt[1]
                if cmd == 0x12:
                    self.acks += 1
                elif cmd == 0x11:
                    try:
                        status, mid, pos, vel, tau, tm, tr = decode_feedback(pkt)
                    except Exception:
                        self.bad += 1
                        continue
                    if mid in (YAW_CAN_ID, PITCH_CAN_ID):
                        with self.lock:
                            self.latest[mid] = (status, pos, vel, tau, tm, tr)
                            self.rx_frames[mid] += 1
                            self.max_vel[mid] = max(self.max_vel[mid], abs(vel))
                            self.max_tau[mid] = max(self.max_tau[mid], abs(tau))
                            if status != 1:
                                self.abort_reason = f"motor {mid} status={status}"
                            elif abs(vel) > ABORT_SPEED:
                                self.abort_reason = f"motor {mid} over-speed {vel:.3f}"
                            elif abs(tau) > ABORT_TORQUE:
                                self.abort_reason = f"motor {mid} over-torque {tau:.3f}"


class Bus:
    def __init__(self, port: str):
        self.ser = serial.Serial(
            port, 921600, bytesize=8, parity="N", stopbits=1,
            timeout=0.01, write_timeout=0.1,
        )

    def send_sync(self, can_id: int, data: bytes, wait: float = 0.25) -> list[bytes]:
        self.ser.reset_input_buffer()
        self.ser.write(pack_frame(can_id, data))
        self.ser.flush()
        buf = bytearray()
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            buf.extend(self.ser.read(256))
        out = []
        while True:
            i = buf.find(b"\xAA")
            if i < 0 or len(buf) - i < 16:
                break
            pkt = bytes(buf[i : i + 16])
            if pkt[15] == 0x55:
                out.append(pkt)
                del buf[: i + 16]
            else:
                del buf[: i + 1]
        return out

    def read_reg(self, can_id: int, rid: int):
        data = bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
        for pkt in self.send_sync(0x7FF, data, 0.25):
            if pkt[1] == 0x11:
                d = pkt[7:15]
                if d[0] == (can_id & 0xFF) and d[1] == ((can_id >> 8) & 0xFF) and d[2] == 0x33 and d[3] == rid:
                    return int.from_bytes(d[4:8], "little")
        return None

    def write_reg(self, can_id: int, rid: int, value: int):
        data = bytes([can_id & 0xFF, (can_id >> 8) & 0xFF, 0x55, rid]) + value.to_bytes(4, "little")
        self.send_sync(0x7FF, data, 0.12)

    def close(self):
        self.ser.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--cycles", type=int, default=1500)
    ap.add_argument("--rate", type=float, default=500.0)
    args = ap.parse_args()
    if args.rate <= 0 or args.rate > 1000:
        raise SystemExit("rate must be in (0, 1000] Hz")

    bus = Bus(args.port)
    yaw_mode = pitch_mode = None
    yaw_timeout = pitch_timeout = None
    yaw_pos = pitch_pos = None
    rx = None
    enabled = False
    try:
        yaw_mode = bus.read_reg(YAW_CAN_ID, 0x0A)
        pitch_mode = bus.read_reg(PITCH_CAN_ID, 0x0A)
        yaw_timeout = bus.read_reg(YAW_CAN_ID, 0x09)
        pitch_timeout = bus.read_reg(PITCH_CAN_ID, 0x09)
        yaw_pos_raw = bus.read_reg(YAW_CAN_ID, 0x50)
        pitch_pos_raw = bus.read_reg(PITCH_CAN_ID, 0x50)
        if None in (yaw_mode, pitch_mode, yaw_timeout, pitch_timeout, yaw_pos_raw, pitch_pos_raw):
            raise RuntimeError("failed to read initial state")
        yaw_pos = struct.unpack("<f", yaw_pos_raw.to_bytes(4, "little"))[0]
        pitch_pos = struct.unpack("<f", pitch_pos_raw.to_bytes(4, "little"))[0]
        print(
            f"initial: yaw mode={yaw_mode} p_m={yaw_pos:.5f}  "
            f"pitch mode={pitch_mode} p_m={pitch_pos:.5f}"
        )

        bus.send_sync(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        bus.send_sync(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        bus.write_reg(YAW_CAN_ID, 0x0A, 2)
        bus.write_reg(PITCH_CAN_ID, 0x0A, 2)
        bus.write_reg(YAW_CAN_ID, 0x09, 20000)    # 1 s watchdog
        bus.write_reg(PITCH_CAN_ID, 0x09, 20000)
        bus.send_sync(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FC"))
        bus.send_sync(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FC"))
        enabled = True
        time.sleep(0.1)

        bus.ser.reset_input_buffer()
        rx = RxThread(bus.ser)
        rx.start()

        yaw_frame = pack_frame(0x100 + YAW_CAN_ID, encode_pos_vel(yaw_pos, 0.0))
        pitch_frame = pack_frame(0x100 + PITCH_CAN_ID, encode_pos_vel(pitch_pos, 0.0))
        period = 1.0 / args.rate
        start_periods: list[float] = []
        overruns = 0
        t_start = time.perf_counter()
        next_t = t_start
        for i in range(args.cycles):
            now = time.perf_counter()
            if i > 0:
                start_periods.append(now - last_now)
            last_now = now
            if now > next_t + period:
                overruns += 1
            try:
                bus.ser.write(yaw_frame)
                bus.ser.write(pitch_frame)
                bus.ser.flush()
            except Exception as e:
                raise RuntimeError(f"serial write error: {e}")
            if i % 50 == 0 and rx.abort_reason:
                raise RuntimeError(rx.abort_reason)
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                overruns += 1
        elapsed = time.perf_counter() - t_start

        # Keep the watchdog fed while stopping.
        bus.ser.write(pack_frame(0x100 + YAW_CAN_ID, encode_pos_vel(yaw_pos, 0.0)))
        bus.ser.write(pack_frame(0x100 + PITCH_CAN_ID, encode_pos_vel(pitch_pos, 0.0)))
        bus.ser.flush()
        time.sleep(0.05)
        rx.stop_flag.set()
        rx.join(timeout=1.0)
        bus.send_sync(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        bus.send_sync(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        enabled = False

        if rx.abort_reason:
            raise RuntimeError(rx.abort_reason)

        actual_rate = args.cycles / elapsed
        p50 = statistics.median(start_periods) * 1000 if start_periods else 0.0
        p99 = (
            statistics.quantiles(start_periods, n=100)[98] * 1000
            if len(start_periods) >= 100
            else max(start_periods, default=0.0) * 1000
        )
        print(
            f"cycles={args.cycles} elapsed={elapsed:.3f}s actual_rate={actual_rate:.1f} Hz "
            f"period p50={p50:.3f} ms p99={p99:.3f} ms overruns={overruns}"
        )
        with rx.lock:
            for cid, name in ((YAW_CAN_ID, "yaw"), (PITCH_CAN_ID, "pitch")):
                print(
                    f"  {name}: rx_frames={rx.rx_frames[cid]} max_vel={rx.max_vel[cid]:.4f} "
                    f"max_tau={rx.max_tau[cid]:.4f} latest={rx.latest.get(cid)}"
                )
            print(f"  acks={rx.acks} bad_frames={rx.bad}")
        return 0
    finally:
        if rx is not None:
            rx.stop_flag.set()
            rx.join(timeout=1.0)
        try:
            if enabled:
                bus.ser.write(pack_frame(0x100 + YAW_CAN_ID, encode_pos_vel(yaw_pos or 0.0, 0.0)))
                bus.ser.write(pack_frame(0x100 + PITCH_CAN_ID, encode_pos_vel(pitch_pos or 0.0, 0.0)))
                bus.ser.flush()
                time.sleep(0.1)
                bus.send_sync(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
                bus.send_sync(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FD"))
        except Exception as e:
            print("cleanup warning:", e)
        try:
            bus.write_reg(YAW_CAN_ID, 0x0A, 3)
            bus.write_reg(PITCH_CAN_ID, 0x0A, 3)
        except Exception as e:
            print("restore mode warning:", e)
        try:
            if yaw_timeout is not None:
                bus.write_reg(YAW_CAN_ID, 0x09, yaw_timeout)
            if pitch_timeout is not None:
                bus.write_reg(PITCH_CAN_ID, 0x09, pitch_timeout)
            bus.send_sync(YAW_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FB"))
            bus.send_sync(PITCH_CAN_ID, bytes.fromhex("FF FF FF FF FF FF FF FB"))
        except Exception as e:
            print("restore timeout/clear warning:", e)
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
