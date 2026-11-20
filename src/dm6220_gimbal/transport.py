"""CAN transports: serial USB2CAN (primary), SocketCAN/virtual fallbacks."""

from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Any

import can
import serial
from serial.tools import list_ports

from .config import CanConfig
from .protocol import pack_usb2can_frame, parse_usb2can_frames

LOG = logging.getLogger(__name__)

USB2CAN_VID = 0x2E88
USB2CAN_PID = 0x4603


def list_serial_ports() -> list[dict[str, Any]]:
    ports = []
    for p in list_ports.comports():
        ports.append(
            {
                "device": p.device,
                "description": p.description,
                "hwid": p.hwid,
                "vid": p.vid,
                "pid": p.pid,
                "is_usb2can": bool(
                    (p.vid == USB2CAN_VID and p.pid == USB2CAN_PID)
                    or "USB2CAN" in (p.description or "").upper()
                ),
            }
        )
    return ports


def find_usb2can_port() -> str | None:
    candidates = list_serial_ports()
    for p in candidates:
        if p["is_usb2can"]:
            return str(p["device"])
    # Fallback: a generic USB serial device (the adapter may not expose VID/PID).
    for p in candidates:
        desc = (p["description"] or "").upper()
        if "USB" in desc and "蓝牙" not in (p["description"] or "") and "BLUETOOTH" not in desc:
            return str(p["device"])
    return None


class SerialUsb2CanBus(can.BusABC):
    """python-can backend for the DAMIAO serial USB2CAN adapter.

    Serial settings: 921600 8N1.  The adapter wraps classic CAN frames in a
    30-byte ``55 AA`` frame and returns 16-byte ``AA ... 55`` frames.
    """

    def __init__(
        self,
        port: str = "COM10",
        baud: int = 921_600,
        can_bitrate: int = 1_000_000,
        can_baud_index: int = 0,
        set_can_baud: bool = True,
        send_heartbeat: bool = False,
        heartbeat_ms: int = 1000,
        rx_queue_size: int = 4096,
        tx_timeout_ms: int = 5,
        auto_detect: bool = False,
        **kwargs: Any,
    ) -> None:
        if auto_detect:
            detected = find_usb2can_port()
            if detected:
                port = detected
        if not port:
            raise can.CanError("USB2CAN serial port is not configured")
        self._port = port
        self._baud = baud
        self._ser = None
        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.01,
                write_timeout=max(0.001, tx_timeout_ms / 1000.0),
            )
        except Exception as exc:  # pyserial raises different subclasses per OS
            raise can.CanError(f"failed to open USB2CAN port {port}: {exc}") from exc

        super().__init__(channel=port, **kwargs)
        self.channel_info = f"serial_usb2can:{port}@{baud}"
        self._tx_lock = threading.Lock()
        self._rx_queue: queue.Queue[can.Message] = queue.Queue(maxsize=rx_queue_size)
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="usb2can-rx", daemon=True)
        self._heartbeat_thread: threading.Thread | None = None
        self._rx_thread.start()

        self.tx_frames = 0
        self.rx_frames = 0
        self.acks = 0
        self.bad_frames = 0
        self.dropped_rx = 0
        self.last_error: str | None = None

        if set_can_baud:
            self.send_raw(bytes([0x55, 0x05, can_baud_index & 0xFF, 0xAA, 0x55]))
        if send_heartbeat:
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop, args=(heartbeat_ms,), name="usb2can-heartbeat", daemon=True
            )
            self._heartbeat_thread.start()
        LOG.info("opened %s (CAN %d bps)", self.channel_info, can_bitrate)

    def send_raw(self, data: bytes) -> None:
        with self._tx_lock:
            try:
                assert self._ser is not None
                self._ser.write(data)
                self._ser.flush()
            except Exception as exc:
                self.last_error = str(exc)
                raise can.CanError(f"USB2CAN serial write failed: {exc}") from exc

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        if msg.is_extended_id:
            raise can.CanError("serial USB2CAN backend only supports standard CAN IDs")
        if msg.is_remote_frame:
            raise can.CanError("serial USB2CAN backend does not support remote frames")
        data = bytes(msg.data)
        if len(data) > 8:
            raise can.CanError("classic CAN frame data must be <= 8 bytes")
        frame = pack_usb2can_frame(msg.arbitration_id, data)
        self.send_raw(frame)
        self.tx_frames += 1

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        while True:
            try:
                if timeout is None:
                    msg = self._rx_queue.get(timeout=0.1)
                else:
                    msg = self._rx_queue.get(timeout=timeout)
                return msg, False
            except queue.Empty:
                if timeout is None and not self._stop.is_set():
                    continue
                return None, False

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                assert self._ser is not None
                chunk = self._ser.read(256)
            except Exception as exc:
                self.last_error = str(exc)
                LOG.error("USB2CAN serial read failed: %s", exc)
                return
            if not chunk:
                continue
            buf.extend(chunk)
            frames, buf = parse_usb2can_frames(buf)
            for frame in frames:
                if frame.cmd == 0x12:
                    self.acks += 1
                    continue
                if frame.cmd != 0x11:
                    continue
                if frame.is_remote:
                    continue
                msg = can.Message(
                    arbitration_id=frame.can_id,
                    data=frame.data,
                    is_extended_id=frame.is_extended,
                    is_remote_frame=False,
                    timestamp=time.time(),
                )
                try:
                    self._rx_queue.put_nowait(msg)
                except queue.Full:
                    try:
                        self._rx_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self._rx_queue.put_nowait(msg)
                    self.dropped_rx += 1
                self.rx_frames += 1

    def _heartbeat_loop(self, heartbeat_ms: int) -> None:
        period = max(0.1, heartbeat_ms / 1000.0)
        while not self._stop.wait(period):
            try:
                self.send_raw(bytes([0x55, 0x04, 0xAA, 0x55]))
            except can.CanError:
                return

    def stats(self) -> dict[str, Any]:
        return {
            "port": self._port,
            "baud": self._baud,
            "tx_frames": self.tx_frames,
            "rx_frames": self.rx_frames,
            "acks": self.acks,
            "bad_frames": self.bad_frames,
            "dropped_rx": self.dropped_rx,
            "last_error": self.last_error,
        }

    def shutdown(self) -> None:
        if not self._stop.is_set():
            self._stop.set()
            if self._heartbeat_thread is not None:
                self._heartbeat_thread.join(timeout=1.0)
            self._rx_thread.join(timeout=1.0)
            try:
                assert self._ser is not None
                self._ser.close()
            except Exception:
                pass
            LOG.info("closed %s", self.channel_info)
        try:
            super().shutdown()
        except Exception:
            pass


def create_bus(config: CanConfig) -> can.BusABC:
    if config.backend == "serial_usb2can":
        return SerialUsb2CanBus(
            port=config.port,
            baud=config.baud,
            can_bitrate=config.can_bitrate,
            can_baud_index=config.can_baud_index,
            set_can_baud=config.set_can_baud,
            send_heartbeat=config.send_heartbeat,
            heartbeat_ms=config.heartbeat_ms,
            rx_queue_size=config.rx_queue_size,
            tx_timeout_ms=config.tx_timeout_ms,
            auto_detect=config.auto_detect,
        )
    if config.backend == "virtual":
        return can.Bus(
            interface="virtual",
            channel=config.port or "gimbal",
            receive_own_messages=False,
        )
    if config.backend == "socketcan":
        return can.Bus(
            interface="socketcan",
            channel=str(config.channel) if isinstance(config.channel, int) else config.channel,
            bitrate=config.can_bitrate,
        )
    raise NotImplementedError(
        "dmcan (USB2CANFD libusb) backend is not implemented in the MVP; "
        "use backend=serial_usb2can"
    )
