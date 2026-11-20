"""DM6220 motor driver and CAN feedback router."""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field

import can

from .config import MotorConfig
from .protocol import (
    CAN_ID_REGISTER,
    SYSTEM_CLEAR_ERROR,
    SYSTEM_DISABLE,
    SYSTEM_ENABLE,
    SYSTEM_SET_ZERO,
    MitLimits,
    decode_feedback,
    encode_mit,
    encode_pos_vel,
    encode_register_read,
    encode_register_write,
    encode_store,
    encode_velocity,
    status_name,
)

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class MotorSnapshot:
    name: str
    can_id: int
    feedback_id: int
    status: int
    status_name: str
    pos: float
    raw_pos: float
    vel: float
    tau: float
    t_mos: float
    t_rotor: float
    last_rx_age: float
    rx_count: int


@dataclass
class _MotorState:
    status: int = 0
    raw_pos: float = 0.0
    raw_vel: float = 0.0
    raw_tau: float = 0.0
    t_mos: float = 0.0
    t_rotor: float = 0.0
    last_rx: float = 0.0
    rx_count: int = 0
    error_count: int = 0


class DmMotor:
    def __init__(self, name: str, cfg: MotorConfig, bus: can.BusABC) -> None:
        self.name = name
        self.cfg = cfg
        self.bus = bus
        self.router: CanRouter | None = None
        self.limits = MitLimits(cfg.p_max, cfg.v_max, cfg.t_max)
        self._state = _MotorState()
        self._lock = threading.Lock()

    @property
    def can_id(self) -> int:
        return self.cfg.can_id

    @property
    def feedback_id(self) -> int:
        return self.cfg.feedback_id

    def attach_router(self, router: "CanRouter") -> None:
        self.router = router

    def handle_feedback(self, msg: can.Message) -> bool:
        if msg.arbitration_id != self.feedback_id or len(msg.data) < 8:
            return False
        try:
            fb = decode_feedback(bytes(msg.data[:8]), self.limits)
        except ValueError:
            with self._lock:
                self._state.error_count += 1
            return False
        if fb.motor_id != (self.can_id & 0x0F):
            with self._lock:
                self._state.error_count += 1
            return False
        with self._lock:
            self._state.status = fb.status
            self._state.raw_pos = fb.pos
            self._state.raw_vel = fb.vel
            self._state.raw_tau = fb.tau
            self._state.t_mos = fb.t_mos
            self._state.t_rotor = fb.t_rotor
            self._state.last_rx = time.monotonic()
            self._state.rx_count += 1
        return True

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            s = self._state
            last_rx_age = float("inf") if s.last_rx == 0.0 else time.monotonic() - s.last_rx
            return MotorSnapshot(
                name=self.name,
                can_id=self.can_id,
                feedback_id=self.feedback_id,
                status=s.status,
                status_name=status_name(s.status),
                pos=self.cfg.direction * s.raw_pos + self.cfg.zero_offset_rad,
                raw_pos=s.raw_pos,
                vel=self.cfg.direction * s.raw_vel,
                tau=self.cfg.direction * s.raw_tau,
                t_mos=s.t_mos,
                t_rotor=s.t_rotor,
                last_rx_age=last_rx_age,
                rx_count=s.rx_count,
            )

    def reset_state(self) -> None:
        with self._lock:
            self._state = _MotorState()

    def seed_raw_position(self, raw_pos: float) -> None:
        """Seed the position from p_m (0x50) before control feedback exists."""
        with self._lock:
            self._state.raw_pos = raw_pos

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        """Send an MIT command in joint coordinates.

        The motor may be mounted with direction=-1, so position, velocity and
        torque are converted to motor coordinates here.
        """
        motor_pos = (pos - self.cfg.zero_offset_rad) * self.cfg.direction
        motor_vel = vel * self.cfg.direction
        motor_tau = tau * self.cfg.direction
        payload = encode_mit(motor_pos, motor_vel, kp, kd, motor_tau, self.limits)
        self.bus.send(can.Message(arbitration_id=self.can_id, data=payload, is_extended_id=False))

    def send_velocity(self, velocity_rad_s: float) -> None:
        payload = encode_velocity(velocity_rad_s * self.cfg.direction)
        self.bus.send(
            can.Message(arbitration_id=0x200 + self.can_id, data=payload, is_extended_id=False)
        )

    def send_pos_vel(self, position_rad: float, velocity_limit_rad_s: float) -> None:
        """Native position-velocity mode (0x100+ID).

        The drive runs position -> velocity -> current.  The velocity limit is
        a magnitude and is not affected by the joint direction.
        """
        motor_pos = (position_rad - self.cfg.zero_offset_rad) * self.cfg.direction
        payload = encode_pos_vel(motor_pos, velocity_limit_rad_s)
        self.bus.send(
            can.Message(arbitration_id=0x100 + self.can_id, data=payload, is_extended_id=False)
        )

    def _system(self, payload: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=self.can_id, data=payload, is_extended_id=False))

    def enable(self) -> None:
        self._system(SYSTEM_ENABLE)

    def disable(self) -> None:
        self._system(SYSTEM_DISABLE)

    def set_zero(self) -> None:
        self._system(SYSTEM_SET_ZERO)

    def clear_error(self) -> None:
        self._system(SYSTEM_CLEAR_ERROR)

    def read_register(self, rid: int, timeout: float = 0.5) -> int:
        if self.router is None:
            raise RuntimeError("motor is not attached to a CanRouter")
        return self.router.read_register(self.can_id, rid, timeout)

    def write_register(self, rid: int, value: int, timeout: float = 0.5) -> bool:
        if self.router is None:
            raise RuntimeError("motor is not attached to a CanRouter")
        return self.router.write_register(self.can_id, rid, value, timeout)

    def store_parameters(self) -> None:
        if self.router is None:
            raise RuntimeError("motor is not attached to a CanRouter")
        self.router.store_parameters(self.can_id)

    def set_control_mode(self, mode: int) -> bool:
        """1=MIT, 2=POS_VEL, 3=VEL, 4=FORCE_POS."""
        return self.write_register(0x0A, mode)


class CanRouter:
    """Reads CAN messages and dispatches motor feedback / register responses."""

    def __init__(self, bus: can.BusABC, motors: dict[str, DmMotor]) -> None:
        self.bus = bus
        self.motors = motors
        self.by_feedback_id = {m.feedback_id: m for m in motors.values()}
        self._pending: dict[tuple[int, int], queue.Queue[tuple[int, int]]] = {}
        self._pending_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._rx_loop, name="can-router", daemon=True)
        self.rx_messages = 0
        self.dispatch_errors = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
            except Exception as exc:
                LOG.error("CAN receive failed: %s", exc)
                return
            if msg is None:
                continue
            self.rx_messages += 1
            data = bytes(msg.data)
            if len(data) >= 8:
                can_id = data[0] | (data[1] << 8)
                rid = data[3]
                key = (can_id, rid)
                with self._pending_lock:
                    q = self._pending.get(key)
                if q is not None and data[2] in (0x33, 0x55):
                    try:
                        q.put_nowait((data[2], int.from_bytes(data[4:8], "little")))
                    except queue.Full:
                        pass
                    continue
            motor = self.by_feedback_id.get(msg.arbitration_id)
            if motor is not None:
                try:
                    motor.handle_feedback(msg)
                except Exception:
                    self.dispatch_errors += 1
                    LOG.exception("failed to dispatch feedback for %s", motor.name)

    def _request(
        self,
        can_id: int,
        rid: int,
        payload: bytes,
        expected_cmd: int,
        timeout: float,
    ) -> tuple[int, int] | None:
        key = (can_id, rid)
        q: queue.Queue[tuple[int, int]] = queue.Queue(maxsize=4)
        with self._pending_lock:
            self._pending[key] = q
        try:
            self.bus.send(can.Message(arbitration_id=CAN_ID_REGISTER, data=payload, is_extended_id=False))
            try:
                cmd, value = q.get(timeout=timeout)
            except queue.Empty:
                return None
            if cmd != expected_cmd:
                return None
            return cmd, value
        finally:
            with self._pending_lock:
                self._pending.pop(key, None)

    def read_register(self, can_id: int, rid: int, timeout: float = 0.5) -> int:
        result = self._request(can_id, rid, encode_register_read(can_id, rid), 0x33, timeout)
        if result is None:
            raise TimeoutError(f"register read timeout: can_id=0x{can_id:02X} rid=0x{rid:02X}")
        return result[1]

    def write_register(self, can_id: int, rid: int, value: int, timeout: float = 0.5) -> bool:
        result = self._request(
            can_id, rid, encode_register_write(can_id, rid, value), 0x55, timeout
        )
        return result is not None

    def store_parameters(self, can_id: int) -> None:
        self.bus.send(
            can.Message(arbitration_id=CAN_ID_REGISTER, data=encode_store(can_id), is_extended_id=False)
        )
