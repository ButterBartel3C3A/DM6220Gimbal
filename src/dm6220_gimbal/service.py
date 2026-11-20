"""Gimbal service: state machine, control loop, safety, telemetry."""

from __future__ import annotations

import logging
import math
import statistics
import struct
import threading
import time
from collections import deque
from enum import Enum
from typing import Any

import can

from .config import AppConfig
from .control import DriveCommand, GimbalController, LowPass, VelocityEstimator
from .motor import CanRouter, DmMotor, MotorSnapshot
from .transport import SerialUsb2CanBus, create_bus

LOG = logging.getLogger(__name__)


class GimbalState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    ENABLED = "ENABLED"
    RUNNING = "RUNNING"
    FAULT = "FAULT"
    ESTOP = "ESTOP"


class GimbalServiceError(RuntimeError):
    pass


class GimbalService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.bus: can.BusABC | None = None
        self.motors: dict[str, DmMotor] = {}
        self.router: CanRouter | None = None
        self.controller = GimbalController(cfg)

        self.state = GimbalState.DISCONNECTED
        self.fault_reason: str | None = None
        self._state_lock = threading.RLock()

        self._stop_control = threading.Event()
        self._control_thread: threading.Thread | None = None
        self._stop_resync = threading.Event()
        self._resync_thread: threading.Thread | None = None

        self._loop_periods: deque[float] = deque(maxlen=1000)
        self._loop_count = 0
        self._overruns = 0
        self._last_loop_error: str | None = None
        self._initial_modes: dict[str, int] = {}
        self._yaw_abs: float | None = None
        self._target_source = "none"
        self._target_deadline: float | None = None
        self._last_target_time = 0.0
        self._warned_temp = False
        self._last_commands: dict[str, DriveCommand] = {}
        self._vel_filters: dict[str, LowPass] = {}
        self._filtered_vel: dict[str, float] = {}
        self._pos_vel_estimators: dict[str, VelocityEstimator] = {}
        self._pos_vel: dict[str, float] = {}

    # ------------------------------------------------------------------ helpers
    def _set_state(self, state: GimbalState, reason: str | None = None) -> None:
        with self._state_lock:
            self.state = state
            self.fault_reason = reason
        LOG.info("state -> %s%s", state.value, f" ({reason})" if reason else "")

    def _require_connected(self) -> None:
        if self.state not in (GimbalState.CONNECTED, GimbalState.ENABLED, GimbalState.RUNNING):
            raise GimbalServiceError(f"not connected (state={self.state.value})")

    def _require_enabled(self) -> None:
        if self.state not in (GimbalState.ENABLED, GimbalState.RUNNING):
            raise GimbalServiceError(f"not enabled (state={self.state.value})")

    def _check_target_rate(self) -> None:
        now = time.monotonic()
        min_interval = 1.0 / max(1.0, self.cfg.api.max_command_rate_hz)
        if now - self._last_target_time < min_interval:
            raise GimbalServiceError("target command rate limit exceeded")
        self._last_target_time = now

    def _snapshots(self) -> dict[str, MotorSnapshot]:
        return {name: motor.snapshot() for name, motor in self.motors.items()}

    def _yaw_pitch_joints(
        self, snaps: dict[str, MotorSnapshot], update_yaw: bool = True
    ) -> tuple[float, float]:
        if update_yaw:
            yaw_joint = self.controller.yaw_joint_from_snapshot(snaps["yaw"])
        else:
            yaw_joint = self.controller.yaw_joint_estimate(snaps["yaw"])
        pitch_joint = snaps["pitch"].pos
        return yaw_joint, pitch_joint

    def _write_drive_float(self, motor: DmMotor, rid: int, value: float) -> None:
        bits = struct.unpack("<I", struct.pack("<f", float(value)))[0]
        if not motor.write_register(rid, bits, timeout=0.5):
            raise GimbalServiceError(
                f"{motor.name}: failed to write drive parameter RID 0x{rid:02X}"
            )

    def _apply_drive_params(self, name: str, motor: DmMotor) -> None:
        if self.cfg.control.mode != "pos_vel":
            return
        mcfg = self.cfg.motors[name]
        if mcfg.position_kp_apr is not None:
            self._write_drive_float(motor, 0x1B, mcfg.position_kp_apr)
        if mcfg.position_ki_apr is not None:
            self._write_drive_float(motor, 0x1C, mcfg.position_ki_apr)
        if mcfg.position_kp_apr is not None or mcfg.position_ki_apr is not None:
            LOG.info(
                "%s: drive POS_VEL gains KP_APR=%s KI_APR=%s",
                name, mcfg.position_kp_apr, mcfg.position_ki_apr,
            )

    def _safe_stop_command(self, motor: DmMotor) -> None:
        try:
            if self.cfg.control.mode == "pos_vel":
                # Hold the current position with a zero velocity limit.
                motor.send_pos_vel(motor.snapshot().pos, 0.0)
            else:
                motor.send_velocity(0.0)
        except Exception:
            pass

    def _safe_zero_disable(self, restore_modes: bool = True) -> None:
        for _ in range(2):
            for motor in self.motors.values():
                self._safe_stop_command(motor)
            time.sleep(0.02)
        for name, motor in self.motors.items():
            try:
                motor.disable()
            except Exception:
                pass
            if restore_modes:
                try:
                    # Production is POS_VEL/velocity only.  Leave the drive in
                    # velocity mode when disabled, even if a legacy MIT mode
                    # was found at connect time.
                    motor.write_register(0x0A, 3, timeout=0.3)
                except Exception:
                    pass
            try:
                motor.write_register(0x09, 0, timeout=0.3)
            except Exception:
                pass

    # --------------------------------------------------------------- lifecycle
    def connect(self) -> dict[str, Any]:
        with self._state_lock:
            if self.state != GimbalState.DISCONNECTED:
                raise GimbalServiceError(f"already {self.state.value}")
            try:
                self.bus = create_bus(self.cfg.can)
            except Exception as exc:
                raise GimbalServiceError(f"failed to open CAN transport: {exc}") from exc

            self._loop_periods.clear()
            self._loop_count = 0
            self._overruns = 0
            self._last_loop_error = None
            self._warned_temp = False
            self._yaw_abs = None

            self.motors = {
                name: DmMotor(name, motor_cfg, self.bus)
                for name, motor_cfg in self.cfg.motors.items()
            }
            self._vel_filters = {name: LowPass(20.0) for name in self.motors}
            self._filtered_vel = {name: 0.0 for name in self.motors}
            self._pos_vel_estimators = {name: VelocityEstimator(0.05, 10.0) for name in self.motors}
            self._pos_vel = {name: 0.0 for name in self.motors}
            self.router = CanRouter(self.bus, self.motors)
            for motor in self.motors.values():
                motor.attach_router(self.router)
            self.router.start()
            self.controller = GimbalController(self.cfg)

        # Register requests both prove communication and let us seed the current
        # positions without sending any motion command. Control feedback only
        # starts after the motors receive control frames.
        self._initial_modes = {}
        positions: dict[str, float | None] = {}
        for name, motor in self.motors.items():
            try:
                mode = motor.read_register(0x0A, timeout=0.75)
            except Exception as exc:
                self._close_transport()
                raise GimbalServiceError(
                    f"{name}: no register response; check COM port, CAN wiring, "
                    f"IDs, 1 Mbps and that DMTool is closed"
                ) from exc
            self._initial_modes[name] = mode
            try:
                raw = motor.read_register(self.cfg.kinematics.absolute_position_rid, timeout=0.75)
                pos = struct.unpack("<f", raw.to_bytes(4, "little"))[0]
                motor.seed_raw_position(pos)
                positions[name] = pos
            except Exception:
                positions[name] = None
                LOG.warning("%s: could not read absolute position (0x50)", name)

        if positions.get("yaw") is not None:
            self._yaw_abs = positions["yaw"]
            self.controller.yaw_estimator.update(positions["yaw"])

        self._set_state(GimbalState.CONNECTED)
        LOG.info(
            "connected: %s",
            ", ".join(
                f"{name} can=0x{motor.can_id:02X} fb=0x{motor.feedback_id:02X} "
                f"mode={self._initial_modes.get(name)} p_m={positions.get(name)}"
                for name, motor in self.motors.items()
            ),
        )
        return self.snapshot()

    def disconnect(self) -> dict[str, Any]:
        if self.state in (GimbalState.ENABLED, GimbalState.RUNNING, GimbalState.FAULT, GimbalState.ESTOP):
            self._stop_control.set()
            if self._control_thread is not None:
                self._control_thread.join(timeout=2.0)
                self._control_thread = None
            self._stop_resync.set()
            if self._resync_thread is not None:
                self._resync_thread.join(timeout=1.5)
                self._resync_thread = None
            self._safe_zero_disable()
        self._close_transport()
        self._set_state(GimbalState.DISCONNECTED)
        return self.snapshot()

    def _close_transport(self) -> None:
        if self.router is not None:
            try:
                self.router.stop()
            except Exception:
                pass
            self.router = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.motors = {}

    # -------------------------------------------------------------- enable/stop
    def enable(self) -> dict[str, Any]:
        self._require_connected()
        if self.state in (GimbalState.ENABLED, GimbalState.RUNNING):
            return self.snapshot()

        snaps = self._snapshots()
        yaw_joint, pitch_joint = self._yaw_pitch_joints(snaps)
        self.controller.hold_current(yaw_joint, pitch_joint)
        self._target_deadline = None
        for name, filt in self._vel_filters.items():
            filt.reset(0.0)
            self._filtered_vel[name] = 0.0
            self._pos_vel_estimators[name].reset()
            self._pos_vel[name] = 0.0

        timeout_units = max(0, int(self.cfg.safety.can_timeout_ms / 0.05))
        drive_mode = 2 if self.cfg.control.mode == "pos_vel" else 3
        for name, motor in self.motors.items():
            try:
                motor.disable()
                if not motor.set_control_mode(drive_mode):
                    raise GimbalServiceError(
                        f"{name}: failed to enter drive mode {drive_mode} "
                        f"({'POS_VEL' if drive_mode == 2 else 'VEL'})"
                    )
                self._apply_drive_params(name, motor)
                motor.clear_error()
                if timeout_units:
                    motor.write_register(0x09, timeout_units, timeout=0.5)
                motor.enable()
            except Exception as exc:
                self._safe_zero_disable()
                self._set_state(GimbalState.FAULT, f"enable failed for {name}: {exc}")
                raise GimbalServiceError(f"enable failed for {name}: {exc}") from exc

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snaps = self._snapshots()
            if all(s.status == 1 for s in snaps.values()):
                break
            time.sleep(0.02)
        else:
            statuses = {name: s.status for name, s in snaps.items()}
            self._safe_zero_disable()
            self._set_state(GimbalState.FAULT, f"motors did not enable: {statuses}")
            raise GimbalServiceError(f"motors did not enable: {statuses}")

        self._stop_control.clear()
        self._stop_resync.clear()
        self._control_thread = threading.Thread(target=self._control_loop, name="gimbal-control", daemon=True)
        self._control_thread.start()
        if self.cfg.kinematics.absolute_position_resync_hz > 0:
            self._resync_thread = threading.Thread(target=self._resync_loop, name="yaw-resync", daemon=True)
            self._resync_thread.start()
        self._set_state(GimbalState.ENABLED)
        return self.snapshot()

    def disable(self) -> dict[str, Any]:
        if self.state in (GimbalState.DISCONNECTED, GimbalState.CONNECTED):
            return self.snapshot()
        self._target_deadline = None
        self._stop_control.set()
        if self._control_thread is not None:
            self._control_thread.join(timeout=2.0)
            self._control_thread = None
        self._stop_resync.set()
        if self._resync_thread is not None:
            self._resync_thread.join(timeout=1.5)
            self._resync_thread = None
        self._safe_zero_disable()
        if self.state not in (GimbalState.FAULT, GimbalState.ESTOP):
            self._set_state(GimbalState.CONNECTED)
        return self.snapshot()

    def estop(self) -> dict[str, Any]:
        self._set_state(GimbalState.ESTOP, "emergency stop")
        self._target_deadline = None
        self._stop_control.set()
        if self._control_thread is not None:
            self._control_thread.join(timeout=1.5)
            self._control_thread = None
        self._stop_resync.set()
        if self._resync_thread is not None:
            self._resync_thread.join(timeout=1.0)
            self._resync_thread = None
        self._safe_zero_disable()
        return self.snapshot()

    def clear_fault(self) -> dict[str, Any]:
        if self.state not in (GimbalState.FAULT, GimbalState.ESTOP):
            return self.snapshot()
        if self.bus is None:
            self._set_state(GimbalState.DISCONNECTED)
            return self.snapshot()
        for motor in self.motors.values():
            try:
                motor.disable()
                motor.clear_error()
            except Exception:
                pass
        self._set_state(GimbalState.CONNECTED)
        return self.snapshot()

    def _enter_fault(self, reason: str) -> None:
        if self.state in (GimbalState.FAULT, GimbalState.ESTOP, GimbalState.DISCONNECTED):
            return
        LOG.error("gimbal fault: %s", reason)
        self._last_loop_error = reason
        self._set_state(GimbalState.FAULT, reason)
        self._stop_control.set()
        self._stop_resync.set()
        self._safe_zero_disable()

    # --------------------------------------------------------------- setpoints
    def set_attitude(
        self,
        yaw_deg: float,
        pitch_deg: float,
        roll_deg: float = 0.0,
        source: str = "api",
        duration_s: float = 0.0,
        ttl_ms: float | None = None,
    ) -> dict[str, Any]:
        self._require_enabled()
        self._check_target_rate()
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)
        try:
            self.controller.set_target_attitude(yaw, pitch, roll, duration_s or None)
        except ValueError as exc:
            raise GimbalServiceError(str(exc)) from exc
        self._target_source = source
        ttl = self.cfg.control.command_ttl_ms if ttl_ms is None else float(ttl_ms)
        self._target_deadline = time.monotonic() + max(0.05, ttl / 1000.0)
        self._set_state(GimbalState.RUNNING)
        return self.snapshot()

    def set_joints(
        self,
        yaw_rad: float,
        pitch_rad: float,
        source: str = "api",
        duration_s: float = 0.0,
        ttl_ms: float | None = None,
    ) -> dict[str, Any]:
        self._require_enabled()
        self._check_target_rate()
        try:
            self.controller.set_target_joints(yaw_rad, pitch_rad, duration_s or None)
        except ValueError as exc:
            raise GimbalServiceError(str(exc)) from exc
        self._target_source = source
        ttl = self.cfg.control.command_ttl_ms if ttl_ms is None else float(ttl_ms)
        self._target_deadline = time.monotonic() + max(0.05, ttl / 1000.0)
        self._set_state(GimbalState.RUNNING)
        return self.snapshot()

    def set_pid(self, axis: str, kp: float, ki: float, kd: float) -> dict[str, Any]:
        if axis not in ("yaw", "pitch"):
            raise GimbalServiceError("axis must be yaw or pitch")
        if not self.cfg.safety.allow_ai_pid_write:
            raise GimbalServiceError("PID writes are disabled by safety.allow_ai_pid_write")
        kp = max(0.0, min(100.0, float(kp)))
        ki = max(0.0, min(10.0, float(ki)))
        kd = max(0.0, min(10.0, float(kd)))
        ctrl = self.controller.yaw_ctrl if axis == "yaw" else self.controller.pitch_ctrl
        if self.cfg.control.mode == "pos_vel":
            raise GimbalServiceError(
                "POS_VEL drive gains live in the servo drive; set "
                "motors.<axis>.position_kp_apr/position_ki_apr in YAML "
                "(the service writes them at enable), or use velocity_pos for PC PID tuning"
            )
        ctrl.position_pid.cfg.kp = kp
        ctrl.position_pid.cfg.ki = ki
        ctrl.position_pid.cfg.kd = kd
        LOG.info("PC position PID %s updated: kp=%.3f ki=%.3f kd=%.3f", axis, kp, ki, kd)
        return self.snapshot()

    # ---------------------------------------------------------------- control
    def _control_loop(self) -> None:
        period = 1.0 / self.cfg.control.rate_hz
        next_t = time.perf_counter()
        prev_t = next_t
        while not self._stop_control.is_set():
            now = time.perf_counter()
            dt = now - prev_t
            prev_t = now
            dt = max(period * 0.5, min(period * 2.0, dt))
            try:
                snaps = self._snapshots()
                if self._loop_count == 0:
                    # Re-lock the target to the first real control feedback after
                    # enabling; the p_m seed can differ slightly from the feedback
                    # frame and a large initial error would cause a jump.
                    yaw_joint, pitch_joint = self._yaw_pitch_joints(snaps)
                    self.controller.hold_current(yaw_joint, pitch_joint)
                if (
                    self.state == GimbalState.RUNNING
                    and self._target_deadline is not None
                    and now >= self._target_deadline
                ):
                    yaw_joint, pitch_joint = self._yaw_pitch_joints(snaps)
                    self.controller.hold_current(yaw_joint, pitch_joint)
                    self._target_deadline = None
                    self._set_state(GimbalState.ENABLED)
                    LOG.info("target TTL expired; holding current position")
                for name, s in snaps.items():
                    if not math.isfinite(s.last_rx_age) or s.last_rx_age > self.cfg.control.feedback_timeout_ms / 1000.0:
                        self._enter_fault(f"{name}: feedback timeout ({s.last_rx_age:.3f}s)")
                        return
                    if s.status != 1:
                        self._enter_fault(f"{name}: status={s.status} ({s.status_name})")
                        return
                    vel_f = self._vel_filters[name].update(s.vel, dt)
                    # Use the unwrapped yaw estimate; s.pos is single-turn and
                    # would look like a huge velocity spike at the +/-12.5 wrap.
                    pos_for_vel = (
                        self.controller.yaw_joint_estimate(snaps["yaw"])
                        if name == "yaw"
                        else s.pos
                    )
                    pos_vel = self._pos_vel_estimators[name].update(pos_for_vel, now)
                    self._filtered_vel[name] = vel_f
                    self._pos_vel[name] = pos_vel
                    limit = self.cfg.safety.hard_max_velocity_rad_s
                    # The DM velocity estimate is noisy near zero (spikes of
                    # several rad/s with almost no position change).  Enforce the
                    # user limit on position-differentiated velocity; keep a wide
                    # raw backstop for a genuine runaway.
                    raw_backstop = max(5.0, limit * 5.0)
                    if abs(s.vel) > raw_backstop:
                        self._enter_fault(
                            f"{name}: raw over-speed {s.vel:+.3f} rad/s (backstop {raw_backstop:.3f})"
                        )
                        return
                    if abs(pos_vel) > limit:
                        self._enter_fault(
                            f"{name}: estimated over-speed {pos_vel:+.3f} rad/s (limit {limit:.3f})"
                        )
                        return
                    if max(s.t_mos, s.t_rotor) >= self.cfg.safety.hard_max_temp_c:
                        self._enter_fault(f"{name}: over-temperature {s.t_mos}/{s.t_rotor} C")
                        return
                    if not self._warned_temp and max(s.t_mos, s.t_rotor) >= self.cfg.motors[name].temp_warn_c:
                        LOG.warning("%s temperature warning: %.1f/%.1f C", name, s.t_mos, s.t_rotor)
                        self._warned_temp = True

                yaw_cmd, pitch_cmd = self.controller.update(dt, snaps["yaw"], snaps["pitch"])
                self._last_commands = {"yaw": yaw_cmd, "pitch": pitch_cmd}
                if self._loop_count < 200:
                    LOG.debug(
                        "ctrl %d yaw t=%.5f p=%.5f cpos=%.5f cvel=%.4f | "
                        "pitch t=%.5f p=%.5f cpos=%.5f cvel=%.4f",
                        self._loop_count,
                        self.controller.target_yaw,
                        self.controller.yaw_joint_estimate(snaps["yaw"]),
                        yaw_cmd.pos,
                        yaw_cmd.vel,
                        self.controller.target_pitch,
                        snaps["pitch"].pos,
                        pitch_cmd.pos,
                        pitch_cmd.vel,
                    )
                if self.cfg.control.mode == "pos_vel":
                    self.motors["yaw"].send_pos_vel(yaw_cmd.pos, yaw_cmd.vel)
                    self.motors["pitch"].send_pos_vel(pitch_cmd.pos, pitch_cmd.vel)
                else:
                    self.motors["yaw"].send_velocity(yaw_cmd.vel)
                    self.motors["pitch"].send_velocity(pitch_cmd.vel)
            except Exception as exc:
                self._enter_fault(f"control loop error: {exc}")
                return

            self._loop_count += 1
            self._loop_periods.append(dt)
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                self._overruns += 1
                if self._overruns > self.cfg.safety.max_loop_overruns:
                    # Do not fault on a few late cycles; log once at the threshold.
                    if self._overruns == self.cfg.safety.max_loop_overruns + 1:
                        LOG.warning("control loop is behind schedule (%d overruns)", self._overruns)

    def _resync_loop(self) -> None:
        interval = 1.0 / self.cfg.kinematics.absolute_position_resync_hz
        while not self._stop_resync.wait(interval):
            if self.state not in (GimbalState.ENABLED, GimbalState.RUNNING):
                continue
            try:
                raw = self.motors["yaw"].read_register(
                    self.cfg.kinematics.absolute_position_rid, timeout=0.3
                )
                absolute = struct.unpack("<f", raw.to_bytes(4, "little"))[0]
                self._yaw_abs = absolute
                self.controller.resync_yaw_absolute(absolute)
            except Exception as exc:
                LOG.debug("yaw absolute position resync failed: %s", exc)

    # ----------------------------------------------------------------- status
    def snapshot(self) -> dict[str, Any]:
        with self._state_lock:
            state = self.state
            fault = self.fault_reason
        motors = {
            name: self._motor_dict(
                m.snapshot(), self._filtered_vel.get(name), self._pos_vel.get(name)
            )
            for name, m in self.motors.items()
        }
        attitude = None
        joints = None
        target = None
        if "yaw" in self.motors and "pitch" in self.motors:
            try:
                snaps = self._snapshots()
                yaw_joint, pitch_joint = self._yaw_pitch_joints(snaps, update_yaw=False)
                yaw_att, pitch_att = self.controller.kinematics.joints_to_attitude(
                    yaw_joint, pitch_joint
                )
                attitude = {
                    "yaw_deg": math.degrees(yaw_att),
                    "pitch_deg": math.degrees(pitch_att),
                    "roll_deg": 0.0,
                }
                joints = {
                    "yaw_rad": yaw_joint,
                    "yaw_raw_rad": snaps["yaw"].raw_pos,
                    "yaw_abs_rad": self._yaw_abs,
                    "pitch_rad": pitch_joint,
                }
                target = {
                    "yaw_rad": self.controller.target_yaw,
                    "pitch_rad": self.controller.target_pitch,
                    "age_ms": self.controller.target_age() * 1000.0,
                    "source": self._target_source,
                }
            except Exception:
                pass
        periods = list(self._loop_periods)
        p50 = statistics.median(periods) * 1000.0 if periods else 0.0
        if len(periods) >= 100:
            p99 = statistics.quantiles(periods, n=100)[98] * 1000.0
        else:
            p99 = max(periods, default=0.0) * 1000.0
        bus_stats = None
        if isinstance(self.bus, SerialUsb2CanBus):
            bus_stats = self.bus.stats()
        commands = {
            name: {
                "pos_rad": c.pos,
                "vel_rad_s": c.vel,
            }
            for name, c in self._last_commands.items()
        }
        return {
            "state": state.value,
            "connected": self.bus is not None,
            "fault": fault,
            "mode": self.cfg.control.mode,
            "attitude": attitude,
            "joints": joints,
            "target": target,
            "commands": commands,
            "motors": motors,
            "loop": {
                "rate_hz": self.cfg.control.rate_hz,
                "count": self._loop_count,
                "jitter_ms_p50": p50,
                "jitter_ms_p99": p99,
                "overruns": self._overruns,
                "last_error": self._last_loop_error,
            },
            "limits": {
                "torque_nm": min(m.torque_limit_nm for m in self.cfg.motors.values()),
                "velocity_rad_s": self.cfg.safety.hard_max_velocity_rad_s,
            },
            "bus": bus_stats,
        }

    @staticmethod
    def _motor_dict(
        s: MotorSnapshot,
        filtered_vel: float | None = None,
        estimated_vel: float | None = None,
    ) -> dict[str, Any]:
        age = s.last_rx_age if math.isfinite(s.last_rx_age) else None
        return {
            "name": s.name,
            "can_id": s.can_id,
            "feedback_id": s.feedback_id,
            "status": s.status,
            "status_name": s.status_name,
            "pos_rad": s.pos,
            "raw_pos_rad": s.raw_pos,
            "vel_rad_s": s.vel,
            "vel_filtered_rad_s": filtered_vel,
            "vel_est_rad_s": estimated_vel,
            "tau_nm": s.tau,
            "t_mos_c": s.t_mos,
            "t_rotor_c": s.t_rotor,
            "rx_age_ms": None if age is None else age * 1000.0,
            "rx_count": s.rx_count,
        }
