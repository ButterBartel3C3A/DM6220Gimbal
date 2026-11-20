"""PID, axis controllers, and the dual-axis gimbal controller."""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

from .config import AppConfig, PidConfig
from .kinematics import Gimbal2D, YawEstimator
from .motor import MotorSnapshot


@dataclass(frozen=True)
class AxisState:
    pos: float
    vel: float
    tau: float
    status: int
    last_rx_age: float


@dataclass(frozen=True)
class DriveCommand:
    """Command for the servo drive's native position/velocity loops.

    pos_vel mode:      pos = joint position target, vel = velocity limit
    velocity_pos mode: pos = 0, vel = velocity target for 0x200+ID
    """

    pos: float = 0.0
    vel: float = 0.0


class LowPass:
    def __init__(self, cutoff_hz: float) -> None:
        self.cutoff_hz = max(0.0, cutoff_hz)
        self.value: float | None = None

    def reset(self, value: float | None = None) -> None:
        self.value = value

    def update(self, x: float, dt: float) -> float:
        if self.cutoff_hz <= 0:
            self.value = x
            return x
        if self.value is None:
            self.value = x
            return x
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = max(1e-6, dt) / (tau + max(1e-6, dt))
        self.value += alpha * (x - self.value)
        return self.value


class VelocityEstimator:
    """Robust velocity from position samples over a short window.

    The DM velocity telemetry is noisy near zero; differentiating the 16-bit
    position over a 50 ms window is much better for a 1 rad/s safety limit.
    """

    def __init__(self, window_s: float = 0.05, cutoff_hz: float = 10.0) -> None:
        self.window_s = max(0.01, window_s)
        self.cutoff_hz = max(0.0, cutoff_hz)
        self._samples: deque[tuple[float, float]] = deque()
        self.value = 0.0

    def reset(self) -> None:
        self._samples.clear()
        self.value = 0.0

    def update(self, position: float, now: float) -> float:
        self._samples.append((now, position))
        while self._samples and now - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        if len(self._samples) < 2:
            return self.value
        t0, p0 = self._samples[0]
        t1, p1 = self._samples[-1]
        dt = t1 - t0
        if dt <= 1e-6:
            return self.value
        raw = (p1 - p0) / dt
        if self.cutoff_hz <= 0:
            self.value = raw
        else:
            tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
            alpha = dt / (tau + dt)
            self.value += alpha * (raw - self.value)
        return self.value


class PID:
    def __init__(self, cfg: PidConfig, name: str) -> None:
        self.cfg = cfg
        self.name = name
        self._integral = 0.0
        self._prev_measurement: float | None = None
        self._derivative = 0.0
        self._last_output = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_measurement = None
        self._derivative = 0.0
        self._last_output = 0.0

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        dt = max(1e-6, dt)
        error = setpoint - measurement
        if abs(error) < self.cfg.deadband_rad:
            error = 0.0

        # Conditional integration with output limiting (simple anti-windup).
        candidate_integral = self._integral + self.cfg.ki * error * dt
        limit = abs(self.cfg.integral_limit)
        candidate_integral = max(-limit, min(limit, candidate_integral))

        if self._prev_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (measurement - self._prev_measurement) / dt
        if self.cfg.derivative_filter_hz > 0:
            tau = 1.0 / (2.0 * math.pi * self.cfg.derivative_filter_hz)
            alpha = dt / (tau + dt)
            self._derivative += alpha * (raw_derivative - self._derivative)
        else:
            self._derivative = raw_derivative

        output = (
            self.cfg.kp * error
            + candidate_integral
            - self.cfg.kd * self._derivative
        )
        out_limit = abs(self.cfg.output_limit)
        if abs(output) > out_limit:
            output = math.copysign(out_limit, output)
            # Do not wind up further in the saturated direction.
            if error * output > 0:
                candidate_integral = self._integral
        self._integral = candidate_integral
        self._prev_measurement = measurement
        self._last_output = output
        return output


class SlewLimiter:
    """Rate-limit a target (rad/s)."""

    def __init__(self, max_velocity: float) -> None:
        self.max_velocity = max(1e-9, abs(max_velocity))
        self.value: float | None = None

    def reset(self, value: float | None = None) -> None:
        self.value = value

    def update(self, target: float, dt: float) -> float:
        if self.value is None:
            self.value = target
            return target
        max_delta = self.max_velocity * max(dt, 1e-6)
        delta = target - self.value
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        self.value += delta
        return self.value


class AxisController:
    """Position/velocity outer loop for the drive's native servo loops.

    - pos_vel: pass the position target and velocity limit to the drive
      (0x100+ID); the drive closes position -> velocity -> current.
    - velocity_pos: PC position PID produces a velocity target (0x200+ID);
      the drive closes velocity -> current.
    """

    def __init__(self, axis: str, cfg: AppConfig) -> None:
        self.axis = axis
        self.cfg = cfg
        self.mode = cfg.control.mode
        self.position_pid = PID(cfg.control.position[axis], f"{axis}.position")
        self.velocity_limit = abs(cfg.control.velocity_limit_rad_s)

    def reset(self) -> None:
        self.position_pid.reset()

    def update(self, target: float, state: AxisState, dt: float) -> DriveCommand:
        if self.mode == "pos_vel":
            return DriveCommand(pos=target, vel=self.velocity_limit)

        if self.mode == "velocity_pos":
            v_target = self.position_pid.update(target, state.pos, dt)
            v_target = max(-self.velocity_limit, min(self.velocity_limit, v_target))
            return DriveCommand(pos=0.0, vel=v_target)

        raise ValueError(f"unknown control mode {self.mode}")


class GimbalController:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.kinematics = Gimbal2D(cfg.kinematics, cfg.motors["yaw"].p_max)
        self.yaw_estimator = YawEstimator(span=2.0 * cfg.motors["yaw"].p_max)
        self.yaw_ctrl = AxisController("yaw", cfg)
        self.pitch_ctrl = AxisController("pitch", cfg)
        self.velocity_cap = min(
            cfg.control.trajectory.max_joint_velocity_rad_s,
            cfg.control.velocity_limit_rad_s,
        )
        self.yaw_slew = SlewLimiter(self.velocity_cap)
        self.pitch_slew = SlewLimiter(self.velocity_cap)
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self._target_time = 0.0
        self._last_update = time.monotonic()

    def hold_current(self, yaw_joint: float, pitch_joint: float) -> None:
        self.target_yaw = yaw_joint
        self.target_pitch = pitch_joint
        self.yaw_slew.reset(yaw_joint)
        self.pitch_slew.reset(pitch_joint)
        self.yaw_ctrl.reset()
        self.pitch_ctrl.reset()
        self._target_time = time.monotonic()

    def set_target_joints(
        self, yaw_joint: float, pitch_joint: float, duration_s: float | None = None
    ) -> None:
        clamped_yaw = self.kinematics.clamp_joint("yaw", yaw_joint)
        clamped_pitch = self.kinematics.clamp_joint("pitch", pitch_joint)
        cap = self.velocity_cap
        if duration_s is not None and duration_s > 0:
            yaw_v = abs(clamped_yaw - self.target_yaw) / duration_s
            pitch_v = abs(clamped_pitch - self.target_pitch) / duration_s
            self.yaw_slew.max_velocity = min(cap, max(1e-3, yaw_v))
            self.pitch_slew.max_velocity = min(cap, max(1e-3, pitch_v))
        else:
            self.yaw_slew.max_velocity = cap
            self.pitch_slew.max_velocity = cap
        self.target_yaw = clamped_yaw
        self.target_pitch = clamped_pitch
        self._target_time = time.monotonic()

    def set_target_attitude(
        self,
        yaw: float,
        pitch: float,
        roll: float = 0.0,
        duration_s: float | None = None,
    ) -> tuple[float, float]:
        yaw_joint, pitch_joint = self.kinematics.attitude_to_joints(yaw, pitch, roll)
        self.set_target_joints(yaw_joint, pitch_joint, duration_s)
        return yaw_joint, pitch_joint

    def target_age(self) -> float:
        return time.monotonic() - self._target_time

    def yaw_joint_from_snapshot(self, yaw_snapshot: MotorSnapshot) -> float:
        estimate = self.yaw_estimator.update(yaw_snapshot.raw_pos)
        return self.cfg.motors["yaw"].direction * estimate + self.cfg.motors["yaw"].zero_offset_rad

    def yaw_joint_estimate(self, _yaw_snapshot: MotorSnapshot) -> float:
        """Non-mutating estimate for telemetry (does not advance the unwrapper)."""
        return (
            self.cfg.motors["yaw"].direction * self.yaw_estimator.value
            + self.cfg.motors["yaw"].zero_offset_rad
        )

    def update(
        self, dt: float, yaw_snapshot: MotorSnapshot, pitch_snapshot: MotorSnapshot
    ) -> tuple[DriveCommand, DriveCommand]:
        yaw_joint = self.yaw_joint_from_snapshot(yaw_snapshot)
        pitch_joint = pitch_snapshot.pos

        target_yaw = self.yaw_slew.update(self.target_yaw, dt)
        target_pitch = self.pitch_slew.update(self.target_pitch, dt)

        yaw_state = AxisState(
            pos=yaw_joint,
            vel=yaw_snapshot.vel,
            tau=yaw_snapshot.tau,
            status=yaw_snapshot.status,
            last_rx_age=yaw_snapshot.last_rx_age,
        )
        pitch_state = AxisState(
            pos=pitch_joint,
            vel=pitch_snapshot.vel,
            tau=pitch_snapshot.tau,
            status=pitch_snapshot.status,
            last_rx_age=pitch_snapshot.last_rx_age,
        )
        yaw_cmd = self.yaw_ctrl.update(target_yaw, yaw_state, dt)
        pitch_cmd = self.pitch_ctrl.update(target_pitch, pitch_state, dt)
        self._last_update = time.monotonic()
        return yaw_cmd, pitch_cmd

    def resync_yaw_absolute(self, absolute_motor_pos: float) -> float:
        return self.yaw_estimator.resync(absolute_motor_pos)

    def current_attitude(self, yaw_snapshot: MotorSnapshot, pitch_snapshot: MotorSnapshot) -> tuple[float, float]:
        yaw_joint = self.yaw_joint_estimate(yaw_snapshot)
        return self.kinematics.joints_to_attitude(yaw_joint, pitch_snapshot.pos)
