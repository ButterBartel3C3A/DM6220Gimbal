"""2-DOF yaw/pitch kinematics and continuous-yaw position estimation."""

from __future__ import annotations

import math
import threading

from .config import KinematicsConfig


class Gimbal2D:
    """Analytic 2-DOF yaw/pitch mapping (R_WC = Rz(yaw) * Ry(pitch))."""

    def __init__(self, cfg: KinematicsConfig, motor_p_max: float) -> None:
        self.cfg = cfg
        self.span = 2.0 * motor_p_max

    def clamp_joint(self, axis: str, value: float) -> float:
        if axis == "yaw":
            if self.cfg.yaw_continuous:
                return value
            return max(self.cfg.yaw_min_rad, min(self.cfg.yaw_max_rad, value))
        if self.cfg.pitch_continuous:
            return value
        return max(self.cfg.pitch_min_rad, min(self.cfg.pitch_max_rad, value))

    def attitude_to_joints(self, yaw: float, pitch: float, roll: float = 0.0) -> tuple[float, float]:
        if abs(roll) > self.cfg.roll_tolerance_rad:
            raise ValueError(
                f"roll {roll:.4f} rad exceeds tolerance; this gimbal has only yaw/pitch"
            )
        yaw_joint = self.cfg.yaw_sign * yaw + self.cfg.yaw_offset_rad
        pitch_joint = self.cfg.pitch_sign * pitch + self.cfg.pitch_offset_rad
        return self.clamp_joint("yaw", yaw_joint), self.clamp_joint("pitch", pitch_joint)

    def joints_to_attitude(self, yaw_joint: float, pitch_joint: float) -> tuple[float, float]:
        yaw = (yaw_joint - self.cfg.yaw_offset_rad) / self.cfg.yaw_sign
        pitch = (pitch_joint - self.cfg.pitch_offset_rad) / self.cfg.pitch_sign
        return yaw, pitch

    def wrap_delta(self, delta: float) -> float:
        half = self.span / 2.0
        return (delta + half) % self.span - half


class YawEstimator:
    """Unwraps the 16-bit feedback position and slowly resyncs with p_m (0x50).

    The 16-bit MIT feedback wraps at +/- p_max (25 rad span for DM6220).
    The motor-side register 0x50 is a multi-turn absolute position.
    """

    def __init__(self, span: float = 25.0) -> None:
        self.span = span
        self._lock = threading.Lock()
        self._raw_prev: float | None = None
        self._unwrapped = 0.0
        self._offset = 0.0
        self._resync_count = 0

    def reset(self) -> None:
        with self._lock:
            self._raw_prev = None
            self._unwrapped = 0.0
            self._offset = 0.0
            self._resync_count = 0

    def update(self, raw: float) -> float:
        with self._lock:
            if self._raw_prev is None:
                self._unwrapped = raw
            else:
                delta = raw - self._raw_prev
                half = self.span / 2.0
                if delta > half:
                    delta -= self.span
                elif delta < -half:
                    delta += self.span
                self._unwrapped += delta
            self._raw_prev = raw
            return self._unwrapped + self._offset

    def resync(self, absolute: float, alpha: float = 0.2) -> float:
        """Apply a slow correction towards the multi-turn absolute position.

        Returns the corrected estimate. `alpha` is per-resync gain; callers
        should resync at a low rate (e.g. 2 Hz) so the correction stays slow.
        """
        with self._lock:
            estimate = self._unwrapped + self._offset
            error = (absolute - estimate + self.span / 2.0) % self.span - self.span / 2.0
            self._offset += max(-0.25, min(0.25, alpha * error))
            self._resync_count += 1
            return self._unwrapped + self._offset

    @property
    def value(self) -> float:
        with self._lock:
            return self._unwrapped + self._offset

    @property
    def unwrapped(self) -> float:
        with self._lock:
            return self._unwrapped

    @property
    def offset(self) -> float:
        with self._lock:
            return self._offset

    @property
    def resync_count(self) -> int:
        with self._lock:
            return self._resync_count


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
