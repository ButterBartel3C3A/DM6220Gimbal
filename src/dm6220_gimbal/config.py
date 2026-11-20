"""Configuration models and YAML loading."""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class CanConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    backend: Literal["serial_usb2can", "virtual", "dmcan", "socketcan"] = "serial_usb2can"

    # serial USB2CAN
    port: str = "COM10"
    auto_detect: bool = True
    baud: int = 921_600
    can_bitrate: int = 1_000_000
    can_baud_index: int = 0
    set_can_baud: bool = True
    protocol: str = "usb2can_v2"
    send_heartbeat: bool = False
    heartbeat_ms: int = 1000
    rx_queue_size: int = 4096
    tx_timeout_ms: int = 5

    # USB2CANFD / SocketCAN fallback
    device_index: int = 0
    channel: int = 0
    sample_point: float = 0.8
    library_path: str | None = None


class MotorConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    model: str = "DM6220"
    can_id: int = Field(ge=1, le=0x7FF)
    feedback_id: int = Field(ge=0, le=0x7FF)
    direction: int = 1
    zero_offset_rad: float = 0.0
    p_max: float = Field(default=12.5, gt=0)
    v_max: float = Field(default=45.0, gt=0)
    t_max: float = Field(default=10.0, gt=0)
    torque_limit_nm: float = Field(default=0.8, gt=0)
    temp_warn_c: float = 75.0
    temp_fault_c: float = 90.0
    # Native POS_VEL position-loop gains (drive RID 0x1B/0x1C), RAM only.
    # The service writes these at enable when control.mode == "pos_vel".
    position_kp_apr: float | None = None
    position_ki_apr: float | None = None

    @model_validator(mode="after")
    def _validate_motor(self) -> "MotorConfig":
        if self.direction not in (-1, 1):
            raise ValueError("motor.direction must be +1 or -1")
        if self.temp_fault_c <= self.temp_warn_c:
            raise ValueError("temp_fault_c must be greater than temp_warn_c")
        return self


class PidConfig(BaseModel):
    """PC position-loop PID (used only in velocity_pos mode).

    The output is a velocity target in rad/s, not torque.
    """

    model_config = ConfigDict(extra="forbid")

    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 0.3
    output_limit: float = 1.0  # rad/s
    derivative_filter_hz: float = 20.0
    deadband_rad: float = 0.0


class TrajectoryConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    max_joint_velocity_rad_s: float = 1.0
    max_joint_accel_rad_s2: float = 12.0
    default_duration_s: float = 0.5


class ControlConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    rate_hz: float = Field(default=500.0, ge=50.0, le=1000.0)
    # pos_vel: driver POS_VEL (0x100+ID), PC sends position + velocity limit
    # velocity_pos: PC position PID -> driver velocity mode (0x200+ID)
    mode: Literal["pos_vel", "velocity_pos"] = "pos_vel"
    feedback_timeout_ms: float = 100.0
    command_ttl_ms: float = 500.0
    velocity_limit_rad_s: float = Field(default=0.8, gt=0.0)
    position: dict[str, PidConfig]
    trajectory: TrajectoryConfig = Field(default_factory=TrajectoryConfig)

    @model_validator(mode="after")
    def _validate_axes(self) -> "ControlConfig":
        missing = {"yaw", "pitch"} - set(self.position)
        if missing:
            raise ValueError(f"control.position missing axes: {sorted(missing)}")
        return self


class KinematicsConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    convention: Literal["yaw_pitch_zyx"] = "yaw_pitch_zyx"
    yaw_sign: int = 1
    pitch_sign: int = 1
    yaw_offset_rad: float = 0.0
    pitch_offset_rad: float = 0.0
    yaw_continuous: bool = True
    yaw_min_rad: float = -1.0e9
    yaw_max_rad: float = 1.0e9
    pitch_continuous: bool = False
    pitch_min_rad: float = -1.4835
    pitch_max_rad: float = 1.4835
    roll_tolerance_rad: float = 0.01
    unwrap_yaw: bool = True
    absolute_position_rid: int = 0x50
    absolute_position_resync_hz: float = 2.0

    @model_validator(mode="after")
    def _validate_kinematics(self) -> "KinematicsConfig":
        if self.yaw_sign not in (-1, 1) or self.pitch_sign not in (-1, 1):
            raise ValueError("yaw_sign/pitch_sign must be +1 or -1")
        if self.yaw_min_rad >= self.yaw_max_rad:
            raise ValueError("yaw_min_rad must be less than yaw_max_rad")
        if self.pitch_min_rad >= self.pitch_max_rad:
            raise ValueError("pitch_min_rad must be less than pitch_max_rad")
        if self.absolute_position_resync_hz < 0:
            raise ValueError("absolute_position_resync_hz must be >= 0")
        return self


class ApiConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    host: str = "127.0.0.1"
    port: int = 8765
    token: str | None = None
    max_command_rate_hz: float = 20.0
    max_target_step_deg: float = 10.0
    cors_origins: list[str] = Field(default_factory=list)


class SafetyConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    estop_behavior: Literal["ramp_and_disable", "hold", "disable"] = "disable"
    estop_ramp_ms: float = 300.0
    max_loop_overruns: int = 10
    enable_ramp_ms: float = 300.0
    allow_ai_pid_write: bool = True
    hard_max_torque_nm: float = 1.5
    hard_max_velocity_rad_s: float = 1.0
    hard_max_temp_c: float = 100.0
    can_timeout_ms: int = 1000


class TelemetryConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    snapshot_hz: float = 50.0
    record_hz: float = 100.0
    record_dir: str = "recordings"
    ring_buffer_seconds: float = 30.0
    jsonl: bool = True
    csv: bool = False


class LoggingConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    level: str = "INFO"
    file: str = "logs/gimbald.log"
    json_logs: bool = True
    console: bool = True


class AppConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    can: CanConfig = Field(default_factory=CanConfig)
    motors: dict[str, MotorConfig]
    control: ControlConfig
    kinematics: KinematicsConfig = Field(default_factory=KinematicsConfig)
    api: ApiConfig = Field(default_factory=ApiConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    telemetry: TelemetryConfig = Field(default_factory=TelemetryConfig)
    logging: LoggingConfig = Field(default_factory=LoggingConfig)
    source_path: Path | None = Field(default=None, exclude=True)

    @model_validator(mode="after")
    def _validate_app(self) -> "AppConfig":
        missing = {"yaw", "pitch"} - set(self.motors)
        if missing:
            raise ValueError(f"motors missing axes: {sorted(missing)}")

        ids = [m.can_id for m in self.motors.values()]
        if len(ids) != len(set(ids)):
            raise ValueError("motor can_id values must be unique")
        feedback_ids = [m.feedback_id for m in self.motors.values()]
        if len(feedback_ids) != len(set(feedback_ids)):
            raise ValueError("motor feedback_id values must be unique")

        for name, motor in self.motors.items():
            if motor.torque_limit_nm > self.safety.hard_max_torque_nm:
                raise ValueError(
                    f"motor {name} torque_limit_nm exceeds safety.hard_max_torque_nm"
                )
        if self.control.trajectory.max_joint_velocity_rad_s > self.safety.hard_max_velocity_rad_s:
            raise ValueError(
                "trajectory.max_joint_velocity_rad_s exceeds safety.hard_max_velocity_rad_s"
            )
        return self


def default_config_path() -> Path:
    for candidate in (Path("config/gimbal.yaml"), Path("config/gimbal.example.yaml"), Path("gimbal.yaml")):
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "no config found; copy config/gimbal.example.yaml to config/gimbal.yaml"
    )


def load_config(path: str | Path | None = None) -> AppConfig:
    config_path = Path(path) if path is not None else default_config_path()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{config_path}: expected a YAML mapping")
    cfg = AppConfig.model_validate(data)
    cfg.source_path = config_path
    return cfg
