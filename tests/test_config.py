from dm6220_gimbal.config import load_config


def test_example_config_loads():
    cfg = load_config("config/gimbal.example.yaml")
    assert cfg.motors["yaw"].can_id == 0x01
    assert cfg.motors["pitch"].can_id == 0x02
    assert cfg.motors["yaw"].feedback_id == 0x11
    assert cfg.motors["pitch"].feedback_id == 0x12
    assert cfg.control.mode in ("pos_vel", "velocity_pos")
    assert "yaw" in cfg.control.position and "pitch" in cfg.control.position
    assert cfg.kinematics.yaw_continuous is True
    assert cfg.safety.can_timeout_ms == 1000
