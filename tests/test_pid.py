import pytest

from dm6220_gimbal.config import PidConfig
from dm6220_gimbal.control import PID, SlewLimiter


def test_pid_output_limit():
    pid = PID(PidConfig(kp=10.0, ki=0.0, kd=0.0, output_limit=0.5), "test")
    assert pid.update(1.0, 0.0, 0.002) == pytest.approx(0.5)


def test_pid_integral_and_anti_windup():
    pid = PID(PidConfig(kp=0.0, ki=1.0, kd=0.0, output_limit=2.0, integral_limit=1.0), "test")
    for _ in range(1000):
        out = pid.update(1.0, 0.0, 0.01)
    assert out == pytest.approx(1.0)


def test_pid_derivative_on_measurement():
    pid = PID(PidConfig(kp=0.0, ki=0.0, kd=1.0, output_limit=10.0, derivative_filter_hz=0.0), "test")
    pid.update(0.0, 0.0, 0.01)
    out = pid.update(0.0, 1.0, 0.01)
    assert out < 0.0


def test_slew_limiter():
    s = SlewLimiter(max_velocity=1.0)
    s.reset(0.0)
    assert s.update(10.0, 0.1) == pytest.approx(0.1)
    assert s.update(10.0, 0.1) == pytest.approx(0.2)
    assert s.update(0.0, 0.1) == pytest.approx(0.1)
