import math

import pytest

from dm6220_gimbal.config import KinematicsConfig
from dm6220_gimbal.kinematics import Gimbal2D, YawEstimator, normalize_angle


def test_attitude_joint_mapping_default():
    g = Gimbal2D(KinematicsConfig(), 12.5)
    yaw_joint, pitch_joint = g.attitude_to_joints(0.5, 0.2)
    assert yaw_joint == pytest.approx(0.5)
    assert pitch_joint == pytest.approx(0.2)
    assert g.joints_to_attitude(yaw_joint, pitch_joint) == pytest.approx((0.5, 0.2))


def test_roll_is_rejected():
    g = Gimbal2D(KinematicsConfig(), 12.5)
    with pytest.raises(ValueError):
        g.attitude_to_joints(0.0, 0.0, roll=0.1)


def test_pitch_limit():
    g = Gimbal2D(KinematicsConfig(pitch_min_rad=-0.5, pitch_max_rad=0.5), 12.5)
    _, pitch_joint = g.attitude_to_joints(0.0, 2.0)
    assert pitch_joint == 0.5


def test_yaw_unwrap():
    est = YawEstimator(span=25.0)
    assert est.update(12.4) == pytest.approx(12.4)
    # Crossing +12.5 -> -12.5 should continue forward by 0.2 rad.
    assert est.update(-12.4) == pytest.approx(12.6)
    assert est.update(-12.0) == pytest.approx(13.0)


def test_yaw_resync():
    est = YawEstimator(span=25.0)
    est.update(0.0)
    # One resync is intentionally rate-limited to avoid a visible jump.
    est.resync(10.0, alpha=0.2)
    assert est.offset == pytest.approx(0.25)
    assert est.value == pytest.approx(0.25)
    for _ in range(10):
        est.resync(10.0, alpha=0.2)
    assert est.value > 2.0


def test_normalize_angle():
    assert normalize_angle(3 * math.pi) == pytest.approx(-math.pi)
    assert abs(normalize_angle(-3 * math.pi)) == pytest.approx(math.pi)
