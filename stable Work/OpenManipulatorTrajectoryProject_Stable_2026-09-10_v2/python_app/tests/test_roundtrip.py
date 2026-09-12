import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from kinematics import MotorAngles, fk_to_motor_angles, motor_to_fk_angles, raw_to_degrees, degrees_to_raw


def test_calibrated_home_raw_and_directions():
    assert config.HOME_RAW == {11: 1917, 12: 2046, 13: 4049, 14: 0}
    assert config.WORK_RAW == {11: 1917, 12: 2046, 13: 4049, 14: 943}
    assert config.JOINT_DIRECTION == {11: 1.0, 12: 1.0, 13: 1.0, 14: 1.0}


def test_motor_joint_roundtrip_home():
    motors = MotorAngles(config.HOME_ID11, config.HOME_ID12, config.HOME_ID13, config.HOME_ID14)
    rebuilt = fk_to_motor_angles(motor_to_fk_angles(motors))
    assert all(abs(a - b) < 1e-9 for a, b in zip(rebuilt.as_tuple(), motors.as_tuple(), strict=True))


def test_raw_degree_conversion_layer():
    assert raw_to_degrees(0) == 0.0
    assert abs(raw_to_degrees(1024) - 90.0) < 1e-9
    assert degrees_to_raw(90.0) == 1024
