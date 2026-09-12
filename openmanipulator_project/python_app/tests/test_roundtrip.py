import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from kinematics import MotorAngles, fk_to_motor_angles, motor_to_fk_angles, raw_to_degrees, degrees_to_raw


def test_id14_physical_zero_is_separate_from_custom_home():
    assert config.ID14_ZERO == 0.0
    assert config.HOME_ID14 == 90.0
    assert config.CUSTOM_HOME_ID14 == 90.0


def test_motor_joint_roundtrip_home():
    motors = MotorAngles(351.0, 1.0, 1.0, 90.0)
    rebuilt = fk_to_motor_angles(motor_to_fk_angles(motors))
    assert abs(rebuilt.id11 - 351.0) < 1e-9
    assert abs(rebuilt.id12 - 1.0) < 1e-9
    assert abs(rebuilt.id13 - 1.0) < 1e-9
    assert abs(rebuilt.id14 - 90.0) < 1e-9


def test_raw_degree_conversion_layer():
    assert raw_to_degrees(0) == 0.0
    assert abs(raw_to_degrees(1024) - 90.0) < 1e-9
    assert degrees_to_raw(90.0) == 1024
