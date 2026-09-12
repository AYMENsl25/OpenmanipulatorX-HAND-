import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from kinematics import (
    JointAngles,
    MotorAngles,
    forward_kinematics,
    forward_kinematics_from_joints,
    forward_kinematics_official,
)


def test_home_fk():
    motors = MotorAngles(config.HOME_ID11, config.HOME_ID12, config.HOME_ID13, config.HOME_ID14)
    tip_xyz = forward_kinematics(motors)
    official_xyz = forward_kinematics_official(motors)
    assert math.dist((tip_xyz.x, tip_xyz.y, tip_xyz.z), config.REST_XYZ_MM) < 0.01
    assert math.dist((official_xyz.x, official_xyz.y, official_xyz.z), config.REST_OFFICIAL_XYZ_MM) < 0.01


def test_named_work_pose_fk():
    xyz = forward_kinematics_from_joints(JointAngles(*config.WORK_JOINT_DEGREES))
    assert math.dist((xyz.x, xyz.y, xyz.z), config.WORK_XYZ_MM) < 0.01
    assert config.FK_JOINT_LIMITS["theta4"].contains(config.WORK_JOINT_DEGREES[3])


def test_base_offset_rotates_with_joint1():
    xyz = forward_kinematics_from_joints(JointAngles(90.0, 0.0, 0.0, 0.0))
    assert abs(xyz.x) < 0.01
    assert abs(xyz.y - config.REST_XYZ_MM[0]) < 0.01
    assert abs(xyz.z - config.REST_XYZ_MM[2]) < 0.01
