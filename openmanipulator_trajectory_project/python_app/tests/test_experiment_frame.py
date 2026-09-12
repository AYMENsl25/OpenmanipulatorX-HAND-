import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from experiment_frame import (
    PhysicalTargetError,
    experiment_points,
    forward_kinematics_physical,
    internal_to_physical,
    inverse_kinematics_physical,
    physical_to_internal,
    validate_physical_target,
)
from kinematics import KinematicsError, MotorAngles, XYZ


def test_configured_physical_workspace_and_points():
    assert config.PHYSICAL_FRAME["origin"] == "ID11_CENTER_AXIS"
    assert config.MEASURED_XYZ_LIMITS["x"].minimum == -100.0
    assert config.MEASURED_XYZ_LIMITS["x"].maximum == 350.0
    assert config.MEASURED_XYZ_LIMITS["y"].minimum == -200.0
    assert config.MEASURED_XYZ_LIMITS["y"].maximum == 200.0
    assert config.GROUND_Z_MM == 0.0
    assert experiment_points()["P01"] == XYZ(-35.0, 145.0, 0.0)
    assert experiment_points()["P07"] == XYZ(-35.0, -145.0, 0.0)
    assert experiment_points()["P08"] == XYZ(80.0, 70.0, 0.0)
    assert experiment_points()["P09"] == XYZ(230.0, 70.0, 0.0)
    assert experiment_points()["P10"] == XYZ(80.0, -70.0, 0.0)
    assert experiment_points()["P11"] == XYZ(230.0, -70.0, 0.0)


def test_physical_internal_transform_is_reversible_and_flips_y():
    physical = XYZ(-35.0, 145.0, 12.5)
    internal = physical_to_internal(physical)
    assert internal == XYZ(-35.0, -145.0, 12.5)
    assert internal_to_physical(internal) == physical


def test_soft_envelope_accepts_negative_x_and_protects_ground():
    validate_physical_target(XYZ(-100.0, 200.0, 0.0))
    try:
        validate_physical_target(XYZ(-100.001, 0.0, 0.0))
    except KinematicsError as exc:
        assert "SOFT CARTESIAN LIMIT" in str(exc)
    else:
        raise AssertionError("X below -100 mm should be rejected")
    try:
        validate_physical_target(XYZ(0.0, 0.0, -0.001))
    except PhysicalTargetError as exc:
        assert "GROUND PROTECTION" in str(exc)
    else:
        raise AssertionError("Z below the physical ground should be rejected")


def test_work_pose_physical_fk_and_ik_roundtrip():
    motors = MotorAngles(*config.WORK_MOTOR_DEGREES)
    physical_fk = forward_kinematics_physical(motors)
    assert math.dist(
        (physical_fk.x, physical_fk.y, physical_fk.z), config.WORK_XYZ_MM
    ) < 0.01
    solved = inverse_kinematics_physical(
        physical_fk.x, physical_fk.y, physical_fk.z
    )
    assert solved.ik.position_error <= 0.05
    assert math.dist(
        (
            solved.physical_fk_check.x,
            solved.physical_fk_check.y,
            solved.physical_fk_check.z,
        ),
        (physical_fk.x, physical_fk.y, physical_fk.z),
    ) <= 0.05


def test_negative_x_points_respect_current_calibrated_joint_limits():
    for name in ("P01", "P07"):
        point = experiment_points()[name]
        solved = inverse_kinematics_physical(point.x, point.y, point.z)
        assert config.FK_JOINT_LIMITS["theta1"].contains(solved.ik.joint_angles.theta1)
        assert abs(solved.ik.position_error) <= config.IK_POSITION_TOLERANCE_MM
