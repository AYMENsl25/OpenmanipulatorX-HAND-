"""Dependency-free test runner for the project FK/IK checks."""

from __future__ import annotations

import math

import config
from experiment_frame import (
    experiment_points,
    internal_to_physical,
    inverse_kinematics_physical,
    physical_to_internal,
    validate_physical_target,
)
from kinematics import (
    JointAngles,
    MotorAngles,
    degrees_to_raw,
    forward_kinematics,
    forward_kinematics_from_joints,
    forward_kinematics_official,
    fk_to_motor_angles,
    inverse_kinematics,
    motor_to_fk_angles,
    raw_to_degrees,
    KinematicsError,
    XYZ,
)
from point_experiment import (
    MEASUREMENT_STOP,
    MEASUREMENT_WARNING,
    READY,
    plan_named_experiment_points,
    tcp_error_severity,
)


def check_home_fk() -> None:
    motors = MotorAngles(config.HOME_ID11, config.HOME_ID12, config.HOME_ID13, config.HOME_ID14)
    tip_xyz = forward_kinematics(motors)
    official_xyz = forward_kinematics_official(motors)
    tip_error = math.dist((tip_xyz.x, tip_xyz.y, tip_xyz.z), config.REST_XYZ_MM)
    official_error = math.dist(
        (official_xyz.x, official_xyz.y, official_xyz.z), config.REST_OFFICIAL_XYZ_MM
    )
    print("REST FK")
    print(f"working tip: {tip_xyz}; error={tip_error:.6f} mm")
    print(f"official gripper frame: {official_xyz}; error={official_error:.6f} mm")
    assert tip_error < 0.01
    assert official_error < 0.01


def check_rest_and_work_poses() -> None:
    rest = JointAngles(*config.REST_JOINT_DEGREES)
    work = JointAngles(*config.WORK_JOINT_DEGREES)
    rest_xyz = forward_kinematics_from_joints(rest)
    work_xyz = forward_kinematics_from_joints(work)
    work_motors = fk_to_motor_angles(work)
    print("NAMED REST / WORK POSES")
    print(f"REST q={rest.as_tuple()} XYZ={rest_xyz}")
    print(f"WORK q={work.as_tuple()} XYZ={work_xyz}")
    print(f"WORK motors={work_motors}")
    assert math.dist((rest_xyz.x, rest_xyz.y, rest_xyz.z), config.REST_XYZ_MM) < 0.01
    assert math.dist((work_xyz.x, work_xyz.y, work_xyz.z), config.WORK_XYZ_MM) < 0.01
    assert math.dist(work_motors.as_tuple(), config.WORK_MOTOR_DEGREES) < 0.01
    assert config.FK_JOINT_LIMITS["theta4"].contains(work.theta4)


def check_ik_round_trip(label: str, target: tuple[float, float, float]) -> None:
    result = inverse_kinematics(*target)
    print(label)
    print(f"Target XYZ: X={target[0]:.3f}, Y={target[1]:.3f}, Z={target[2]:.3f}")
    print(
        "IK angles: "
        f"theta1={result.joint_angles.theta1:.3f}, theta2={result.joint_angles.theta2:.3f}, "
        f"theta3={result.joint_angles.theta3:.3f}, theta4={result.joint_angles.theta4:.3f}"
    )
    print(
        "Motor angles: "
        f"ID11={result.motor_angles.id11:.3f}, ID12={result.motor_angles.id12:.3f}, "
        f"ID13={result.motor_angles.id13:.3f}, ID14={result.motor_angles.id14:.3f}"
    )
    print(
        "FK reconstructed XYZ: "
        f"X={result.xyz_check.x:.3f}, Y={result.xyz_check.y:.3f}, Z={result.xyz_check.z:.3f}"
    )
    print(f"dX={result.dx:.6f}, dY={result.dy:.6f}, dZ={result.dz:.6f}")
    print(f"total error={result.position_error:.6f} mm")
    assert result.position_error <= 0.05


def check_conversion_layer() -> None:
    print("RAW/DEGREE CONVERSION")
    print(f"raw 1024 -> {raw_to_degrees(1024):.3f} deg")
    print(f"90 deg -> {degrees_to_raw(90.0)} raw")
    assert abs(raw_to_degrees(1024) - 90.0) < 1e-9
    assert degrees_to_raw(90.0) == 1024


def check_screenshot_work_pose() -> None:
    motors = MotorAngles(168.5742198507731, 179.6484352984539, 355.6933571734539, 82.880859375)
    joints = motor_to_fk_angles(motors)
    xyz = forward_kinematics(motors)
    print("SCREENSHOT WORK POSE")
    print(f"motors={motors.as_tuple()}")
    print(f"q={tuple(round(v, 3) for v in joints.as_tuple())}")
    print(f"working-tip XYZ=({xyz.x:.3f},{xyz.y:.3f},{xyz.z:.3f})")
    assert abs(joints.theta4 - 82.880859375) < 0.001
    # Regression values for this captured encoder sample using the calibrated
    # 158.7 mm center-between-fingers TCP.
    assert abs(xyz.x - 180.239) < 0.01
    assert abs(xyz.z - 47.981) < 0.01

    boundary = MotorAngles(*(raw_to_degrees(value) for value in (1917, 2046, -47, 943)))
    assert abs(motor_to_fk_angles(boundary).theta3) < 1e-9


def check_experiment_frame() -> None:
    print("ID11-CENTERED PHYSICAL EXPERIMENT FRAME")
    assert config.MEASURED_XYZ_LIMITS["x"].minimum == -100.0
    assert config.MEASURED_XYZ_LIMITS["x"].maximum == 350.0
    assert config.MEASURED_XYZ_LIMITS["y"].minimum == -200.0
    assert config.MEASURED_XYZ_LIMITS["y"].maximum == 200.0
    physical = XYZ(-35.0, 145.0, 0.0)
    internal = physical_to_internal(physical)
    print(f"P01 physical={physical}; transformed internal={internal}")
    assert internal == XYZ(-35.0, -145.0, 0.0)
    assert internal_to_physical(internal) == physical
    for point in experiment_points().values():
        validate_physical_target(point)
    for name in ("P01", "P07"):
        point = experiment_points()[name]
        result = inverse_kinematics_physical(point.x, point.y, point.z)
        print(f"{name}: READY at q1={result.ik.joint_angles.theta1:.3f} deg")
        assert config.FK_JOINT_LIMITS["theta1"].contains(result.ik.joint_angles.theta1)


def check_point_experiment_plan() -> None:
    print("P01-P11 APPROACH / TOUCH / RETRACT PLAN")
    plan = plan_named_experiment_points()
    expected = {
        "P01": READY,
        "P02": READY,
        "P03": READY,
        "P04": READY,
        "P05": READY,
        "P06": READY,
        "P07": READY,
        "P08": READY,
        "P09": READY,
        "P10": READY,
        "P11": READY,
    }
    assert plan.ready_count == 11
    assert plan.blocked_count == 0
    for point in plan.points:
        print(f"{point.name}: {point.status}; {point.message}")
        assert point.status == expected[point.name]
        if point.motion_ready:
            assert point.touch_fk_error_mm is not None
            assert point.touch_fk_error_mm <= config.IK_POSITION_TOLERANCE_MM
    outside = XYZ(-100.0, 100.0, 0.0)
    try:
        inverse_kinematics_physical(outside.x, outside.y, outside.z)
    except KinematicsError as exc:
        assert "UNREACHABLE / JOINT LIMIT" in str(exc)
    else:
        raise AssertionError("A q1 target outside +/-110 deg unexpectedly passed")
    assert tcp_error_severity(5.651) == MEASUREMENT_WARNING
    assert tcp_error_severity(10.001) == MEASUREMENT_STOP


if __name__ == "__main__":
    check_home_fk()
    print()
    check_rest_and_work_poses()
    print()
    check_ik_round_trip("WORK TIP IK ROUND TRIP", config.WORK_XYZ_MM)
    print()
    check_ik_round_trip("SAFE LOW IK ROUND TRIP", (168.299, 10.806, 45.000))
    print()
    check_conversion_layer()
    print()
    check_screenshot_work_pose()
    print()
    check_experiment_frame()
    print()
    check_point_experiment_plan()
    print()
    print("All checks passed.")
