import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from kinematics import JointAngles, XYZ
from point_experiment import (
    GROUND_PROTECTION,
    READY,
    SOFT_LIMIT,
    UNREACHABLE_GEOMETRY,
    UNREACHABLE_JOINT_LIMIT,
    MEASUREMENT_OK,
    MEASUREMENT_STOP,
    MEASUREMENT_WARNING,
    plan_named_experiment_points,
    plan_point_experiment,
    tcp_error_severity,
)


def test_named_points_are_planned_in_config_order_with_configured_clearance():
    plan = plan_named_experiment_points()
    assert [point.name for point in plan.points] == [f"P{index:02d}" for index in range(1, 12)]
    assert plan.approach_clearance_mm == config.APPROACH_CLEARANCE_MM
    assert plan.retract_clearance_mm == config.RETRACT_CLEARANCE_MM
    for point in plan.points:
        assert point.approach_target.z == point.touch_target.z + config.APPROACH_CLEARANCE_MM
        assert point.retract_target.z == point.touch_target.z + config.RETRACT_CLEARANCE_MM
        assert point.internal_target.y == -point.physical_target.y


def test_centimeter_points_have_safe_complete_motion_plans():
    plan = plan_named_experiment_points()
    by_name = {point.name: point for point in plan.points}
    for name in ("P08", "P09", "P10", "P11"):
        point = by_name[name]
        assert point.status == READY
        assert point.approach.ready
        assert point.touch.ready
        assert point.retract.ready
        assert point.motion_ready
        assert point.touch_fk_error_mm is not None
        assert point.touch_fk_error_mm <= config.IK_POSITION_TOLERANCE_MM


def test_ready_touch_exposes_selected_joints_motors_and_fk_error():
    points = {"WORK": XYZ(*config.WORK_XYZ_MM)}
    plan = plan_point_experiment(points, approach_clearance_mm=0.0, retract_clearance_mm=0.0)
    point = plan.points[0]
    assert point.status == READY
    assert point.motion_ready
    assert point.selected_joints is not None
    assert point.selected_motors is not None
    assert point.touch_fk_error_mm is not None
    assert point.touch_fk_error_mm <= config.IK_POSITION_TOLERANCE_MM
    assert point.touch.physical_fk_check is not None
    assert math.dist(
        point.touch.physical_fk_check.__dict__.values(),
        point.physical_target.__dict__.values(),
    ) <= config.IK_POSITION_TOLERANCE_MM


def test_all_required_failure_categories_are_reported_without_raising():
    cases = {
        "SOFT": (XYZ(-101.0, 0.0, 0.0), SOFT_LIMIT),
        "GROUND": (XYZ(100.0, 0.0, -1.0), GROUND_PROTECTION),
        "GEOMETRY": (XYZ(0.0, 0.0, 0.0), UNREACHABLE_GEOMETRY),
        "JOINT": (XYZ(-100.0, 100.0, 0.0), UNREACHABLE_JOINT_LIMIT),
    }
    for name, (target, expected) in cases.items():
        plan = plan_point_experiment(
            {name: target},
            approach_clearance_mm=0.0,
            retract_clearance_mm=0.0,
        )
        point = plan.points[0]
        assert point.status == expected
        assert not point.motion_ready
        assert point.touch.status == expected


def test_reference_advances_only_after_a_reachable_phase():
    points = {
        "A": XYZ(*config.WORK_XYZ_MM),
        "B": XYZ(*config.WORK_XYZ_MM),
    }
    start = JointAngles(0.0, 0.0, 0.0, 80.0)
    plan = plan_point_experiment(
        points,
        starting_reference=start,
        approach_clearance_mm=0.0,
        retract_clearance_mm=0.0,
    )
    first, second = plan.points
    assert first.approach.reference_joint_angles == start
    assert first.touch.reference_joint_angles == first.approach.joint_angles
    assert first.retract.reference_joint_angles == first.touch.joint_angles
    assert second.approach.reference_joint_angles == first.retract.joint_angles
    assert plan.final_reference == second.retract.joint_angles


def test_invalid_clearance_and_unknown_name_are_rejected():
    try:
        plan_point_experiment(approach_clearance_mm=-1.0)
    except ValueError as error:
        assert "approach_clearance_mm" in str(error)
    else:
        raise AssertionError("Negative approach clearance should be rejected")

    try:
        plan_point_experiment(names=["P99"])
    except KeyError as error:
        assert "P99" in str(error)
    else:
        raise AssertionError("Unknown point name should be rejected")


def test_measured_tcp_error_uses_warning_then_hard_stop():
    assert tcp_error_severity(4.999, warning_mm=5.0, stop_mm=10.0) == MEASUREMENT_OK
    assert tcp_error_severity(5.651, warning_mm=5.0, stop_mm=10.0) == MEASUREMENT_WARNING
    assert tcp_error_severity(10.001, warning_mm=5.0, stop_mm=10.0) == MEASUREMENT_STOP
