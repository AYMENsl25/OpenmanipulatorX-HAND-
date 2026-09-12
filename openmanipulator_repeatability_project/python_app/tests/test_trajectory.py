import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from experiment_frame import validate_physical_target
from kinematics import KinematicsError, MotorAngles, XYZ
from trajectory import TrajectoryPoint, XYZTrajectoryPoint, endpoint_points, xyz_endpoint_points


def test_endpoint_points_preserve_first_and_last_teach_samples():
    first = TrajectoryPoint(0.0, MotorAngles(351, 1, 1, 0), XYZ(1, 2, 3))
    last = TrajectoryPoint(1.0, MotorAngles(350, 2, 3, 4), XYZ(4, 5, 6))
    start, end = endpoint_points([first, last])
    assert start == first
    assert end == last


def test_xyz_endpoint_points_work_after_loading_xyz_workbook():
    first = XYZTrajectoryPoint(0.0, XYZ(10, 20, 30))
    last = XYZTrajectoryPoint(1.0, XYZ(40, 50, 60))
    assert xyz_endpoint_points([first, last]) == (first, last)


def test_measured_workspace_rejects_previous_outlier():
    try:
        validate_physical_target(XYZ(74.563, 389.0, 359.303))
    except KinematicsError:
        return
    raise AssertionError("Measured workspace should reject Y=389 mm")
