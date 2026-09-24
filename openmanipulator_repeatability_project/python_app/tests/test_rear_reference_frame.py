"""Offline rear-origin contract checks; never opens a serial port."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config  # noqa: E402
from kinematics import MotorAngles, XYZ  # noqa: E402
from rear_reference_frame import (axis_to_rear_xyz, rear_to_axis_xyz,
                                  rear_to_internal_transform)  # noqa: E402
from trajectory_gui import TrajectoryGUI  # noqa: E402


class RearReferenceFrameTests(unittest.TestCase):
    def test_rear_target_maps_to_axis_without_changing_legacy_frame(self) -> None:
        self.assertEqual(config.PHYSICAL_FRAME["origin"], "ID11_CENTER_AXIS")
        self.assertEqual(config.PHYSICAL_TO_INTERNAL["x_offset_mm"], 0.0)
        self.assertEqual(config.REAR_TO_AXIS_X_MM, 25.0)
        rear = XYZ(220.0, 90.0, 50.0)
        axis = rear_to_axis_xyz(rear)
        self.assertEqual(axis, XYZ(195.0, 90.0, 50.0))
        self.assertEqual(axis_to_rear_xyz(axis), rear)
        self.assertEqual(rear_to_internal_transform()["x_offset_mm"], -25.0)

    def test_manual_ik_receives_axis_target_once(self) -> None:
        axis_x, y, z = config.WORK_XYZ_MM
        fake_gui = SimpleNamespace(
            target_xyz={
                "X": SimpleNamespace(get=lambda: str(axis_x + 25.0)),
                "Y": SimpleNamespace(get=lambda: str(y)),
                "Z": SimpleNamespace(get=lambda: str(z)),
            },
            controller=SimpleNamespace(
                read_motor_angles=lambda: MotorAngles(*config.WORK_MOTOR_DEGREES)),
        )
        rear, physical_result, *_rest = TrajectoryGUI._nearest_ik_from_current(fake_gui)
        self.assertTrue(math.isclose(rear.x, axis_x + 25.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(physical_result.physical_target.x, axis_x, abs_tol=1e-9))
        self.assertTrue(math.isclose(physical_result.internal_target.x, axis_x, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()
