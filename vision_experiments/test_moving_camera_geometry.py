"""Offline transform and ray-plane checks; no camera or robot connection."""

from pathlib import Path
import sys
import unittest

import numpy as np

APP = Path(__file__).parent.parent / "openmanipulator_repeatability_project" / "python_app"
sys.path.insert(0, str(APP))

import config  # noqa: E402
from kinematics import (JointAngles, forward_kinematics_from_joints,
                        forward_kinematics_official_from_joints)  # noqa: E402
from moving_camera_geometry import (GeometryUnavailable, MovingCameraCalibration,
                                    base_to_gripper, localize_pixel_on_cube_top)  # noqa: E402


class MovingCameraGeometryTests(unittest.TestCase):
    def test_gripper_orientation_matches_existing_fk_tool_extension(self) -> None:
        joints = JointAngles(17.0, -20.0, -25.7, 124.5)
        pose = base_to_gripper(joints)
        official = forward_kinematics_official_from_joints(joints)
        tip = forward_kinematics_from_joints(joints)
        delta = np.array([tip.x - official.x, tip.y - official.y, tip.z - official.z])
        self.assertTrue(np.allclose(delta, config.GRIPPER_TIP_EXTENSION * pose[:3, 0], atol=1e-9))
        self.assertTrue(np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-12))
        self.assertAlmostEqual(np.linalg.det(pose[:3, :3]), 1.0, places=12)

    def test_ray_intersects_cube_top_in_physical_frame(self) -> None:
        joints = JointAngles(0.0, -20.0, -25.7, 124.5)
        wanted_camera = np.eye(4)
        wanted_camera[:3, :3] = np.diag([1.0, -1.0, -1.0])
        wanted_camera[:3, 3] = [200.0, 0.0, 150.0]
        gripper_to_camera = np.linalg.inv(base_to_gripper(joints)) @ wanted_camera
        calibration = MovingCameraCalibration(
            (640, 480), np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]]),
            np.zeros(5), gripper_to_camera, 20.0, "synthetic fixture")
        result = localize_pixel_on_cube_top(420, 290, joints, calibration, (640, 480))
        self.assertAlmostEqual(result.x, 226.0, places=6)
        self.assertAlmostEqual(result.y, 13.0, places=6)
        self.assertAlmostEqual(result.z, 20.0, places=6)
        with self.assertRaises(GeometryUnavailable):
            localize_pixel_on_cube_top(420, 290, joints, calibration, (800, 600))


if __name__ == "__main__":
    unittest.main()
