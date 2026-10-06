"""Offline scan planning tests; no GUI window, serial port, or motor command."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import csv
import io
import math
import sys
import unittest

import cv2
import numpy as np

APP = Path(__file__).parent.parent / "openmanipulator_repeatability_project" / "python_app"
sys.path.insert(0, str(APP))
sys.path.insert(0, str(Path(__file__).parent))

from kinematics import JointAngles, XYZ  # noqa: E402
from scan_cube_xyz_preview import ScanCalibration, rotate_center_calibration  # noqa: E402
from rear_reference_frame import rear_to_internal_transform  # noqa: E402
from vision_panel import VisionPanel  # noqa: E402


class VisionScanLogicTests(unittest.TestCase):
    def test_scan_csv_labels_axis_fk_and_rear_plate_x(self) -> None:
        class KeepOpen(io.StringIO):
            def close(self):
                pass

        output = KeepOpen()
        result = {
            "object_id": "cube_1", "status": "VALID", "center_u": 320,
            "center_v": 200, "width_px": 40, "height_px": 42,
            "contour_area": 1500.0, "provisional_x_mm": 220.0,
            "provisional_y_mm": 90.0, "xy_method": "FIXED_POSE", "nearest_edge": None,
        }
        state = SimpleNamespace(
            joints=SimpleNamespace(as_tuple=lambda: (0.0, -20.0, -25.7, 124.5)),
            physical_xyz=XYZ(195.0, 90.0, 50.0),
        )
        with (patch.object(Path, "mkdir"), patch.object(Path, "exists", return_value=False),
              patch.object(Path, "open", return_value=output)):
            VisionPanel._append_csv(None, Path("unused.csv"), [result], state,
                                    "CENTER", "manual", 1)
        row = next(csv.DictReader(io.StringIO(output.getvalue())))
        self.assertEqual(row["fk_x"], "195.0")
        self.assertEqual(row["provisional_x"], "220.0")
        self.assertEqual(row["axis_x_preview"], "195.0")
        self.assertEqual(row["fk_frame"], "ID11_CENTER_AXIS")
        self.assertEqual(row["plate_frame"], "ID11_REAR_FACE_ESTIMATED")

    def test_center_yaw_transfer_rotates_about_id11_axis_not_rear_face(self) -> None:
        center = ScanCalibration.load()
        center.image_points_uv = {
            "P1": (70, 430), "P2": (70, 40),
            "P3": (620, 430), "P4": (620, 40),
        }
        center.calibrated_joints_deg = (0.0, -20.0, -25.7, 124.5)
        rear_point = np.array([[[220.0, 0.0]]], np.float32)
        pixel = cv2.perspectiveTransform(rear_point, np.linalg.inv(center.matrix()))[0, 0]
        moved = rotate_center_calibration(
            center, (10.0, -20.0, -25.7, 124.5), rear_to_internal_transform())
        self.assertIsNotNone(moved)
        actual = moved.pixel_to_xy(float(pixel[0]), float(pixel[1]))
        self.assertAlmostEqual(actual[0], 25 + 195 * math.cos(math.radians(10)), delta=0.01)
        self.assertAlmostEqual(actual[1], -195 * math.sin(math.radians(10)), delta=0.01)

    def test_incomplete_or_wrong_side_profile_uses_center_yaw_model(self) -> None:
        center = ScanCalibration.load()
        center.image_points_uv = {
            "P1": (626, 456), "P2": (588, 48),
            "P3": (19, 470), "P4": (26, 53),
        }
        center.calibrated_joints_deg = (0.0, -20.0, -25.7, 124.5)
        left = ScanCalibration("LEFT", 640, 480, center.references_xy_mm.copy(),
                               image_points_uv={"P3": (65, 453),
                                                "P4": (143, 19)},
                               calibrated_joints_deg=(10.0, -20.0, -25.7, 124.5))
        right = ScanCalibration("RIGHT", 640, 480, center.references_xy_mm.copy(),
                                image_points_uv=center.image_points_uv.copy(),
                                calibrated_joints_deg=(10.0, -20.0, -25.7, 124.5))
        panel = SimpleNamespace(
            center_calibration=center,
            left_mapping=SimpleNamespace(get=lambda: "LEFT = J1 +"),
            _pose_direction_matches=VisionPanel._pose_direction_matches,
        )
        state = SimpleNamespace(joints=JointAngles(10.0, -20.0, -25.7, 124.5))
        for profile in (left, right):
            effective = VisionPanel._effective_calibration(panel, profile, state)
            self.assertIsNotNone(effective.matrix())
            self.assertEqual(effective.mapping_mode, "ROTATED_CENTER")

    def test_both_area_scan_visits_each_step_and_returns_to_center(self) -> None:
        sequence = VisionPanel._scan_sequence(0.0, 1.0, 20.0, 10.0, "BOTH")
        self.assertEqual(sequence, [
            ("CENTER", 0.0), ("LEFT", 10.0), ("LEFT", 20.0),
            ("LEFT", 10.0), ("CENTER", 0.0), ("RIGHT", -10.0),
            ("RIGHT", -20.0), ("RIGHT", -10.0), ("CENTER", 0.0),
        ])

    def test_one_direction_scan_does_not_move_into_other_side(self) -> None:
        sequence = VisionPanel._scan_sequence(0.0, -1.0, 15.0, 10.0, "LEFT")
        self.assertEqual(sequence, [
            ("CENTER", 0.0), ("LEFT", -10.0), ("LEFT", -15.0),
            ("LEFT", -10.0), ("CENTER", 0.0),
        ])

    def test_j1_target_uses_existing_fk_and_preserves_other_joints(self) -> None:
        state = SimpleNamespace(
            joints=JointAngles(0.0, -20.0, -25.8, 124.6),
            physical_xyz=XYZ(108.1, 0.0, 138.2),
        )
        predicted, _motors, target = VisionPanel._target_from_j1(state, 10.0)
        self.assertEqual(predicted.as_tuple(), (10.0, -20.0, -25.8, 124.6))
        self.assertGreater(abs(target.y), 1.0)

    def test_scanning_requires_named_camera_posture(self) -> None:
        normal = SimpleNamespace(joints=JointAngles(0.0, -20.0, -25.8, 124.6))
        VisionPanel._assert_scan_posture(normal)
        wrong_height = SimpleNamespace(joints=JointAngles(0.0, 0.0, 0.0, 82.9))
        with self.assertRaisesRegex(Exception, "not near the configured camera SCAN posture"):
            VisionPanel._assert_scan_posture(wrong_height)

    def test_majority_requires_three_of_five_frames(self) -> None:
        cube = {"object_id": "cube_1", "center_u": 100, "center_v": 200,
                "status": "VALID", "provisional_x_mm": None, "provisional_y_mm": None}
        noise = {"object_id": "cube_2", "center_u": 400, "center_v": 200,
                 "status": "VALID", "provisional_x_mm": None, "provisional_y_mm": None}
        frames = [[cube, noise], [cube], [cube], [], []]
        stable = VisionPanel._majority_results(frames)
        self.assertEqual([item["object_id"] for item in stable], ["cube_1"])

    def test_majority_tracks_position_when_frame_ids_shift(self) -> None:
        a = {"object_id": "cube_1", "center_u": 100, "center_v": 200,
             "status": "VALID", "provisional_x_mm": None, "provisional_y_mm": None}
        b = {"object_id": "cube_2", "center_u": 400, "center_v": 200,
             "status": "VALID", "provisional_x_mm": None, "provisional_y_mm": None}
        shifted_b = dict(b, object_id="cube_1")
        stable = VisionPanel._majority_results([[a, b], [a, b], [shifted_b], [a, b], [a, b]])
        self.assertEqual([item["center_u"] for item in stable], [100, 400])


if __name__ == "__main__":
    unittest.main()
