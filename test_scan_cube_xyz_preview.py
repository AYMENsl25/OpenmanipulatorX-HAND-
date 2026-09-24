"""Offline geometry and center/edge regression tests; never opens a robot port."""

from pathlib import Path
import json
import math
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from scan_cube_xyz_preview import (ScanCalibration, classify_candidate, process_frame,
                                   rotate_center_calibration)


CONFIG = Path(__file__).with_name("scan_cube_xyz_config.json")
IMAGES = Path(__file__).parent / "data/opencv_tests_v3/images"


class ScanPreviewTests(unittest.TestCase):
    def setUp(self) -> None:
        self.cal = ScanCalibration.load(CONFIG)
        self.cal.image_points_uv = {}
        self.cal.calibrated_joints_deg = None

    def test_no_made_up_pixel_references(self) -> None:
        self.assertIsNone(self.cal.matrix())
        self.assertIsNone(self.cal.pixel_to_xy(320, 240))

    def test_four_measured_correspondences_allow_asymmetric_homography(self) -> None:
        # Deliberately asymmetric: the image center does not map to Y=0.
        self.cal.image_points_uv = {
            "P1": (70, 430),
            "P2": (70, 40),
            "P3": (620, 430),
            "P4": (620, 40),
        }
        for name, uv in self.cal.image_points_uv.items():
            actual = self.cal.pixel_to_xy(*uv)
            expected = self.cal.references_xy_mm[name]
            self.assertAlmostEqual(actual[0], expected[0], places=3)
            self.assertAlmostEqual(actual[1], expected[1], places=3)
        self.assertNotAlmostEqual(self.cal.pixel_to_xy(320, 240)[1], 0, delta=1)

    def test_extra_point_is_checked_without_fitting_it(self) -> None:
        self.cal.image_points_uv = {
            "P1": (70, 430), "P2": (70, 40),
            "P3": (620, 430), "P4": (620, 40),
            "P5": (330, 27),
        }
        self.assertGreater(self.cal.corner_check_errors_mm()["P5"], 5)
        self.cal.image_points_uv.pop("P5")
        self.assertEqual(self.cal.corner_check_errors_mm(), {})

    def test_old_saved_names_and_matching_manual_point_migrate_in_memory(self) -> None:
        original = {
            "pose": "CENTER", "image_size": [640, 480],
            "physical_references_xy_mm": {
                "P1_X130_Y+134": [130, 134], "P2_X300_Y+134": [300, 134],
                "P3_X130_Y-90": [130, -90], "P4_X300_Y-90": [300, -90],
                "M1": [295, 0],
            },
            "image_points_uv": {
                "P1_X130_Y+134": [637, 451], "P2_X300_Y+134": [598, 37],
                "P3_X130_Y-90": [26, 453], "P4_X300_Y-90": [45, 34],
                "M1": [308, 22],
            },
            "joint_angles_deg": [0, -20, -25, 124],
        }
        config_text = CONFIG.read_text(encoding="utf-8")
        saved_path = CONFIG.parent / "data/vision_calibration/center.json"
        with patch.object(Path, "exists", lambda path: path == saved_path), patch.object(
            Path, "read_text",
            lambda path, encoding=None: json.dumps(original) if path == saved_path else config_text,
        ):
            migrated = ScanCalibration.load(CONFIG)
        self.assertEqual(set(migrated.image_points_uv), {"P1", "P2", "P3", "P4", "P5"})
        self.assertEqual(migrated.references_xy_mm["P5"], (295.0, 0.0))
        self.assertNotIn("M1", migrated.references_xy_mm)
        self.assertIsNotNone(migrated.matrix())

    def test_cannot_reuse_center_points_for_left_pose(self) -> None:
        self.assertIsNone(ScanCalibration.load(CONFIG, "LEFT").matrix())

    def test_center_yaw_transfer_uses_measured_angle_and_physical_y_sign(self) -> None:
        self.cal.image_points_uv = {
            "P1": (70, 430), "P2": (70, 40),
            "P3": (620, 430), "P4": (620, 40),
        }
        self.cal.calibrated_joints_deg = (0.0, -20.0, -25.7, 124.5)
        frame = {"x_scale": 1.0, "x_offset_mm": 0.0,
                 "y_scale": -1.0, "y_offset_mm": 0.0}
        center_pixel = np.linalg.inv(self.cal.matrix()) @ np.array([200.0, 0.0, 1.0])
        center_pixel /= center_pixel[2]
        moved = rotate_center_calibration(self.cal, (10.0, -20.0, -25.7, 124.5), frame)
        self.assertIsNotNone(moved)
        x, y = moved.pixel_to_xy(float(center_pixel[0]), float(center_pixel[1]))
        self.assertAlmostEqual(x, 200.0 * math.cos(math.radians(10.0)), delta=0.01)
        self.assertAlmostEqual(y, -200.0 * math.sin(math.radians(10.0)), delta=0.01)
        self.assertEqual(moved.mapping_mode, "ROTATED_CENTER")
        self.assertIsNone(rotate_center_calibration(
            self.cal, (10.0, -17.0, -25.7, 124.5), frame))

    def test_crossed_reference_clicks_do_not_make_xy(self) -> None:
        self.cal.image_points_uv = {
            "P1": (70, 430), "P2": (620, 40),
            "P3": (620, 430), "P4": (70, 40),
        }
        self.assertIsNone(self.cal.matrix())

    def test_gui_calibration_requires_matching_encoder_pose(self) -> None:
        self.cal.calibrated_joints_deg = (0.0, -20.0, -25.8, 124.6)
        self.assertTrue(self.cal.matches_joints((0.2, -20.1, -25.7, 124.4)))
        self.assertFalse(self.cal.matches_joints((10.0, -20.1, -25.7, 124.4)))

    def test_edge_candidate_has_no_xy(self) -> None:
        candidate = {"object_id": "cube_1", "center_u": 6, "center_v": 240,
                     "bbox_xywh": (0, 220, 35, 40), "touches_image_edge": True,
                     "width_px": 35, "height_px": 40, "area_px": 1200}
        result = classify_candidate(candidate, self.cal)
        self.assertEqual((result["status"], result["nearest_edge"]), ("EDGE_RESCAN", "LEFT"))
        self.assertIsNone(result["provisional_y_mm"])

    def test_full_frame_scan_roi_keeps_near_border_candidate(self) -> None:
        self.assertEqual(self.cal.edge_margin_px, 0)
        candidate = {"object_id": "cube_1", "center_u": 35, "center_v": 240,
                     "bbox_xywh": (10, 215, 50, 50), "touches_image_edge": False,
                     "width_px": 50, "height_px": 50, "area_px": 2500}
        self.assertEqual(classify_candidate(candidate, self.cal)["status"], "VALID")
        candidate["bbox_xywh"] = (0, 215, 50, 50)
        candidate["touches_image_edge"] = True
        self.assertEqual(classify_candidate(candidate, self.cal)["status"], "EDGE_RESCAN")

    def test_ten_mm_padding_is_detection_only(self) -> None:
        self.cal.image_points_uv = {
            "P1": (70, 430), "P2": (70, 40),
            "P3": (620, 430), "P4": (620, 40),
        }
        self.assertEqual(self.cal.physical_bounds_padding_mm, 10)
        candidate = {"object_id": "cube_1", "center_u": 392, "center_v": 28,
                     "bbox_xywh": (382, 20, 20, 16), "touches_image_edge": False,
                     "width_px": 20, "height_px": 16, "area_px": 320}
        near = classify_candidate(candidate, self.cal)
        self.assertEqual(near["status"], "VALID")
        self.assertIsNotNone(near["provisional_x_mm"])
        candidate["center_v"] = 11
        candidate["bbox_xywh"] = (382, 3, 20, 16)
        beyond = classify_candidate(candidate, self.cal)
        self.assertEqual(beyond["status"], "VALID")
        self.assertIsNone(beyond["provisional_x_mm"])

    def test_six_synthetic_scenes(self) -> None:
        scenes = {
            "center": [(280, 210, 340, 270)],
            "left": [(0, 210, 45, 270)],
            "right": [(600, 210, 640, 270)],
            "top": [(290, 0, 350, 45)],
            "bottom": [(290, 435, 350, 480)],
            "two": [(120, 180, 180, 240), (420, 180, 480, 240)],
        }
        for name, boxes in scenes.items():
            with self.subTest(name=name):
                frame = np.full((480, 640, 3), 190, np.uint8)
                for x1, y1, x2, y2 in boxes:
                    cv2.rectangle(frame, (x1, y1), (x2 - 1, y2 - 1), (10, 10, 10), -1)
                _view, results, _details = process_frame(frame, self.cal)
                self.assertEqual(len(results), len(boxes))
                expected = "VALID" if name in ("center", "two") else "EDGE_RESCAN"
                self.assertTrue(all(r["status"] == expected for r in results))
                self.assertTrue(all(r["provisional_x_mm"] is None for r in results))

    def test_saved_near_bottom_frame_keeps_two_blobs(self) -> None:
        path = IMAGES / "test_20260922_170639_137072_raw.png"
        if not path.exists():
            self.skipTest("Saved camera image not present")
        frame = cv2.imread(str(path))
        _view, results, details = process_frame(frame, self.cal)
        self.assertEqual(details["detections"], 2)
        self.assertEqual({r["status"] for r in results}, {"VALID"})


if __name__ == "__main__":
    unittest.main()
