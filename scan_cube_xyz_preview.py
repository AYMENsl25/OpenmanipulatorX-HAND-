"""Camera-only black-cube detection and measured fixed-pose provisional ground XY."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from opencv_cube_test_v3 import find_candidates, make_mask


ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "scan_cube_xyz_config.json"
CALIBRATION_DIR = ROOT / "data" / "vision_calibration"
OBJECT_COLORS = ((255, 255, 0), (255, 0, 255), (0, 255, 255),
                 (0, 220, 0), (0, 165, 255))  # BGR object IDs, not physical colors
LEGACY_REFERENCE_NAMES = {
    "P1_X130_Y+134": "P1", "P2_X300_Y+134": "P2",
    "P3_X130_Y-90": "P3", "P4_X300_Y-90": "P4",
    "P3_X130_Y-134": "P3", "P4_X300_Y-134": "P4",
}


@dataclass
class ScanCalibration:
    pose: str
    image_width: int
    image_height: int
    references_xy_mm: dict[str, tuple[float, float]]
    image_points_uv: dict[str, tuple[float, float]] = field(default_factory=dict)
    edge_margin_px: int = 12
    physical_bounds_padding_mm: float = 0.0
    reference_frame: str = "ID11_REAR_FACE_ESTIMATED"
    threshold: int = 46
    calibrated_joints_deg: tuple[float, float, float, float] | None = None
    projection_matrix: np.ndarray | None = None
    mapping_mode: str = "FIXED_POSE"
    yaw_delta_deg: float = 0.0

    @classmethod
    def load(cls, config_path: Path = DEFAULT_CONFIG, pose: str = "CENTER") -> "ScanCalibration":
        data = json.loads(config_path.read_text(encoding="utf-8"))
        refs = {name: tuple(map(float, xy)) for name, xy in data["physical_references_xy_mm"].items()}
        result = cls(pose.upper(), int(data["image_width"]), int(data["image_height"]),
                     refs, edge_margin_px=int(data.get("edge_margin_px", 12)),
                     physical_bounds_padding_mm=float(data.get("physical_bounds_padding_mm", 0.0)),
                     reference_frame=str(data.get("reference_frame", "ID11_REAR_FACE_ESTIMATED")),
                     threshold=int(data.get("threshold", 46)))
        saved = config_path.parent / "data" / "vision_calibration" / f"{result.pose.lower()}.json"
        if saved.exists():
            observed = json.loads(saved.read_text(encoding="utf-8"))
            stored_refs = observed.get("physical_references_xy_mm", {})
            if (observed.get("pose") == result.pose
                    and observed.get("reference_frame", result.reference_frame) == result.reference_frame
                    and observed.get("image_size") == [result.image_width, result.image_height]):
                renamed = dict(LEGACY_REFERENCE_NAMES)
                for name, xy in stored_refs.items():
                    if not name.startswith("M"):
                        continue
                    for known_name, known_xy in result.references_xy_mm.items():
                        if all(abs(float(a) - b) < 1e-6 for a, b in zip(xy, known_xy, strict=True)):
                            renamed[name] = known_name
                            break
                result.references_xy_mm.update({
                    name: tuple(map(float, xy)) for name, xy in stored_refs.items()
                    if name.startswith("M") and name not in renamed
                })
                result.image_points_uv = {
                    renamed.get(name, name): tuple(map(float, uv))
                    for name, uv in observed.get("image_points_uv", {}).items()
                    if renamed.get(name, name) in result.references_xy_mm
                }
                joints = observed.get("joint_angles_deg")
                if joints is not None and len(joints) == 4:
                    result.calibrated_joints_deg = tuple(map(float, joints))
                else:
                    # Old click files had no pose binding; their pixels must be remeasured.
                    result.image_points_uv = {}
        return result

    def save(self, directory: Path = CALIBRATION_DIR,
             joints_deg: tuple[float, float, float, float] | None = None) -> Path:
        if joints_deg is not None:
            self.calibrated_joints_deg = tuple(map(float, joints_deg))
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{self.pose.lower()}.json"
        path.write_text(json.dumps({
            "pose": self.pose, "image_size": [self.image_width, self.image_height],
            "reference_frame": self.reference_frame,
            "physical_references_xy_mm": {k: list(v) for k, v in self.references_xy_mm.items()},
            "image_points_uv": {k: list(v) for k, v in self.image_points_uv.items()},
            "joint_angles_deg": (None if self.calibrated_joints_deg is None
                                  else list(self.calibrated_joints_deg)),
            "status": "PROVISIONAL_NOT_HAND_EYE_CALIBRATED",
        }, indent=2) + "\n", encoding="utf-8")
        return path

    def matches_joints(self, joints, tolerance_deg: float = 1.0) -> bool:
        if self.calibrated_joints_deg is None or joints is None:
            return False
        values = joints.as_tuple() if hasattr(joints, "as_tuple") else tuple(joints)
        return max(abs(a - b) for a, b in zip(values, self.calibrated_joints_deg, strict=True)) <= tolerance_deg

    def matrix(self) -> np.ndarray | None:
        if self.projection_matrix is not None:
            return self.projection_matrix
        names = [name for name in self.references_xy_mm if name in self.image_points_uv]
        if len(names) < 4:
            return None
        if any(not (0 <= u < self.image_width and 0 <= v < self.image_height)
               for u, v in self.image_points_uv.values()):
            return None
        corners = ("P1", "P2", "P4", "P3")
        if all(name in self.image_points_uv for name in corners):
            quad = np.asarray([self.image_points_uv[name] for name in corners], np.float32)
            if not cv2.isContourConvex(quad) or abs(cv2.contourArea(quad)) < 1000:
                return None
        pixels = np.asarray([self.image_points_uv[name] for name in names], np.float32)
        ground = np.asarray([self.references_xy_mm[name] for name in names], np.float32)
        if np.linalg.matrix_rank(np.column_stack((pixels, np.ones(len(pixels))))) < 3:
            return None
        matrix, _ = cv2.findHomography(pixels, ground, method=0)
        if matrix is None or not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix)) < 1e-12:
            return None
        return matrix

    def pixel_to_xy(self, u: float, v: float) -> tuple[float, float] | None:
        matrix = self.matrix()
        if matrix is None:
            return None
        mapped = cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), matrix)[0, 0]
        return (float(mapped[0]), float(mapped[1])) if np.all(np.isfinite(mapped)) else None

    def corner_check_errors_mm(self) -> dict[str, float]:
        """Check extra clicks against P1-P4 without fitting those clicks.

        Four corners alone can fit perfectly even when their physical positions
        or clicks are wrong. Extra points supply an independent consistency check.
        """
        corners = ("P1", "P2", "P4", "P3")
        if not all(name in self.image_points_uv and name in self.references_xy_mm
                   for name in corners):
            return {}
        corner_only = ScanCalibration(
            pose=self.pose, image_width=self.image_width, image_height=self.image_height,
            references_xy_mm={name: self.references_xy_mm[name] for name in corners},
            image_points_uv={name: self.image_points_uv[name] for name in corners},
            reference_frame=self.reference_frame,
        )
        if corner_only.matrix() is None:
            return {}
        errors = {}
        for name, uv in self.image_points_uv.items():
            if name in corners or name not in self.references_xy_mm:
                continue
            estimate = corner_only.pixel_to_xy(*uv)
            if estimate is not None:
                x, y = self.references_xy_mm[name]
                errors[name] = math.hypot(estimate[0] - x, estimate[1] - y)
        return errors


def rotate_center_calibration(center: ScanCalibration, joints,
                              physical_to_internal: dict,
                              posture_tolerance_deg: float = 1.0) -> ScanCalibration | None:
    """Transfer a measured center plane homography through a measured J1 yaw.

    Valid only for a rigid camera mount and unchanged J2-J4. The result remains
    a plate-plane estimate; it does not correct cube-top parallax.
    """
    if center.pose != "CENTER" or center.calibrated_joints_deg is None:
        return None
    current = joints.as_tuple() if hasattr(joints, "as_tuple") else tuple(joints)
    if len(current) != 4 or any(not math.isfinite(float(q)) for q in current):
        return None
    if any(abs(float(a) - b) > posture_tolerance_deg for a, b in
           zip(current[1:], center.calibrated_joints_deg[1:], strict=True)):
        return None
    homography = center.matrix()
    if homography is None:
        return None
    angle = math.radians(float(current[0]) - center.calibrated_joints_deg[0])
    if abs(math.degrees(angle)) > 45.0:
        return None
    c, s = math.cos(angle), math.sin(angle)
    physical_to_internal_xy = np.array([
        [float(physical_to_internal["x_scale"]), 0.0,
         float(physical_to_internal["x_offset_mm"])],
        [0.0, float(physical_to_internal["y_scale"]),
         float(physical_to_internal["y_offset_mm"])],
        [0.0, 0.0, 1.0],
    ])
    if abs(np.linalg.det(physical_to_internal_xy)) < 1e-12:
        return None
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    rotated = np.linalg.inv(physical_to_internal_xy) @ yaw @ physical_to_internal_xy @ homography
    if not np.all(np.isfinite(rotated)):
        return None
    return ScanCalibration(
        pose="CENTER", image_width=center.image_width, image_height=center.image_height,
        references_xy_mm=center.references_xy_mm.copy(), image_points_uv={},
        edge_margin_px=center.edge_margin_px,
        physical_bounds_padding_mm=center.physical_bounds_padding_mm,
        reference_frame=center.reference_frame,
        threshold=center.threshold,
        calibrated_joints_deg=center.calibrated_joints_deg,
        projection_matrix=rotated, mapping_mode="ROTATED_CENTER",
        yaw_delta_deg=math.degrees(angle),
    )


def nearest_edge(u: float, v: float, width: int, height: int) -> str:
    return min(((u, "LEFT"), (width - 1 - u, "RIGHT"),
                (v, "TOP"), (height - 1 - v, "BOTTOM")))[1]


def classify_candidate(candidate: dict, calibration: ScanCalibration) -> dict:
    h, w = calibration.image_height, calibration.image_width
    u, v = int(candidate["center_u"]), int(candidate["center_v"])
    x, y, bw, bh = candidate["bbox_xywh"]
    margin = calibration.edge_margin_px
    edge = bool(candidate["touches_image_edge"] or x <= margin or y <= margin
                or x + bw >= w - margin or y + bh >= h - margin)
    status = "EDGE_RESCAN" if edge else "VALID"
    direction = nearest_edge(u, v, w, h) if edge else None
    xy = calibration.pixel_to_xy(u, v) if status == "VALID" else None
    if xy is not None:
        xs, ys = zip(*calibration.references_xy_mm.values())
        pad = calibration.physical_bounds_padding_mm
        if not (min(xs) - pad <= xy[0] <= max(xs) + pad
                and min(ys) - pad <= xy[1] <= max(ys) + pad):
            xy = None
    return {
        "object_id": candidate["object_id"], "class": "cube", "status": status,
        "center_u": u, "center_v": v, "bbox_xywh": [x, y, bw, bh],
        "width_px": candidate["width_px"], "height_px": candidate["height_px"],
        "contour_area": round(float(candidate["area_px"]), 1),
        "provisional_x_mm": None if xy is None else round(xy[0], 1),
        "provisional_y_mm": None if xy is None else round(xy[1], 1),
        "xy_method": ("NONE" if xy is None else calibration.mapping_mode),
        "nearest_edge": direction,
        "rescan_hint": None if direction is None else
        f"Toward image {direction}; Cartesian sign needs verified camera transform",
    }


def process_frame(frame: np.ndarray, calibration: ScanCalibration) -> tuple[np.ndarray, list[dict], dict]:
    if frame is None or frame.shape[:2] != (calibration.image_height, calibration.image_width):
        raise ValueError("Camera resolution differs from this scan pose's reference resolution")
    start = time.perf_counter()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask, _, used = make_mask(gray, 0, calibration.threshold)
    candidates = find_candidates(frame, mask, 200, 0.45, 0.70, include_edge=True)
    # The broad dark border outside the plate is not cube-sized.
    raw_count = len(candidates)
    candidates = [c for c in candidates if c["mean_gray"] <= 100
                  and c["area_px"] <= 12000
                  and max(c["bbox_xywh"][2:]) <= 150]
    for number, candidate in enumerate(candidates, 1):
        candidate["object_id"] = f"cube_{number}"
    results = [classify_candidate(c, calibration) for c in candidates]
    view = frame.copy()
    m = calibration.edge_margin_px
    cv2.rectangle(view, (m, m), (calibration.image_width - m - 1,
                                calibration.image_height - m - 1), (255, 0, 0), 1)
    for name, uv in calibration.image_points_uv.items():
        pt = tuple(round(value) for value in uv)
        cv2.circle(view, pt, 5, (0, 0, 255), -1)
        cv2.putText(view, name, (pt[0] + 5, pt[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
    for index, (candidate, result) in enumerate(zip(candidates, results, strict=True)):
        color = OBJECT_COLORS[index % len(OBJECT_COLORS)]
        cv2.polylines(view, [np.asarray(candidate["rotated_box"], np.int32)], True, color, 2)
        u, v = result["center_u"], result["center_v"]
        cv2.circle(view, (u, v), 4, color, -1)
        xy_text = ("XY UNCALIBRATED" if result["provisional_x_mm"] is None else
                   f"XY rear {result['provisional_x_mm']:.0f},{result['provisional_y_mm']:.0f} PROVISIONAL")
        label = f"{result['object_id']} {result['status']} {result['nearest_edge'] or ''} {xy_text}"
        pos = (max(4, min(u - 60, calibration.image_width - 400)), max(19, v - 14))
        cv2.putText(view, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3)
        cv2.putText(view, label, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)
    geometry = ("XY UNCALIBRATED" if calibration.matrix() is None else
                "ROTATED CENTER XY ESTIMATE" if calibration.mapping_mode == "ROTATED_CENTER" else
                "PROVISIONAL XY")
    cv2.putText(view, f"{calibration.pose} | {geometry} | NO ROBOT MOTION", (8, calibration.image_height - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.43, (255, 255, 255), 1)
    return view, results, {
        "scan_pose": calibration.pose, "reference_frame": calibration.reference_frame,
        "geometry": geometry, "threshold": used,
        "detections": len(results), "valid": sum(r["status"] == "VALID" for r in results),
        "edge_rescan": sum(r["status"] == "EDGE_RESCAN" for r in results),
        "rejected_candidates": raw_count - len(candidates),
        "processing_ms": round((time.perf_counter() - start) * 1000, 1), "objects": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--index", type=int)
    parser.add_argument("--pose", choices=("CENTER", "LEFT", "RIGHT"), default="CENTER")
    parser.add_argument("--joints", nargs=4, type=float, metavar=("J1", "J2", "J3", "J4"),
                        help="Measured joint angles for this image and fixed camera pose")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    calibration = ScanCalibration.load(args.config, args.pose)
    if not calibration.matches_joints(args.joints):
        calibration.image_points_uv = {}
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit(f"Cannot open {args.image}")
        view, _, details = process_frame(frame, calibration)
        print(json.dumps(details, indent=2))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.output), view)
        return
    camera = cv2.VideoCapture(args.index)
    if not camera.isOpened():
        raise SystemExit(f"Cannot open camera index {args.index}")
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise SystemExit("Camera read failed")
            view, _, _ = process_frame(frame, calibration)
            cv2.imshow("SCAN cube preview | Q quit | no robot motion", view)
            if cv2.waitKey(1) & 0xff in (ord("q"), ord("Q")):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
