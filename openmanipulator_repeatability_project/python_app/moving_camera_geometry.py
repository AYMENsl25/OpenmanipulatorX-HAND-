"""Measured eye-in-hand geometry. No motor commands or estimated calibration values."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from experiment_frame import internal_to_physical, physical_to_internal
from kinematics import JointAngles, XYZ, forward_kinematics_official_pose_from_joints


DEFAULT_CALIBRATION = Path(__file__).resolve().parents[2] / "data" / "vision_calibration" / "moving_camera.json"


class GeometryUnavailable(ValueError):
    """Required measured geometry is absent, invalid, or inapplicable."""


def _rigid_transform(values, name: str) -> np.ndarray:
    transform = np.asarray(values, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise GeometryUnavailable(f"{name} must be a finite 4x4 transform")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-8)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3)):
        raise GeometryUnavailable(f"{name} is not a rigid right-handed transform")
    return transform


@dataclass(frozen=True)
class MovingCameraCalibration:
    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion: np.ndarray
    gripper_to_camera: np.ndarray
    cube_top_z_mm: float
    validation_evidence: str

    @classmethod
    def load(cls, path: Path = DEFAULT_CALIBRATION) -> "MovingCameraCalibration | None":
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("status") != "VALIDATED":
            raise GeometryUnavailable("Moving-camera calibration is not validated")
        image_size = tuple(data["image_size"])
        if len(image_size) != 2 or any(not isinstance(value, int) or value <= 0
                                       for value in image_size):
            raise GeometryUnavailable("Calibration needs a positive [width, height] image size")
        camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        if (camera_matrix.shape != (3, 3) or not np.all(np.isfinite(camera_matrix))
                or camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0
                or not np.allclose(camera_matrix[2], (0, 0, 1), atol=1e-8)):
            raise GeometryUnavailable("Camera intrinsics are invalid")
        distortion = np.asarray(data["distortion"], dtype=np.float64).reshape(-1)
        if distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
            raise GeometryUnavailable("Camera distortion coefficients are invalid")
        gripper_to_camera = _rigid_transform(data["T_gripper_camera"], "T_gripper_camera")
        cube_top_z_mm = float(data["cube_top_z_mm"])
        if not np.isfinite(cube_top_z_mm) or cube_top_z_mm < 0:
            raise GeometryUnavailable("Measured cube-top height is invalid")
        evidence = str(data.get("validation_evidence", "")).strip()
        if not evidence:
            raise GeometryUnavailable("A validated calibration needs validation evidence")
        return cls(image_size, camera_matrix, distortion, gripper_to_camera,
                   cube_top_z_mm, evidence)


def base_to_gripper(joints: JointAngles) -> np.ndarray:
    """T_base_gripper in the existing right-handed internal FK frame."""
    pose = forward_kinematics_official_pose_from_joints(joints)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(pose.rotation, dtype=np.float64)
    transform[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return transform


def base_to_camera(joints: JointAngles,
                   calibration: MovingCameraCalibration) -> np.ndarray:
    return base_to_gripper(joints) @ calibration.gripper_to_camera


def localize_pixel_on_cube_top(u: float, v: float, joints: JointAngles,
                               calibration: MovingCameraCalibration,
                               image_size: tuple[int, int]) -> XYZ:
    """Undistort a *top-face* pixel and intersect its base ray with cube top.

    A silhouette or bounding-box centre is not necessarily a top-face point.
    Callers must supply a pixel known to lie on the cube's top plane.
    """
    if image_size != calibration.image_size:
        raise GeometryUnavailable("Camera resolution differs from intrinsic calibration")
    if not (np.isfinite(u) and np.isfinite(v)
            and 0 <= u < image_size[0] and 0 <= v < image_size[1]):
        raise GeometryUnavailable("Pixel is outside the calibrated image")
    undistorted = cv2.undistortPoints(
        np.asarray([[[u, v]]], dtype=np.float64),
        calibration.camera_matrix, calibration.distortion)[0, 0]
    ray_camera = np.array([undistorted[0], undistorted[1], 1.0], dtype=np.float64)
    transform = base_to_camera(joints, calibration)
    origin = transform[:3, 3]
    direction = transform[:3, :3] @ ray_camera
    if abs(direction[2]) < 1e-9:
        raise GeometryUnavailable("Camera ray is parallel to the cube-top plane")
    plane_z_internal = physical_to_internal(XYZ(0.0, 0.0, calibration.cube_top_z_mm)).z
    distance = (plane_z_internal - origin[2]) / direction[2]
    if not np.isfinite(distance) or distance <= 0:
        raise GeometryUnavailable("Cube-top plane is behind the camera")
    point = origin + distance * direction
    return internal_to_physical(XYZ(*map(float, point)))
