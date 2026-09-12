"""Calibration, FK and deterministic fixed-pitch IK for OpenMANIPULATOR-X.

All lengths are millimetres and all public angles are degrees.  DYNAMIXEL RAW
positions never enter the kinematic equations: conversion is performed only by
``raw_to_joint`` and ``joint_to_raw``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable


RAW_COUNTS_PER_REV = 4096
RAW_DEGREES_PER_COUNT = 360.0 / RAW_COUNTS_PER_REV

# Official ROBOTIS model geometry, converted from metres to millimetres.
BASE_X_MM = 12.0
SHOULDER_Z_MM = 17.0 + 59.5
LINK2_X_MM = 24.0
LINK2_Z_MM = 128.0
LINK2_MM = math.hypot(LINK2_X_MM, LINK2_Z_MM)
LINK2_ZERO_PITCH_RAD = math.atan2(LINK2_Z_MM, LINK2_X_MM)
LINK3_MM = 124.0
TCP_MM = 126.0


class CalibrationError(ValueError):
    pass


class IKReason(str, Enum):
    OK = "OK"
    OUTSIDE_WORKSPACE = "OUTSIDE_WORKSPACE"
    JOINT_LIMIT = "JOINT_LIMIT"
    SINGULARITY = "SINGULARITY"
    NO_SOLUTION = "NO_SOLUTION"
    INVALID_ORIENTATION = "INVALID_ORIENTATION"


@dataclass(frozen=True)
class JointCalibration:
    id: int
    raw_min: int
    raw_home: int | None
    raw_max: int
    joint_min_deg: float | None
    joint_home_deg: float | None
    joint_max_deg: float | None
    direction: int | None
    safe_raw_min: int
    safe_raw_max: int
    provisional: bool = False

    @property
    def complete(self) -> bool:
        return (
            self.raw_home is not None
            and self.joint_min_deg is not None
            and self.joint_home_deg is not None
            and self.joint_max_deg is not None
            and self.direction in (-1, 1)
        )


# J2-J4 mathematical endpoint angles, HOME and direction are intentionally None.
# Replace them only after the measurement sequence in README.md is completed.
CALIBRATIONS: dict[int, JointCalibration] = {
    11: JointCalibration(11, 915, 1941, 2961, -90.0, 0.0, 90.0, 1, 935, 2941),
    12: JointCalibration(12, 2040, None, 3000, None, None, None, None, 2060, 2980),
    13: JointCalibration(13, 3570, None, 4062, None, None, None, None, 3590, 4042, True),
    14: JointCalibration(14, 71, None, 995, None, None, None, None, 91, 975),
}

GRIPPER_OPEN_RAW = 1211
GRIPPER_CLOSED_RAW = 2672

# Official nominal model limits. Physical automatic motion additionally requires
# completed measured calibration and the smaller calibrated safe range.
NOMINAL_JOINT_LIMITS_DEG = {
    11: (-180.0, 180.0),
    12: (math.degrees(-2.05), 90.0),
    13: (-90.0, math.degrees(1.53)),
    14: (math.degrees(-1.8), math.degrees(2.0)),
}


@dataclass(frozen=True)
class Joints:
    q1: float
    q2: float
    q3: float
    q4: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return self.q1, self.q2, self.q3, self.q4


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    tool_pitch_deg: float


@dataclass(frozen=True)
class IKResult:
    success: bool
    reason: IKReason
    message: str
    joints: Joints | None = None
    reconstructed: Pose | None = None
    dx: float | None = None
    dy: float | None = None
    dz: float | None = None
    error_mm: float | None = None


def raw_to_motor_degrees(raw: int) -> float:
    return raw * RAW_DEGREES_PER_COUNT


def _calibration(motor_id: int) -> JointCalibration:
    try:
        cal = CALIBRATIONS[motor_id]
    except KeyError as exc:
        raise CalibrationError(f"ID{motor_id} is not an arm joint") from exc
    if not cal.complete:
        raise CalibrationError(f"ID{motor_id} calibration is pending; capture HOME, direction and endpoint joint angles")
    return cal


def is_raw_position_valid(motor_id: int, raw: int, *, automatic: bool = False) -> bool:
    cal = CALIBRATIONS[motor_id]
    low, high = (cal.safe_raw_min, cal.safe_raw_max) if automatic else (cal.raw_min, cal.raw_max)
    return low <= raw <= high


def is_joint_valid(motor_id: int, joint_deg: float) -> bool:
    cal = _calibration(motor_id)
    low = min(cal.joint_min_deg, cal.joint_max_deg)  # type: ignore[arg-type]
    high = max(cal.joint_min_deg, cal.joint_max_deg)  # type: ignore[arg-type]
    return low <= joint_deg <= high


def _lerp(value: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if x0 == x1:
        raise CalibrationError("Calibration segment has zero RAW span")
    return y0 + (value - x0) * (y1 - y0) / (x1 - x0)


def raw_to_joint(motor_id: int, raw: int) -> float:
    cal = _calibration(motor_id)
    if not is_raw_position_valid(motor_id, raw):
        raise CalibrationError(f"ID{motor_id} RAW {raw} is outside measured range [{cal.raw_min}, {cal.raw_max}]")
    assert cal.raw_home is not None and cal.joint_home_deg is not None
    assert cal.joint_min_deg is not None and cal.joint_max_deg is not None
    if raw <= cal.raw_home:
        return _lerp(raw, cal.raw_min, cal.raw_home, cal.joint_min_deg, cal.joint_home_deg)
    return _lerp(raw, cal.raw_home, cal.raw_max, cal.joint_home_deg, cal.joint_max_deg)


def joint_to_raw(motor_id: int, joint_deg: float, *, automatic: bool = False) -> int:
    cal = _calibration(motor_id)
    if not is_joint_valid(motor_id, joint_deg):
        raise CalibrationError(f"ID{motor_id} joint {joint_deg:.3f} deg is outside calibrated range")
    assert cal.raw_home is not None and cal.joint_home_deg is not None
    assert cal.joint_min_deg is not None and cal.joint_max_deg is not None
    if (cal.joint_min_deg <= cal.joint_home_deg and joint_deg <= cal.joint_home_deg) or (
        cal.joint_min_deg > cal.joint_home_deg and joint_deg >= cal.joint_home_deg
    ):
        raw = _lerp(joint_deg, cal.joint_min_deg, cal.joint_home_deg, cal.raw_min, cal.raw_home)
    else:
        raw = _lerp(joint_deg, cal.joint_home_deg, cal.joint_max_deg, cal.raw_home, cal.raw_max)
    result = int(round(raw))
    if not is_raw_position_valid(motor_id, result, automatic=automatic):
        kind = "automatic safe" if automatic else "measured"
        raise CalibrationError(f"ID{motor_id} RAW {result} is outside {kind} range")
    return result


def clamp_joint_to_safe_range(motor_id: int, joint_deg: float) -> float:
    """Explicit utility for UI previews; never call this silently before motion."""
    cal = _calibration(motor_id)
    safe_a = raw_to_joint(motor_id, cal.safe_raw_min)
    safe_b = raw_to_joint(motor_id, cal.safe_raw_max)
    return min(max(joint_deg, min(safe_a, safe_b)), max(safe_a, safe_b))


def calibration_ready() -> bool:
    return all(cal.complete and not cal.provisional for cal in CALIBRATIONS.values())


def joints_to_raw(joints: Joints, *, automatic: bool = True) -> dict[int, int]:
    return {motor_id: joint_to_raw(motor_id, q, automatic=automatic) for motor_id, q in zip((11, 12, 13, 14), joints.as_tuple(), strict=True)}


def raw_to_joints(raw: dict[int, int]) -> Joints:
    return Joints(*(raw_to_joint(i, raw[i]) for i in (11, 12, 13, 14)))


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def forward_kinematics(joints: Joints) -> Pose:
    if not _finite(joints.as_tuple()):
        raise ValueError("Joint angles must be finite")
    q1, q2, q3, q4 = map(math.radians, joints.as_tuple())
    # Rotation about +Y makes positive q pitch a +X vector toward -Z.
    a2 = LINK2_ZERO_PITCH_RAD - q2
    a3 = -(q2 + q3)
    pitch = -(q2 + q3 + q4)
    radial = BASE_X_MM + LINK2_MM * math.cos(a2) + LINK3_MM * math.cos(a3) + TCP_MM * math.cos(pitch)
    z = SHOULDER_Z_MM + LINK2_MM * math.sin(a2) + LINK3_MM * math.sin(a3) + TCP_MM * math.sin(pitch)
    return Pose(radial * math.cos(q1), radial * math.sin(q1), z, math.degrees(pitch))


def _nominal_limits_ok(joints: Joints) -> bool:
    return all(NOMINAL_JOINT_LIMITS_DEG[i][0] - 1e-9 <= q <= NOMINAL_JOINT_LIMITS_DEG[i][1] + 1e-9 for i, q in zip((11, 12, 13, 14), joints.as_tuple(), strict=True))


def inverse_kinematics(x: float, y: float, z: float, tool_pitch_deg: float, *, reference: Joints | None = None, tolerance_mm: float = 1e-6) -> IKResult:
    if not _finite((x, y, z, tool_pitch_deg)) or not -180.0 <= tool_pitch_deg <= 180.0:
        return IKResult(False, IKReason.INVALID_ORIENTATION, "Target and pitch must be finite; pitch must be in [-180, 180]")
    q1 = math.atan2(y, x)
    radial = math.hypot(x, y)
    pitch = math.radians(tool_pitch_deg)
    wrist_x = radial - BASE_X_MM - TCP_MM * math.cos(pitch)
    wrist_z = z - SHOULDER_Z_MM - TCP_MM * math.sin(pitch)
    d2 = wrist_x * wrist_x + wrist_z * wrist_z
    if d2 < 1e-12:
        return IKResult(False, IKReason.SINGULARITY, "Wrist target coincides with shoulder axis")
    c = (d2 - LINK2_MM**2 - LINK3_MM**2) / (2.0 * LINK2_MM * LINK3_MM)
    if c < -1.0 - 1e-9 or c > 1.0 + 1e-9:
        return IKResult(False, IKReason.OUTSIDE_WORKSPACE, "Fixed-pitch wrist target is outside the two-link workspace")
    c = min(1.0, max(-1.0, c))
    candidates: list[tuple[float, Joints, Pose, float, float, float, float]] = []
    for elbow_delta in (math.acos(c), -math.acos(c)):
        a2 = math.atan2(wrist_z, wrist_x) - math.atan2(LINK3_MM * math.sin(elbow_delta), LINK2_MM + LINK3_MM * math.cos(elbow_delta))
        a3 = a2 + elbow_delta
        q2 = LINK2_ZERO_PITCH_RAD - a2
        q3 = -a3 - q2
        q4 = -pitch - q2 - q3
        joints = Joints(*map(math.degrees, (q1, q2, q3, q4)))
        if not _nominal_limits_ok(joints):
            continue
        pose = forward_kinematics(joints)
        dx, dy, dz = pose.x - x, pose.y - y, pose.z - z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        if error <= tolerance_mm:
            score = sum(abs(a - b) for a, b in zip(joints.as_tuple(), reference.as_tuple(), strict=True)) if reference else sum(abs(a) for a in joints.as_tuple())
            candidates.append((score, joints, pose, dx, dy, dz, error))
    if not candidates:
        return IKResult(False, IKReason.JOINT_LIMIT, "Geometric solutions exist but violate nominal joint limits")
    _, joints, pose, dx, dy, dz, error = min(candidates, key=lambda item: item[0])
    return IKResult(True, IKReason.OK, "IK reconstructed by FK", joints, pose, dx, dy, dz, error)

