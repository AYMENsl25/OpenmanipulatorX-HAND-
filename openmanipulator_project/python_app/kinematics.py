"""Forward and inverse kinematics for the project OpenMANIPULATOR-X model."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import config


class KinematicsError(Exception):
    pass


@dataclass(frozen=True)
class MotorAngles:
    id11: float
    id12: float
    id13: float
    id14: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return self.id11, self.id12, self.id13, self.id14


@dataclass(frozen=True)
class JointAngles:
    theta1: float
    theta2: float
    theta3: float
    theta4: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return self.theta1, self.theta2, self.theta3, self.theta4


@dataclass(frozen=True)
class XYZ:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class IKResult:
    joint_angles: JointAngles
    motor_angles: MotorAngles
    xyz_check: XYZ
    dx: float
    dy: float
    dz: float
    position_error: float


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def normalize_motor_angle(angle: float) -> float:
    """Normalize an encoder-display angle to [0, 360)."""
    return angle % 360.0


def raw_to_degrees(raw: int | float) -> float:
    """Convert DYNAMIXEL raw encoder counts to 0..360 degrees."""
    return normalize_motor_angle((float(raw) / config.RAW_COUNTS_PER_REV) * 360.0)


def degrees_to_raw(degrees: float) -> int:
    """Convert 0..360 degrees to DYNAMIXEL raw encoder counts."""
    return int(round(normalize_motor_angle(degrees) / 360.0 * config.RAW_COUNTS_PER_REV))


def _assert_finite(values: Iterable[float], label: str) -> None:
    for value in values:
        if not math.isfinite(value):
            raise KinematicsError(f"{label} contains NaN or infinite value")


def motor_to_fk_angles(motors: MotorAngles) -> JointAngles:
    _assert_finite(motors.as_tuple(), "Motor angles")
    return JointAngles(
        theta1=normalize_angle(motors.id11 - config.ID11_CENTER),
        theta2=normalize_angle(motors.id12 - config.ID12_ZERO),
        theta3=normalize_angle(motors.id13 - config.ID13_ZERO),
        theta4=normalize_angle(motors.id14 - config.ID14_ZERO),
    )


def fk_to_motor_angles(joints: JointAngles) -> MotorAngles:
    _assert_finite(joints.as_tuple(), "FK angles")
    return MotorAngles(
        id11=normalize_motor_angle(joints.theta1 + config.ID11_CENTER),
        id12=normalize_motor_angle(joints.theta2 + config.ID12_ZERO),
        id13=normalize_motor_angle(joints.theta3 + config.ID13_ZERO),
        id14=normalize_motor_angle(joints.theta4 + config.ID14_ZERO),
    )


def forward_kinematics_from_joints(joints: JointAngles) -> XYZ:
    _assert_finite(joints.as_tuple(), "FK angles")
    t1 = math.radians(joints.theta1)
    t2 = math.radians(joints.theta2)
    t3 = math.radians(joints.theta3)
    t4 = math.radians(joints.theta4)

    a2 = t2
    a3 = t2 + t3
    a4 = t2 + t3 + t4

    radial = (
        config.L2_X * math.cos(a2)
        + config.L3_X * math.cos(a3)
        + config.L4_X * math.cos(a4)
    )
    z = (
        config.Z_BASE
        + config.L1_Z
        + config.L2_Z * math.sin(a2)
        + config.L3_X * math.sin(a3)
        + config.L4_X * math.sin(a4)
    )
    return XYZ(
        x=radial * math.cos(t1),
        y=radial * math.sin(t1),
        z=z,
    )


def forward_kinematics(motors: MotorAngles) -> XYZ:
    return forward_kinematics_from_joints(motor_to_fk_angles(motors))


def _joint_limits_ok(joints: JointAngles) -> bool:
    return (
        config.FK_JOINT_LIMITS["theta1"].contains(joints.theta1)
        and config.FK_JOINT_LIMITS["theta2"].contains(joints.theta2)
        and config.FK_JOINT_LIMITS["theta3"].contains(joints.theta3)
        and config.FK_JOINT_LIMITS["theta4"].contains(joints.theta4)
    )


def validate_motor_angles(motors: MotorAngles) -> None:
    _assert_finite(motors.as_tuple(), "Motor angles")
    for motor_id, value in zip((11, 12, 13, 14), motors.as_tuple(), strict=True):
        if not config.MOTOR_ANGLE_LIMITS[motor_id].contains(normalize_motor_angle(value)):
            raise KinematicsError(f"ID{motor_id} motor angle is out of range")


def _bisect_roots(function, low: float, high: float, step: float = 0.5) -> list[float]:
    roots: list[float] = []
    x0 = low
    y0 = function(x0)
    x = low + step
    while x <= high + 1e-9:
        y = function(x)
        if abs(y0) < 1e-8:
            roots.append(x0)
        elif y0 * y < 0.0:
            a, b = x0, x
            fa = y0
            for _ in range(60):
                mid = (a + b) / 2.0
                fm = function(mid)
                if abs(fm) < 1e-10:
                    a = b = mid
                    break
                if fa * fm <= 0.0:
                    b = mid
                else:
                    a = mid
                    fa = fm
            roots.append((a + b) / 2.0)
        x0, y0 = x, y
        x += step

    unique: list[float] = []
    for root in roots:
        if all(abs(root - existing) > 0.05 for existing in unique):
            unique.append(root)
    return unique


def _candidate_joints_for_target(
    x: float,
    y: float,
    z: float,
    preferred_theta4: float | None = None,
) -> list[JointAngles]:
    theta1 = math.degrees(math.atan2(y, x))
    radial = math.hypot(x, y)
    z_plane = z - config.Z_BASE - config.L1_Z

    # Build prioritized list of theta4 candidates
    t4_candidates: list[float] = []
    if preferred_theta4 is not None:
        t4_candidates.append(normalize_angle(preferred_theta4))
    if config.DEFAULT_THETA4 not in t4_candidates:
        t4_candidates.append(config.DEFAULT_THETA4)
    for preset in (0.0, 45.0, -45.0, 30.0, -30.0, 60.0, -60.0):
        if preset not in t4_candidates:
            t4_candidates.append(preset)

    # Add dense sweep across theta4 joint limits in 5-degree increments
    t4_min = int(math.floor(config.FK_JOINT_LIMITS["theta4"].minimum))
    t4_max = int(math.ceil(config.FK_JOINT_LIMITS["theta4"].maximum))
    for deg in range(t4_min, t4_max + 1, 5):
        val = float(deg)
        if not any(abs(val - existing) < 1e-3 for existing in t4_candidates):
            t4_candidates.append(val)

    candidates: list[JointAngles] = []
    for theta4 in t4_candidates:
        t4 = math.radians(theta4)
        effective_x = config.L3_X + config.L4_X * math.cos(t4)
        effective_z = config.L4_X * math.sin(t4)
        effective_len = math.hypot(effective_x, effective_z)
        effective_phase = math.atan2(effective_z, effective_x)

        def shoulder_constraint(theta2_deg: float) -> float:
            t2 = math.radians(theta2_deg)
            dx = radial - config.L2_X * math.cos(t2)
            dz = z_plane - config.L2_Z * math.sin(t2)
            return dx * dx + dz * dz - effective_len * effective_len

        roots = _bisect_roots(
            shoulder_constraint,
            config.FK_JOINT_LIMITS["theta2"].minimum,
            config.FK_JOINT_LIMITS["theta2"].maximum,
            step=1.0,
        )
        for theta2 in roots:
            t2 = math.radians(theta2)
            dx = radial - config.L2_X * math.cos(t2)
            dz = z_plane - config.L2_Z * math.sin(t2)
            if math.hypot(dx, dz) < 1e-9:
                continue
            a3 = math.atan2(dz, dx) - effective_phase
            theta3 = normalize_angle(math.degrees(a3 - t2))
            candidates.append(JointAngles(theta1, theta2, theta3, theta4))

    return candidates


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    reference: JointAngles | None = None,
) -> IKResult:
    _assert_finite((x, y, z), "Target XYZ")
    if reference is None:
        reference = motor_to_fk_angles(
            MotorAngles(config.HOME_ID11, config.HOME_ID12, config.HOME_ID13, config.HOME_ID14)
        )

    candidates = _candidate_joints_for_target(x, y, z, preferred_theta4=reference.theta4)
    if not candidates:
        raise KinematicsError("Target XYZ is unreachable")

    valid: list[IKResult] = []
    for joints in candidates:
        if not _joint_limits_ok(joints):
            continue
        motors = fk_to_motor_angles(joints)
        try:
            validate_motor_angles(motors)
        except KinematicsError:
            continue
        xyz_check = forward_kinematics_from_joints(joints)
        dx = xyz_check.x - x
        dy = xyz_check.y - y
        dz = xyz_check.z - z
        error = math.sqrt(dx * dx + dy * dy + dz * dz)
        valid.append(IKResult(joints, motors, xyz_check, dx, dy, dz, error))

    if not valid:
        raise KinematicsError(
            f"No valid IK solution within joint limits for target XYZ=({x:.1f}, {y:.1f}, {z:.1f})"
        )

    # Sort solutions prioritizing low position error and minimal joint motion from reference
    valid.sort(
        key=lambda result: (
            result.position_error > config.IK_POSITION_TOLERANCE_MM,
            abs(normalize_angle(result.joint_angles.theta1 - reference.theta1)) * 1.0
            + abs(normalize_angle(result.joint_angles.theta2 - reference.theta2)) * 1.5
            + abs(normalize_angle(result.joint_angles.theta3 - reference.theta3)) * 1.2
            + abs(normalize_angle(result.joint_angles.theta4 - reference.theta4)) * 0.8,
            result.position_error,
        )
    )
    best = valid[0]
    if best.position_error > config.IK_POSITION_TOLERANCE_MM:
        raise KinematicsError(
            f"IK/FK validation error {best.position_error:.3f} mm exceeds "
            f"{config.IK_POSITION_TOLERANCE_MM:.3f} mm tolerance"
        )
    return best


def validate_ik_solution(result: IKResult) -> None:
    if result.position_error > config.IK_POSITION_TOLERANCE_MM:
        raise KinematicsError("IK solution exceeds FK validation tolerance")
    if not _joint_limits_ok(result.joint_angles):
        raise KinematicsError("IK solution violates joint limits")
    validate_motor_angles(result.motor_angles)
