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
        # ID14=0 is the physical zero. HOME_ID14 is a separate user pose.
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

    phi2 = config.ALPHA2_0 - t2
    phi3 = math.radians(90.0) - t2 - t3
    phi4 = phi3 - t4

    radial = (
        config.L2 * math.cos(phi2)
        + config.L3_X * math.cos(phi3)
        + config.L4_X * math.cos(phi4)
    )
    z = (
        config.Z0
        + config.L2 * math.sin(phi2)
        + config.L3_X * math.sin(phi3)
        + config.L4_X * math.sin(phi4)
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


def _candidate_joints_for_target(
    x: float,
    y: float,
    z: float,
    preferred_theta4: float | None = None,
) -> list[JointAngles]:
    theta1 = math.degrees(math.atan2(y, x))
    radial = math.hypot(x, y)
    z_plane = z - config.Z0

    # Build prioritized list of theta4 candidates
    t4_candidates: list[float] = []
    if preferred_theta4 is not None:
        t4_candidates.append(normalize_angle(preferred_theta4))
    for preset in (90.0, 0.0, 45.0, -45.0, 30.0, -30.0, 60.0, -60.0):
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
        A = config.L3_X + config.L4_X * math.cos(t4)
        B = config.L4_X * math.sin(t4)
        R_eff = math.hypot(A, B)
        psi_eff = math.atan2(B, A)

        D = math.hypot(radial, z_plane)
        if D > config.L2 + R_eff + 1e-3 or D < abs(config.L2 - R_eff) - 1e-3:
            continue

        cos_alpha = (config.L2 * config.L2 + D * D - R_eff * R_eff) / (2.0 * config.L2 * D)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)
        gamma = math.atan2(z_plane, radial)

        for sign in (1.0, -1.0):
            phi2 = gamma + sign * alpha
            t2 = config.ALPHA2_0 - phi2
            t2_deg = math.degrees(t2)

            r_elbow = config.L2 * math.cos(phi2)
            z_elbow = config.L2 * math.sin(phi2)

            dr = radial - r_elbow
            dz = z_plane - z_elbow
            phi_eff = math.atan2(dz, dr)
            phi3 = phi_eff + psi_eff
            t3 = math.radians(90.0) - t2 - phi3
            t3_deg = normalize_angle(math.degrees(t3))

            candidates.append(JointAngles(theta1, t2_deg, t3_deg, theta4))

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
        raise KinematicsError(f"Target XYZ=({x:.1f}, {y:.1f}, {z:.1f}) is outside robot reachable workspace")

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
        if error <= config.IK_POSITION_TOLERANCE_MM:
            valid.append(IKResult(joints, motors, xyz_check, dx, dy, dz, error))

    if not valid:
        raise KinematicsError(
            f"No valid IK solution within joint limits for target XYZ=({x:.1f}, {y:.1f}, {z:.1f})"
        )

    # Sort solutions prioritizing minimal joint motion from reference posture
    valid.sort(
        key=lambda result: (
            abs(normalize_angle(result.joint_angles.theta1 - reference.theta1)) * 1.0
            + abs(normalize_angle(result.joint_angles.theta2 - reference.theta2)) * 1.5
            + abs(normalize_angle(result.joint_angles.theta3 - reference.theta3)) * 1.2
            + abs(normalize_angle(result.joint_angles.theta4 - reference.theta4)) * 0.8,
            result.position_error,
        )
    )
    return valid[0]


def validate_ik_solution(result: IKResult) -> None:
    if result.position_error > config.IK_POSITION_TOLERANCE_MM:
        raise KinematicsError("IK solution exceeds FK validation tolerance")
    if not _joint_limits_ok(result.joint_angles):
        raise KinematicsError("IK solution violates joint limits")
    validate_motor_angles(result.motor_angles)
