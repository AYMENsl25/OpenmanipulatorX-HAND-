"""Physical experiment frame and guarded interface to the internal FK/IK."""

from __future__ import annotations

from dataclasses import dataclass

import config
from kinematics import (
    IKResult,
    JointAngles,
    KinematicsError,
    MotorAngles,
    XYZ,
    forward_kinematics,
    inverse_kinematics,
    validate_ik_solution,
)


class PhysicalTargetError(KinematicsError):
    """A physical target failed a pre-motion experiment-frame check."""


@dataclass(frozen=True)
class PhysicalIKResult:
    physical_target: XYZ
    internal_target: XYZ
    ik: IKResult
    physical_fk_check: XYZ


def _axis_transform(value: float, axis: str) -> float:
    transform = config.PHYSICAL_TO_INTERNAL
    return value * float(transform[f"{axis}_scale"]) + float(transform[f"{axis}_offset_mm"])


def _axis_inverse(value: float, axis: str) -> float:
    transform = config.PHYSICAL_TO_INTERNAL
    scale = float(transform[f"{axis}_scale"])
    if scale == 0.0:
        raise RuntimeError(f"Invalid {axis}-axis experiment transform scale 0")
    return (value - float(transform[f"{axis}_offset_mm"])) / scale


def physical_to_internal(target: XYZ) -> XYZ:
    """Convert ID11-centered physical coordinates to solver coordinates."""
    return XYZ(
        _axis_transform(target.x, "x"),
        _axis_transform(target.y, "y"),
        _axis_transform(target.z, "z"),
    )


def internal_to_physical(target: XYZ) -> XYZ:
    """Convert solver coordinates to ID11-centered physical coordinates."""
    return XYZ(
        _axis_inverse(target.x, "x"),
        _axis_inverse(target.y, "y"),
        _axis_inverse(target.z, "z"),
    )


def validate_physical_target(target: XYZ) -> None:
    """Apply finite, soft-envelope, and physical-ground checks."""
    from kinematics import validate_xyz_target

    validate_xyz_target(target)
    if target.z < config.GROUND_Z_MM:
        raise PhysicalTargetError(
            f"GROUND PROTECTION: physical Z={target.z:.3f} mm is below "
            f"ground Z={config.GROUND_Z_MM:.1f} mm"
        )


def forward_kinematics_physical(motors: MotorAngles) -> XYZ:
    return internal_to_physical(forward_kinematics(motors))


def inverse_kinematics_physical(
    x: float,
    y: float,
    z: float,
    reference: JointAngles | None = None,
) -> PhysicalIKResult:
    physical = XYZ(float(x), float(y), float(z))
    internal = physical_to_internal(physical)
    validate_physical_target(physical)
    result = inverse_kinematics(internal.x, internal.y, internal.z, reference=reference)
    validate_ik_solution(result)
    return PhysicalIKResult(physical, internal, result, internal_to_physical(result.xyz_check))


def experiment_points() -> dict[str, XYZ]:
    return {
        str(point["name"]): XYZ(float(point["x_mm"]), float(point["y_mm"]), float(point["z_mm"]))
        for point in config.EXPERIMENT_POINTS
    }
