"""Provisional rear-of-ID11 measurement frame; existing robot FK/IK stays axis-centred."""

from __future__ import annotations

import math

import config
from kinematics import XYZ


def rear_to_axis_x(x_rear_mm: float) -> float:
    """Convert a ruler X measured from ID11's rear face to the ID11-axis frame."""
    if not math.isfinite(x_rear_mm):
        raise ValueError("Rear-frame X must be finite")
    return x_rear_mm - config.REAR_TO_AXIS_X_MM


def axis_to_rear_x(x_axis_mm: float) -> float:
    if not math.isfinite(x_axis_mm):
        raise ValueError("Axis-frame X must be finite")
    return x_axis_mm + config.REAR_TO_AXIS_X_MM


def rear_to_axis_xyz(target: XYZ) -> XYZ:
    return XYZ(rear_to_axis_x(target.x), target.y, target.z)


def axis_to_rear_xyz(target: XYZ) -> XYZ:
    return XYZ(axis_to_rear_x(target.x), target.y, target.z)


def rear_to_internal_transform() -> dict[str, float]:
    """Compose rear->axis with the existing physical-axis->internal map.

    The legacy yaw-only camera transfer expects this mapping as a dictionary.
    It must rotate about ID11's axis, not about the new rear-face origin.
    """
    transform = dict(config.PHYSICAL_TO_INTERNAL)
    transform["x_offset_mm"] = (
        float(transform["x_offset_mm"])
        - float(transform["x_scale"]) * config.REAR_TO_AXIS_X_MM
    )
    return transform
