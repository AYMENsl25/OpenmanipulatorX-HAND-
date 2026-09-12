"""Offline planner for the named ground-contact point experiment.

This module deliberately has no serial-controller dependency and never sends a
robot command.  It turns physical experiment points into independently audited
approach, touch, and retract IK plans that a higher-level preview/logging layer
can inspect before any future hardware execution is considered.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping

import config
from experiment_frame import (
    PhysicalIKResult,
    experiment_points,
    inverse_kinematics_physical,
    physical_to_internal,
)
from kinematics import JointAngles, KinematicsError, MotorAngles, XYZ


READY = "READY"
SOFT_LIMIT = "SOFT LIMIT"
GROUND_PROTECTION = "GROUND PROTECTION"
UNREACHABLE_GEOMETRY = "UNREACHABLE / GEOMETRY"
UNREACHABLE_JOINT_LIMIT = "UNREACHABLE / JOINT LIMIT"
MEASUREMENT_OK = "OK"
MEASUREMENT_WARNING = "WARNING"
MEASUREMENT_STOP = "STOP"

STATUSES = (
    READY,
    SOFT_LIMIT,
    GROUND_PROTECTION,
    UNREACHABLE_GEOMETRY,
    UNREACHABLE_JOINT_LIMIT,
)


def tcp_error_severity(
    error_mm: float,
    *,
    warning_mm: float | None = None,
    stop_mm: float | None = None,
) -> str:
    """Classify measured Cartesian error using configurable two-level gates."""
    error = float(error_mm)
    warning = float(config.TCP_WARNING_ERROR_MM if warning_mm is None else warning_mm)
    stop = float(config.MAXIMUM_TCP_ERROR_MM if stop_mm is None else stop_mm)
    if not all(math.isfinite(value) for value in (error, warning, stop)):
        raise ValueError("TCP error thresholds must be finite")
    if error < 0.0 or warning < 0.0 or stop <= warning:
        raise ValueError("Require 0 <= error, 0 <= warning < stop")
    if error > stop:
        return MEASUREMENT_STOP
    if error > warning:
        return MEASUREMENT_WARNING
    return MEASUREMENT_OK


@dataclass(frozen=True)
class TargetPlan:
    """Result of planning one physical Cartesian target."""

    phase: str
    physical_target: XYZ
    internal_target: XYZ
    status: str
    message: str
    reference_joint_angles: JointAngles | None = None
    joint_angles: JointAngles | None = None
    motor_angles: MotorAngles | None = None
    internal_fk_check: XYZ | None = None
    physical_fk_check: XYZ | None = None
    fk_error_mm: float | None = None

    @property
    def ready(self) -> bool:
        return self.status == READY


@dataclass(frozen=True)
class PointPlan:
    """Safe three-phase plan for one named experiment point."""

    name: str
    physical_target: XYZ
    internal_target: XYZ
    approach_target: XYZ
    touch_target: XYZ
    retract_target: XYZ
    approach: TargetPlan
    touch: TargetPlan
    retract: TargetPlan
    status: str
    message: str

    @property
    def motion_ready(self) -> bool:
        return self.status == READY and all(
            phase.ready for phase in (self.approach, self.touch, self.retract)
        )

    @property
    def selected_joints(self) -> JointAngles | None:
        """The selected contact-pose joints, if the touch target is reachable."""
        return self.touch.joint_angles

    @property
    def selected_motors(self) -> MotorAngles | None:
        """The selected contact-pose motors, if the touch target is reachable."""
        return self.touch.motor_angles

    @property
    def touch_fk_error_mm(self) -> float | None:
        return self.touch.fk_error_mm


@dataclass(frozen=True)
class ExperimentPlan:
    """Ordered offline result for a collection of named points."""

    points: tuple[PointPlan, ...]
    approach_clearance_mm: float
    retract_clearance_mm: float
    starting_reference: JointAngles | None
    final_reference: JointAngles | None

    @property
    def ready_count(self) -> int:
        return sum(point.motion_ready for point in self.points)

    @property
    def blocked_count(self) -> int:
        return len(self.points) - self.ready_count


def _configured_clearance(attribute: str, fallback: float) -> float:
    value = float(getattr(config, attribute, fallback))
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{attribute} must be a finite value greater than or equal to zero")
    return value


def _classify_error(error: KinematicsError) -> str:
    message = str(error).upper()
    if "SOFT CARTESIAN LIMIT" in message or "SOFT LIMIT" in message:
        return SOFT_LIMIT
    if "GROUND PROTECTION" in message:
        return GROUND_PROTECTION
    if "UNREACHABLE / JOINT LIMIT" in message or "JOINT LIMIT" in message:
        return UNREACHABLE_JOINT_LIMIT
    if "UNREACHABLE / GEOMETRY" in message or "UNREACHABLE" in message:
        return UNREACHABLE_GEOMETRY
    # Unknown kinematic failures must never be promoted to READY.  Geometry is
    # the conservative category for an unsolved Cartesian target.
    return UNREACHABLE_GEOMETRY


def _solve_target(
    phase: str,
    physical_target: XYZ,
    reference: JointAngles | None,
) -> TargetPlan:
    internal_target = physical_to_internal(physical_target)
    try:
        solved: PhysicalIKResult = inverse_kinematics_physical(
            physical_target.x,
            physical_target.y,
            physical_target.z,
            reference=reference,
        )
    except KinematicsError as error:
        return TargetPlan(
            phase=phase,
            physical_target=physical_target,
            internal_target=internal_target,
            status=_classify_error(error),
            message=str(error),
            reference_joint_angles=reference,
        )

    return TargetPlan(
        phase=phase,
        physical_target=solved.physical_target,
        internal_target=solved.internal_target,
        status=READY,
        message="IK, joint, motor, and FK checks passed",
        reference_joint_angles=reference,
        joint_angles=solved.ik.joint_angles,
        motor_angles=solved.ik.motor_angles,
        internal_fk_check=solved.ik.xyz_check,
        physical_fk_check=solved.physical_fk_check,
        fk_error_mm=solved.ik.position_error,
    )


def _first_blocked_phase(phases: Iterable[TargetPlan]) -> TargetPlan | None:
    return next((phase for phase in phases if not phase.ready), None)


def plan_point_experiment(
    points: Mapping[str, XYZ] | None = None,
    *,
    names: Iterable[str] | None = None,
    starting_reference: JointAngles | None = None,
    approach_clearance_mm: float | None = None,
    retract_clearance_mm: float | None = None,
) -> ExperimentPlan:
    """Plan named points in order without communicating with robot hardware.

    The last reachable phase becomes the IK reference for the next phase and
    then the next point.  This preserves the existing nearest-solution behavior
    while still producing diagnostic entries for every blocked phase.
    """

    available = dict(experiment_points() if points is None else points)
    selected_names = list(available if names is None else names)
    missing = [name for name in selected_names if name not in available]
    if missing:
        raise KeyError(f"Unknown experiment point(s): {', '.join(missing)}")

    approach_clearance = (
        _configured_clearance("APPROACH_CLEARANCE_MM", 50.0)
        if approach_clearance_mm is None
        else float(approach_clearance_mm)
    )
    retract_clearance = (
        _configured_clearance("RETRACT_CLEARANCE_MM", 50.0)
        if retract_clearance_mm is None
        else float(retract_clearance_mm)
    )
    for label, clearance in (
        ("approach_clearance_mm", approach_clearance),
        ("retract_clearance_mm", retract_clearance),
    ):
        if not math.isfinite(clearance) or clearance < 0.0:
            raise ValueError(f"{label} must be a finite value greater than or equal to zero")

    reference = starting_reference
    planned_points: list[PointPlan] = []
    for name in selected_names:
        touch_target = available[name]
        approach_target = XYZ(
            touch_target.x,
            touch_target.y,
            touch_target.z + approach_clearance,
        )
        retract_target = XYZ(
            touch_target.x,
            touch_target.y,
            touch_target.z + retract_clearance,
        )

        approach = _solve_target("APPROACH", approach_target, reference)
        if approach.ready:
            reference = approach.joint_angles

        touch = _solve_target("TOUCH", touch_target, reference)
        if touch.ready:
            reference = touch.joint_angles

        retract = _solve_target("RETRACT", retract_target, reference)
        if retract.ready:
            reference = retract.joint_angles

        phases = (approach, touch, retract)
        blocked = _first_blocked_phase(phases)
        status = READY if blocked is None else blocked.status
        message = (
            "Approach, touch, and retract checks passed"
            if blocked is None
            else f"{blocked.phase}: {blocked.message}"
        )
        planned_points.append(
            PointPlan(
                name=name,
                physical_target=touch_target,
                internal_target=physical_to_internal(touch_target),
                approach_target=approach_target,
                touch_target=touch_target,
                retract_target=retract_target,
                approach=approach,
                touch=touch,
                retract=retract,
                status=status,
                message=message,
            )
        )

    return ExperimentPlan(
        points=tuple(planned_points),
        approach_clearance_mm=approach_clearance,
        retract_clearance_mm=retract_clearance,
        starting_reference=starting_reference,
        final_reference=reference,
    )


def plan_named_experiment_points(
    *,
    starting_reference: JointAngles | None = None,
) -> ExperimentPlan:
    """Convenience entry point for configured P01 through P07."""
    return plan_point_experiment(starting_reference=starting_reference)
