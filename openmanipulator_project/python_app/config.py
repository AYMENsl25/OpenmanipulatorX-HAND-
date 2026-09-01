"""Project configuration for the OpenMANIPULATOR-X XYZ controller.

These values are experimental project calibration values, not universal
ROBOTIS zero references. Keep all calibration edits in this file.
"""

from __future__ import annotations

from dataclasses import dataclass


Z_BASE = 17.0
L1_Z = 59.5
L2_X = 24.0
L2_Z = 128.0
L3_X = 124.0
L4_X = 126.0

ID11_CENTER = 351.0
ID12_ZERO = 0.0
ID13_ZERO = 0.0
ID14_ZERO = 0.0

HOME_ID11 = 351.0
HOME_ID12 = 1.0
HOME_ID13 = 1.0
HOME_ID14 = 90.0

DEFAULT_THETA4 = 90.0
IK_POSITION_TOLERANCE_MM = 2.0

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT_SECONDS = 2.0
MOVE_TIMEOUT_SECONDS = 30.0

# OpenCR expects degrees in the same 0..360 encoder-angle convention it returns.
RAW_COUNTS_PER_REV = 4096
RAW_ZERO_DEGREES = 0.0


@dataclass(frozen=True)
class JointLimit:
    minimum: float
    maximum: float

    def contains(self, value: float) -> bool:
        return self.minimum <= value <= self.maximum


# FK joint limits are intentionally conservative software limits. Adjust after
# measured robot-limit validation.
FK_JOINT_LIMITS = {
    "theta1": JointLimit(-180.0, 180.0),
    "theta2": JointLimit(-130.0, 130.0),
    "theta3": JointLimit(-150.0, 150.0),
    "theta4": JointLimit(-180.0, 180.0),
}

# Motor encoder display/command limits. These cover one encoder revolution.
MOTOR_ANGLE_LIMITS = {
    11: JointLimit(0.0, 360.0),
    12: JointLimit(0.0, 360.0),
    13: JointLimit(0.0, 360.0),
    14: JointLimit(0.0, 360.0),
}
