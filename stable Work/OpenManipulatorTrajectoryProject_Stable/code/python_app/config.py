"""Project configuration for the OpenMANIPULATOR-X XYZ controller.

These values are experimental project calibration values, not universal
ROBOTIS zero references. Keep all calibration edits in this file.
"""

from __future__ import annotations

from dataclasses import dataclass


import math

Z_BASE = 17.0
L1_Z = 59.5
Z0 = Z_BASE + L1_Z  # 76.5 mm
BASE_X = 12.0

L2_X = 24.0
L2_Z = 128.0
L2 = math.hypot(L2_X, L2_Z)  # 130.23056 mm
ALPHA2_0 = math.atan2(L2_Z, L2_X)  # 79.380345 deg in radians

L3_X = 124.0
ROBOTIS_GRIPPER_FRAME_LENGTH = 126.0
GRIPPER_TIP_EXTENSION = 39.7
# The experiment tracks the working gripper tip/contact point, not the shorter
# official gripper-frame origin. 82.881 deg at ID14 then gives about X=180.5,
# Z=40.1 mm with q1=q2=q3=0, matching the measured working pose.
L4_X = ROBOTIS_GRIPPER_FRAME_LENGTH + GRIPPER_TIP_EXTENSION

# Calibrated encoder references captured on the real remounted arm.
# This straight, raised REST pose defines mathematical q1=q2=q3=q4=0.
# ID14 uses its physical/encoder zero here; the old RAW 943 pose is WORK.
REST_RAW = {11: 1917, 12: 2046, 13: 4049, 14: 0}
# Compatibility alias for older files and commands. New UI text calls it REST.
HOME_RAW = REST_RAW
JOINT_DIRECTION = {11: 1.0, 12: 1.0, 13: 1.0, 14: 1.0}

REST_ID11 = REST_RAW[11] * 360.0 / 4096.0
REST_ID12 = REST_RAW[12] * 360.0 / 4096.0
REST_ID13 = REST_RAW[13] * 360.0 / 4096.0
REST_ID14 = REST_RAW[14] * 360.0 / 4096.0

HOME_ID11 = REST_ID11
HOME_ID12 = REST_ID12
HOME_ID13 = REST_ID13
HOME_ID14 = REST_ID14

REST_JOINT_DEGREES = (0.0, 0.0, 0.0, 0.0)
# The official ROBOTIS gripper-frame origin and the experiment's farther
# working-tip TCP are both retained explicitly.
REST_OFFICIAL_XYZ_MM = (286.0, 0.0, 204.5)
REST_XYZ_MM = (325.7, 0.0, 204.5)

# Downward working pose captured previously. ID14 RAW 943 is 82.880859 deg,
# which is also calibrated q4 because REST ID14 RAW is zero.
WORK_RAW = {11: 1917, 12: 2046, 13: 4049, 14: 943}
WORK_JOINT_DEGREES = (0.0, 0.0, 0.0, 82.880859375)
WORK_XYZ_MM = (180.53569397713747, 0.0, 40.077449013593764)
WORK_MOTOR_DEGREES = (
    168.486328125,
    179.82421875,
    355.869140625,
    82.880859375,
)

DEFAULT_THETA4 = 0.0
IK_POSITION_TOLERANCE_MM = 2.0
KINEMATICS_VERSION = "gripper-tip-v4-rotating-base-offset"

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT_SECONDS = 2.0
MOVE_TIMEOUT_SECONDS = 30.0
POST_MOVE_TOLERANCE_DEGREES = 2.0

# OpenCR expects degrees in the same 0..360 encoder-angle convention it returns.
RAW_COUNTS_PER_REV = 4096
RAW_ZERO_DEGREES = 0.0


@dataclass(frozen=True)
class JointLimit:
    minimum: float
    maximum: float

    def contains(self, value: float) -> bool:
        return self.minimum <= value <= self.maximum


# Provisional Cartesian envelope from the manual-pose samples. The robot's
# physical work surface is below the ROBOTIS-model Z=0 base-plane reference on
# this remounted arm, so Z is deliberately not box-limited. IK and calibrated
# joint/motor limits remain mandatory for every target.
MEASURED_XYZ_LIMITS = {
    "x": JointLimit(0.0, 350.0),
    "y": JointLimit(-170.0, 170.0),
    "z": None,
}


# FK joint limits are intentionally conservative software limits. Adjust after
# measured robot-limit validation.
FK_JOINT_LIMITS = {
    "theta1": JointLimit(-90.0, 100.0),
    "theta2": JointLimit(-15.0, 85.0),
    "theta3": JointLimit(-60.0, 90.0),
    "theta4": JointLimit(-45.0, 100.0),
}

# Motor encoder display/command limits. These cover one encoder revolution.
MOTOR_ANGLE_LIMITS = {
    11: JointLimit(0.0, 360.0),
    12: JointLimit(0.0, 360.0),
    13: JointLimit(0.0, 360.0),
    14: JointLimit(0.0, 360.0),
}
