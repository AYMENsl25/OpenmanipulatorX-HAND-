"""Project configuration for the OpenMANIPULATOR-X XYZ controller.

These values are experimental project calibration values, not universal
ROBOTIS zero references. Keep all calibration edits in this file.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


from app_paths import PROJECT_ROOT

EXPERIMENT_CONFIG_PATH = PROJECT_ROOT / "config" / "experiment_config.json"
with EXPERIMENT_CONFIG_PATH.open("r", encoding="utf-8") as config_file:
    EXPERIMENT_CONFIG = json.load(config_file)

PHYSICAL_FRAME = EXPERIMENT_CONFIG["physical_frame"]
OPERATOR_REAR_FRAME = EXPERIMENT_CONFIG["operator_rear_frame"]
REAR_TO_AXIS_X_MM = float(OPERATOR_REAR_FRAME["rear_to_axis_x_mm"])
PHYSICAL_TO_INTERNAL = EXPERIMENT_CONFIG["physical_to_internal"]
SOFT_WORKSPACE = EXPERIMENT_CONFIG["soft_workspace"]
POINT_EXPERIMENT = EXPERIMENT_CONFIG["point_experiment"]
REPEATABILITY_EXPERIMENT = EXPERIMENT_CONFIG["repeatability_experiment"]
CAMERA_PAYLOAD_PROFILE = REPEATABILITY_EXPERIMENT["camera_payload_profile"]
PICK_PLACE_MOTION = EXPERIMENT_CONFIG["pick_place_motion"]
EXPERIMENT_POINTS = tuple(EXPERIMENT_CONFIG["experiment_points"])
GROUND_Z_MM = float(PHYSICAL_FRAME["ground_z_mm"])
APPROACH_CLEARANCE_MM = float(POINT_EXPERIMENT["approach_clearance_mm"])
RETRACT_CLEARANCE_MM = float(POINT_EXPERIMENT["retract_clearance_mm"])
TOUCH_DWELL_SECONDS = float(POINT_EXPERIMENT["touch_dwell_seconds"])
POINT_SETTLE_SECONDS = float(POINT_EXPERIMENT["settle_seconds"])
TELEMETRY_INTERVAL_MS = int(POINT_EXPERIMENT["telemetry_interval_ms"])
ANGLE_READ_ATTEMPTS = int(POINT_EXPERIMENT["angle_read_attempts"])
ANGLE_RETRY_DELAY_SECONDS = float(POINT_EXPERIMENT["angle_retry_delay_ms"]) / 1000.0
TCP_WARNING_ERROR_MM = float(POINT_EXPERIMENT["tcp_warning_error_mm"])
MAXIMUM_TCP_ERROR_MM = float(POINT_EXPERIMENT["maximum_tcp_error_mm"])
DEFAULT_REPETITIONS = int(REPEATABILITY_EXPERIMENT["default_repetitions"])
MAXIMUM_REPETITIONS = int(REPEATABILITY_EXPERIMENT["maximum_repetitions"])
TOUCH_SAMPLE_COUNT = int(REPEATABILITY_EXPERIMENT["touch_sample_count"])
MAXIMUM_TOUCH_SAMPLES = int(REPEATABILITY_EXPERIMENT["maximum_touch_samples"])
TOUCH_SAMPLE_INTERVAL_SECONDS = (
    float(REPEATABILITY_EXPERIMENT["touch_sample_interval_ms"]) / 1000.0
)
PAUSE_FOR_MANUAL_MEASUREMENT = bool(
    REPEATABILITY_EXPERIMENT["pause_for_manual_measurement"]
)
CAMERA_PAYLOAD_ENABLED_DEFAULT = bool(CAMERA_PAYLOAD_PROFILE["enabled_by_default"])
CAMERA_PAYLOAD_MAXIMUM_SPEED_SCALE = float(CAMERA_PAYLOAD_PROFILE["maximum_speed_scale"])
CAMERA_PAYLOAD_MINIMUM_TOUCH_SAMPLES = int(CAMERA_PAYLOAD_PROFILE["minimum_touch_samples"])
CAMERA_PAYLOAD_MINIMUM_SAMPLE_INTERVAL_MS = int(
    CAMERA_PAYLOAD_PROFILE["minimum_sample_interval_ms"]
)
CAMERA_PAYLOAD_SETTLE_SECONDS = float(CAMERA_PAYLOAD_PROFILE["settle_seconds"])
CAMERA_PAYLOAD_TOUCH_DWELL_SECONDS = float(CAMERA_PAYLOAD_PROFILE["touch_dwell_seconds"])
CAMERA_PAYLOAD_MOTOR_TRACKING_WARNING_DEGREES = float(
    CAMERA_PAYLOAD_PROFILE["motor_tracking_warning_deg"]
)
PICK_PLACE_DEFAULT_SPEED_SCALE = float(PICK_PLACE_MOTION["default_speed_scale"])
PICK_PLACE_MAXIMUM_SPEED_SCALE = float(PICK_PLACE_MOTION["maximum_speed_scale"])
PICK_PLACE_POST_GRASP_SETTLE_SECONDS = float(
    PICK_PLACE_MOTION["post_grasp_settle_seconds"]
)
PICK_PLACE_POST_RELEASE_SETTLE_SECONDS = float(
    PICK_PLACE_MOTION["post_release_settle_seconds"]
)

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
# The experiment TCP is the center between the two finger tips.  The original
# 39.7 mm axial extension made that real TCP sit 7 mm above the physical ground
# when FK reported Z=0 at X=180, Y=0.  Shorten the axial extension by the
# measured gap; this keeps Z=0 as the protected physical ground plane instead
# of hiding the calibration error in a negative-Z command or frame offset.
NOMINAL_GRIPPER_TIP_EXTENSION = 39.7
MEASURED_TCP_GROUND_GAP_MM = 7.0
GRIPPER_TIP_EXTENSION = NOMINAL_GRIPPER_TIP_EXTENSION - MEASURED_TCP_GROUND_GAP_MM
TCP_DESCRIPTION = "Center between the two finger tips"
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
REST_XYZ_MM = (318.7, 0.0, 204.5)

# Camera SCAN pose measured with the installed payload. ID12-ID14 are the
# average of six stable readings supplied on 2026-09-21. ID11 is deliberately
# set to the calibrated q1=0 center so the scan plane is centered at Y=0.
SCAN_MOTOR_DEGREES = (
    REST_ID11,
    159.829,
    330.0735,
    124.57033333333334,
)
SCAN_JOINT_DEGREES = (
    0.0,
    SCAN_MOTOR_DEGREES[1] - REST_ID12,
    SCAN_MOTOR_DEGREES[2] - REST_ID13,
    SCAN_MOTOR_DEGREES[3] - REST_ID14,
)
SCAN_XYZ_MM = (108.12825286280928, 0.0, 138.2068472595612)

# Downward working pose captured previously. ID14 RAW 943 is 82.880859 deg,
# which is also calibrated q4 because REST ID14 RAW is zero.
WORK_RAW = {11: 1917, 12: 2046, 13: 4049, 14: 943}
WORK_JOINT_DEGREES = (0.0, 0.0, 0.0, 82.880859375)
WORK_XYZ_MM = (179.66816315130788, 0.0, 47.02348315303158)
WORK_MOTOR_DEGREES = (
    168.486328125,
    179.82421875,
    355.869140625,
    82.880859375,
)

DEFAULT_THETA4 = 0.0
IK_POSITION_TOLERANCE_MM = 2.0
KINEMATICS_VERSION = "finger-center-tcp-v6-ground-gap-7mm"

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT_SECONDS = 2.0
MOVE_TIMEOUT_SECONDS = 30.0
# Data-collection arrival threshold; not an accuracy claim or a joint limit.
POST_MOVE_TOLERANCE_DEGREES = 10.0
MOTOR_TRACKING_WARNING_DEGREES = 8.0 * 360.0 / 4096.0

# OpenCR expects degrees in the same 0..360 encoder-angle convention it returns.
RAW_COUNTS_PER_REV = 4096
RAW_ZERO_DEGREES = 0.0
JOINT_LIMIT_RAW_TOLERANCE_DEGREES = 0.5 * 360.0 / RAW_COUNTS_PER_REV + 0.001


@dataclass(frozen=True)
class JointLimit:
    minimum: float
    maximum: float

    def contains(self, value: float) -> bool:
        return self.minimum <= value <= self.maximum


# Configurable physical experiment envelope. Z has no arbitrary upper box
# limit; only the calibrated physical ground plane is enforced.
MEASURED_XYZ_LIMITS = {
    "x": JointLimit(float(SOFT_WORKSPACE["x_min_mm"]), float(SOFT_WORKSPACE["x_max_mm"])),
    "y": JointLimit(float(SOFT_WORKSPACE["y_min_mm"]), float(SOFT_WORKSPACE["y_max_mm"])),
    "z": None,
}


# FK joint limits are intentionally conservative software limits. Adjust after
# measured robot-limit validation.
FK_JOINT_LIMITS = {
    # Expanded for measured P01/P07 coverage. Their required base angles are
    # approximately -103.57/+103.57 deg; +/-110 leaves controlled margin.
    "theta1": JointLimit(-110.0, 110.0),
    # Expanded only far enough to contain the measured camera SCAN pose
    # q2=-19.995 deg, with approximately five degrees of controlled margin.
    "theta2": JointLimit(-25.0, 85.0),
    "theta3": JointLimit(-60.0, 90.0),
    # SCAN uses q4=124.570 deg; 130 deg leaves measured operating margin.
    "theta4": JointLimit(-45.0, 130.0),
}

# Keep the validated Cartesian experiment solver on its established branches.
# The wider FK/motion limits above are used only to admit measured direct motor
# poses such as SCAN; they must not silently change P01-P11 IK solutions.
IK_SOLVER_JOINT_LIMITS = {
    "theta1": JointLimit(-110.0, 110.0),
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
