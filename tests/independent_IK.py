import math
import numpy as np
from scipy.optimize import least_squares


# ============================================================
# OPENMANIPULATOR-X
# INDEPENDENT IK -> FK VALIDATION
#
# No robot required.
#
# Target = known HOME pose
#
# IK receives ONLY:
#   X Y Z
#   Roll Pitch Yaw
#
# It calculates J1-J4 independently.
#
# Then FK is applied to those calculated angles.
# ============================================================


# ============================================================
# HOME TARGET POSE
# ============================================================

TARGET_X = 169.6729
TARGET_Y = -21.4023
TARGET_Z = 86.9950

TARGET_ROLL = 0.00
TARGET_PITCH = 84.82
TARGET_YAW = -7.73


TARGET_POSITION = np.array([
    TARGET_X,
    TARGET_Y,
    TARGET_Z
])


TARGET_RPY = np.array([
    TARGET_ROLL,
    TARGET_PITCH,
    TARGET_YAW
])


# ============================================================
# OPENMANIPULATOR-X CHAIN PARAMETERS
# ============================================================
#
# meters
#
# Based on the ROBOTIS OpenMANIPULATOR-X chain definition.
#
# Joint 1 origin:
#     0.012, 0, 0.017
#
# Joint 2:
#     0, 0, 0.0595
#
# Joint 3:
#     0.024, 0, 0.128
#
# Joint 4:
#     0.124, 0, 0
#
# End-effector:
#     0.126, 0, 0
#
# ============================================================


P1 = np.array([
    0.012,
    0.0,
    0.017
])

P2 = np.array([
    0.0,
    0.0,
    0.0595
])

P3 = np.array([
    0.024,
    0.0,
    0.128
])

P4 = np.array([
    0.124,
    0.0,
    0.0
])

P_TOOL = np.array([
    0.126,
    0.0,
    0.0
])


# ============================================================
# ROTATIONS
# ============================================================

def Rx(a):

    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])


def Ry(a):

    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])


def Rz(a):

    c = math.cos(a)
    s = math.sin(a)

    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])


# ============================================================
# HOMOGENEOUS TRANSFORMATION
# ============================================================

def transform(R, p):

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = p

    return T


# ============================================================
# FK
# ============================================================

def FK(q_deg):

    q1, q2, q3, q4 = np.deg2rad(q_deg)

    T01 = transform(
        Rz(q1),
        P1
    )

    T12 = transform(
        Ry(q2),
        P2
    )

    T23 = transform(
        Ry(q3),
        P3
    )

    T34 = transform(
        Ry(q4),
        P4
    )

    T4E = transform(
        np.eye(3),
        P_TOOL
    )

    T = (
        T01
        @ T12
        @ T23
        @ T34
        @ T4E
    )

    return T


# ============================================================
# ROTATION MATRIX -> RPY
# ============================================================

def matrix_to_rpy(R):

    sy = math.sqrt(
        R[0, 0] ** 2 +
        R[1, 0] ** 2
    )

    if sy > 1e-9:

        roll = math.atan2(
            R[2, 1],
            R[2, 2]
        )

        pitch = math.atan2(
            -R[2, 0],
            sy
        )

        yaw = math.atan2(
            R[1, 0],
            R[0, 0]
        )

    else:

        roll = math.atan2(
            -R[1, 2],
            R[1, 1]
        )

        pitch = math.atan2(
            -R[2, 0],
            sy
        )

        yaw = 0.0

    return np.degrees([
        roll,
        pitch,
        yaw
    ])


# ============================================================
# ANGLE DIFFERENCE
# ============================================================

def angle_difference(a, b):

    return (
        (a - b + 180.0)
        % 360.0
    ) - 180.0


# ============================================================
# POSE FROM JOINTS
# ============================================================

def pose_from_joints(q):

    T = FK(q)

    position_mm = (
        T[:3, 3] * 1000.0
    )

    rpy = matrix_to_rpy(
        T[:3, :3]
    )

    return position_mm, rpy


# ============================================================
# IK ERROR FUNCTION
# ============================================================
#
# IMPORTANT:
#
# The optimizer does NOT receive the original home angles.
#
# It only sees the target pose.
#
# Position error:
#     mm
#
# Orientation error:
#     degrees
#
# ============================================================

def ik_error(q):

    position, rpy = pose_from_joints(q)

    position_error = (
        position - TARGET_POSITION
    )

    orientation_error = np.array([
        angle_difference(
            rpy[0],
            TARGET_ROLL
        ),

        angle_difference(
            rpy[1],
            TARGET_PITCH
        ),

        angle_difference(
            rpy[2],
            TARGET_YAW
        )
    ])

    # Position has higher numerical weight.
    #
    # 1 degree orientation error is treated
    # approximately as 1 mm for optimization.
    #
    return np.concatenate([
        position_error,
        orientation_error
    ])


# ============================================================
# INDEPENDENT IK
# ============================================================
#
# DELIBERATELY DIFFERENT INITIAL GUESS
#
# We do NOT start from:
#
# [-7.73, 0, -3.69, 88.51]
#
# Instead:
#
# [30, 10, 20, 60]
#
# This prevents the optimizer from simply
# returning the known home configuration.
# ============================================================

INITIAL_GUESS = np.array([
    30.0,
    10.0,
    20.0,
    60.0
])


# ============================================================
# JOINT LIMITS
# ============================================================
#
# The optimizer works in degrees.
#
# These are broad mathematical limits for this test.
#
# We are NOT commanding the physical robot.
# ============================================================

LOWER_LIMITS = np.array([
    -180.0,
    -180.0,
    -180.0,
    -180.0
])


UPPER_LIMITS = np.array([
    180.0,
    180.0,
    180.0,
    180.0
])


# ============================================================
# SOLVE IK
# ============================================================

result = least_squares(

    ik_error,

    INITIAL_GUESS,

    bounds=(
        LOWER_LIMITS,
        UPPER_LIMITS
    ),

    xtol=1e-13,
    ftol=1e-13,
    gtol=1e-13,

    max_nfev=5000
)


IK_JOINTS = result.x


# ============================================================
# FK OF IK SOLUTION
# ============================================================

calculated_position, calculated_rpy = (
    pose_from_joints(IK_JOINTS)
)


# ============================================================
# ERRORS
# ============================================================

joint_error = np.array([
    angle_difference(
        IK_JOINTS[0],
        -7.73
    ),

    angle_difference(
        IK_JOINTS[1],
        0.0
    ),

    angle_difference(
        IK_JOINTS[2],
        -3.69
    ),

    angle_difference(
        IK_JOINTS[3],
        88.51
    )
])


xyz_error_vector = (
    calculated_position
    - TARGET_POSITION
)


xyz_error = np.linalg.norm(
    xyz_error_vector
)


rpy_error = np.array([

    angle_difference(
        calculated_rpy[0],
        TARGET_ROLL
    ),

    angle_difference(
        calculated_rpy[1],
        TARGET_PITCH
    ),

    angle_difference(
        calculated_rpy[2],
        TARGET_YAW
    )

])


# ============================================================
# OUTPUT
# ============================================================

print()
print("=" * 65)
print("OPENMANIPULATOR-X INDEPENDENT IK -> FK TEST")
print("=" * 65)


print()
print("TARGET POSE")
print("-" * 65)

print(
    f"X     = {TARGET_X:.6f} mm"
)

print(
    f"Y     = {TARGET_Y:.6f} mm"
)

print(
    f"Z     = {TARGET_Z:.6f} mm"
)

print(
    f"Roll  = {TARGET_ROLL:.6f} deg"
)

print(
    f"Pitch = {TARGET_PITCH:.6f} deg"
)

print(
    f"Yaw   = {TARGET_YAW:.6f} deg"
)


print()
print("INITIAL IK GUESS")
print("-" * 65)

print(
    f"J1 = {INITIAL_GUESS[0]:.6f} deg"
)

print(
    f"J2 = {INITIAL_GUESS[1]:.6f} deg"
)

print(
    f"J3 = {INITIAL_GUESS[2]:.6f} deg"
)

print(
    f"J4 = {INITIAL_GUESS[3]:.6f} deg"
)


print()
print("IK CALCULATED JOINTS")
print("-" * 65)

print(
    f"J1 = {IK_JOINTS[0]:.6f} deg"
)

print(
    f"J2 = {IK_JOINTS[1]:.6f} deg"
)

print(
    f"J3 = {IK_JOINTS[2]:.6f} deg"
)

print(
    f"J4 = {IK_JOINTS[3]:.6f} deg"
)


print()
print("FK FROM IK JOINTS")
print("-" * 65)

print(
    f"X     = {calculated_position[0]:.6f} mm"
)

print(
    f"Y     = {calculated_position[1]:.6f} mm"
)

print(
    f"Z     = {calculated_position[2]:.6f} mm"
)

print(
    f"Roll  = {calculated_rpy[0]:.6f} deg"
)

print(
    f"Pitch = {calculated_rpy[1]:.6f} deg"
)

print(
    f"Yaw   = {calculated_rpy[2]:.6f} deg"
)


print()
print("XYZ ERROR")
print("-" * 65)

print(
    f"dX = {xyz_error_vector[0]:.9f} mm"
)

print(
    f"dY = {xyz_error_vector[1]:.9f} mm"
)

print(
    f"dZ = {xyz_error_vector[2]:.9f} mm"
)

print(
    f"Total XYZ error = {xyz_error:.9f} mm"
)


print()
print("ORIENTATION ERROR")
print("-" * 65)

print(
    f"Roll  error  = {rpy_error[0]:.9f} deg"
)

print(
    f"Pitch error  = {rpy_error[1]:.9f} deg"
)

print(
    f"Yaw   error  = {rpy_error[2]:.9f} deg"
)


print()
print("JOINT ERROR")
print("-" * 65)

print(
    f"J1 error = {joint_error[0]:.9f} deg"
)

print(
    f"J2 error = {joint_error[1]:.9f} deg"
)

print(
    f"J3 error = {joint_error[2]:.9f} deg"
)

print(
    f"J4 error = {joint_error[3]:.9f} deg"
)


print()
print("SOLVER STATUS")
print("-" * 65)

print(
    "Success:",
    result.success
)

print(
    "Iterations:",
    result.nfev
)

print(
    "Cost:",
    result.cost
)

print(
    "Message:",
    result.message
)


# ============================================================
# VALIDATION
# ============================================================

if (
    xyz_error < 0.01
    and
    np.max(np.abs(rpy_error)) < 0.01
):

    print()
    print("=" * 65)
    print("IK -> FK VALIDATION: PASS")
    print("=" * 65)

else:

    print()
    print("=" * 65)
    print("IK -> FK VALIDATION: CHECK FAILED")
    print("=" * 65)


print()
print("ID15 / GRIPPER")
print("-" * 65)

print(
    "ID15 is NOT included in the 4-DOF arm IK/FK."
)

print(
    "Home gripper angle = 142.82 deg"
)

print()
print("No physical robot was commanded.")
print()