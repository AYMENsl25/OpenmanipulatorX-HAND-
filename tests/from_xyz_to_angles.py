
import math


# ============================================================
# OPENMANIPULATOR-X
# INVERSE KINEMATICS
#
# XYZ  ->  JOINT / MOTOR ANGLES
#
# This version uses your REAL HOME POSITION:
#
# ID11 = 351°
# ID12 = 1°
# ID13 = 1°
# ID14 = 90°
#
# No serial
# No robot movement
# ============================================================


# ============================================================
# ROBOTIS LINK DIMENSIONS (mm)
# ============================================================

Z_BASE = 17.0
L1_Z   = 59.5
L2_X   = 24.0
L2_Z   = 128.0
L3_X   = 124.0
L4_X   = 126.0


# ============================================================
# MOTOR ZERO REFERENCES
# ============================================================

ID11_CENTER = 351.0
ID12_ZERO   = 0.0
ID13_ZERO   = 0.0
ID14_ZERO   = 0.0


# ============================================================
# YOUR REAL HOME POSITION
# ============================================================

HOME_ID11 = 351.0
HOME_ID12 = 1.0
HOME_ID13 = 1.0
HOME_ID14 = 90.0


# ============================================================
# ANGLE HELPERS
# ============================================================

def normalize_angle(angle):
    """
    Convert angle to range [-180, +180)
    """
    return (angle + 180.0) % 360.0 - 180.0


# ============================================================
# MOTOR ANGLES -> FK ANGLES
# ============================================================

def motor_to_fk_angles(id11, id12, id13, id14):

    theta1 = normalize_angle(
        id11 - ID11_CENTER
    )

    theta2 = normalize_angle(
        id12 - ID12_ZERO
    )

    theta3 = normalize_angle(
        id13 - ID13_ZERO
    )

    theta4 = normalize_angle(
        id14 - ID14_ZERO
    )

    return theta1, theta2, theta3, theta4


# ============================================================
# FK ANGLES -> MOTOR ANGLES
# ============================================================

def fk_to_motor_angles(
    theta1,
    theta2,
    theta3,
    theta4
):

    id11 = normalize_angle(
        theta1 + ID11_CENTER
    )

    id12 = normalize_angle(
        theta2 + ID12_ZERO
    )

    id13 = normalize_angle(
        theta3 + ID13_ZERO
    )

    id14 = normalize_angle(
        theta4 + ID14_ZERO
    )

    return id11, id12, id13, id14


# ============================================================
# FORWARD KINEMATICS
#
# Same FK equations that you already tested.
# ============================================================

def forward_kinematics_from_fk_angles(
    theta1,
    theta2,
    theta3,
    theta4
):

    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4 = math.radians(theta4)

    # --------------------------------------------------------
    # Planar angles
    # --------------------------------------------------------

    a2 = t2
    a3 = t2 + t3
    a4 = t2 + t3 + t4

    # --------------------------------------------------------
    # Radial distance
    # --------------------------------------------------------

    radial = (
        L2_X * math.cos(a2)
        + L3_X * math.cos(a3)
        + L4_X * math.cos(a4)
    )

    # --------------------------------------------------------
    # Height
    # --------------------------------------------------------

    z = (
        Z_BASE
        + L1_Z
        + L2_Z * math.sin(a2)
        + L3_X * math.sin(a3)
        + L4_X * math.sin(a4)
    )

    # --------------------------------------------------------
    # X / Y
    # --------------------------------------------------------

    x = radial * math.cos(t1)
    y = radial * math.sin(t1)

    return x, y, z


# ============================================================
# INVERSE KINEMATICS
#
# Input:
#     X, Y, Z
#
# Output:
#     Theta1, Theta2, Theta3, Theta4
#
# Theta4 is fixed at HOME = 90°
# ============================================================

def inverse_kinematics(
    x_target,
    y_target,
    z_target
):

    # ========================================================
    # 1. BASE ANGLE
    # ========================================================

    theta1 = math.degrees(
        math.atan2(
            y_target,
            x_target
        )
    )

    # ========================================================
    # 2. TARGET RADIAL DISTANCE
    # ========================================================

    radial_target = math.sqrt(
        x_target ** 2
        +
        y_target ** 2
    )

    # ========================================================
    # 3. HOME CONFIGURATION
    # ========================================================

    home_theta1, \
    home_theta2, \
    home_theta3, \
    home_theta4 = motor_to_fk_angles(
        HOME_ID11,
        HOME_ID12,
        HOME_ID13,
        HOME_ID14
    )

    # ========================================================
    # 4. FIX WRIST ANGLE
    #
    # We use the same wrist angle as HOME.
    # ========================================================

    theta4 = home_theta4

    # ========================================================
    # 5. SEARCH FOR THETA2 / THETA3
    #
    # We search for the combination that produces the
    # requested radial distance and Z position.
    #
    # The solution is also biased toward your home
    # configuration.
    # ========================================================

    best_error = float("inf")

    best_theta2 = None
    best_theta3 = None

    # --------------------------------------------------------
    # Search range
    # --------------------------------------------------------

    for theta2 in [
        -120.0 + i * 0.5
        for i in range(481)
    ]:

        for theta3 in [
            -180.0 + j * 0.5
            for j in range(721)
        ]:

            t2 = math.radians(theta2)
            t3 = math.radians(theta3)
            t4 = math.radians(theta4)

            # ------------------------------------------------
            # Compound angles
            # ------------------------------------------------

            a2 = t2
            a3 = t2 + t3
            a4 = t2 + t3 + t4

            # ------------------------------------------------
            # Radial distance
            # ------------------------------------------------

            radial = (
                L2_X * math.cos(a2)
                + L3_X * math.cos(a3)
                + L4_X * math.cos(a4)
            )

            # ------------------------------------------------
            # Height
            # ------------------------------------------------

            z = (
                Z_BASE
                + L1_Z
                + L2_Z * math.sin(a2)
                + L3_X * math.sin(a3)
                + L4_X * math.sin(a4)
            )

            # ------------------------------------------------
            # Position error
            # ------------------------------------------------

            position_error = math.sqrt(
                (radial - radial_target) ** 2
                +
                (z - z_target) ** 2
            )

            # ------------------------------------------------
            # Distance from home configuration
            #
            # This helps choose the solution closest to
            # the known physical home posture.
            # ------------------------------------------------

            home_distance = math.sqrt(
                (theta2 - home_theta2) ** 2
                +
                (theta3 - home_theta3) ** 2
            )

            # ------------------------------------------------
            # Combined score
            # ------------------------------------------------

            total_error = (
                position_error
                +
                0.001 * home_distance
            )

            if total_error < best_error:

                best_error = total_error

                best_theta2 = theta2
                best_theta3 = theta3

    # ========================================================
    # CHECK SOLUTION
    # ========================================================

    if best_theta2 is None:

        raise ValueError(
            "No IK solution found."
        )

    # ========================================================
    # RETURN FK ANGLES
    # ========================================================

    return (
        theta1,
        best_theta2,
        best_theta3,
        theta4
    )


# ============================================================
# MAIN TEST
# ============================================================

if __name__ == "__main__":

    # ========================================================
    # TARGET
    #
    # YOUR HOME POSITION XYZ
    #
    # Put the XYZ obtained from your FK here.
    # ========================================================

    X_TARGET = 143.523
    Y_TARGET = 0.000
    Z_TARGET = 208.985

    # ========================================================
    # RUN IK
    # ========================================================

    theta1, \
    theta2, \
    theta3, \
    theta4 = inverse_kinematics(
        X_TARGET,
        Y_TARGET,
        Z_TARGET
    )

    # ========================================================
    # CONVERT TO MOTOR ANGLES
    # ========================================================

    ID11, \
    ID12, \
    ID13, \
    ID14 = fk_to_motor_angles(
        theta1,
        theta2,
        theta3,
        theta4
    )

    # ========================================================
    # FK VERIFICATION
    # ========================================================

    x_check, \
    y_check, \
    z_check = forward_kinematics_from_fk_angles(
        theta1,
        theta2,
        theta3,
        theta4
    )

    # ========================================================
    # PRINT
    # ========================================================

    print("=" * 70)
    print("OPENMANIPULATOR-X INVERSE KINEMATICS")
    print("=" * 70)

    print()
    print("TARGET XYZ")
    print("-" * 70)

    print(f"X = {X_TARGET:.3f} mm")
    print(f"Y = {Y_TARGET:.3f} mm")
    print(f"Z = {Z_TARGET:.3f} mm")

    print()
    print("IK JOINT ANGLES")
    print("-" * 70)

    print(f"Theta1 = {theta1:.3f} deg")
    print(f"Theta2 = {theta2:.3f} deg")
    print(f"Theta3 = {theta3:.3f} deg")
    print(f"Theta4 = {theta4:.3f} deg")

    print()
    print("MOTOR ANGLES")
    print("-" * 70)

    print(f"ID11 = {ID11:.3f} deg")
    print(f"ID12 = {ID12:.3f} deg")
    print(f"ID13 = {ID13:.3f} deg")
    print(f"ID14 = {ID14:.3f} deg")

    print()
    print("FK VERIFICATION")
    print("-" * 70)

    print(f"X = {x_check:.3f} mm")
    print(f"Y = {y_check:.3f} mm")
    print(f"Z = {z_check:.3f} mm")

    print()
    print("ERROR")
    print("-" * 70)

    print(f"dX = {x_check - X_TARGET:.3f} mm")
    print(f"dY = {y_check - Y_TARGET:.3f} mm")
    print(f"dZ = {z_check - Z_TARGET:.3f} mm")

    print()
    print("=" * 70)

