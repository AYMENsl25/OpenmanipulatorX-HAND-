import math


# ============================================================
# OPENMANIPULATOR-X FORWARD KINEMATICS
# Motor angles -> X / Y / Z
# ============================================================

# ------------------------------------------------------------
# ROBOTIS LINK DIMENSIONS (mm)
# ------------------------------------------------------------

Z_BASE = 17.0
L1_Z    = 59.5
L2_X    = 24.0
L2_Z    = 128.0
L3_X    = 124.0
L4_X    = 126.0


# ------------------------------------------------------------
# YOUR MOTOR ZERO REFERENCES
# ------------------------------------------------------------

# ID11:
# Motor angle 351 deg = base centered
ID11_CENTER = 351.0

# ID12:
# Motor 0 deg = arm horizontal
ID12_ZERO = 0.0

# ID13:
# Motor 0 deg = arm straight/vertical
ID13_ZERO = 0.0

# ID14:
# Motor 0 deg = hand vertical/straight
ID14_ZERO = 0.0


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

    # --------------------------------------------------------
    # ID11 = BASE
    # 351 deg motor position = physical center
    # --------------------------------------------------------

    theta1 = normalize_angle(id11 - ID11_CENTER)


    # --------------------------------------------------------
    # ID12 = SHOULDER
    # 0 deg = horizontal
    #
    # For the first FK model we use:
    # horizontal = 0 deg
    # --------------------------------------------------------

    theta2 = normalize_angle(id12 - ID12_ZERO)


    # --------------------------------------------------------
    # ID13 = ELBOW
    # 0 deg = straight vertical reference
    # --------------------------------------------------------

    theta3 = normalize_angle(id13 - ID13_ZERO)


    # --------------------------------------------------------
    # ID14 = WRIST
    # 0 deg = hand vertical/straight
    # --------------------------------------------------------

    theta4 = normalize_angle(id14 - ID14_ZERO)


    return theta1, theta2, theta3, theta4


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(id11, id12, id13, id14):

    # Convert motor angles to FK angles
    theta1, theta2, theta3, theta4 = motor_to_fk_angles(
        id11,
        id12,
        id13,
        id14
    )

    # Convert degrees -> radians
    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4 = math.radians(theta4)


    # ========================================================
    # ARM PLANAR ANGLES
    # ========================================================

    # These terms describe the arm in the vertical plane.
    #
    # The exact zero/sign convention is intentionally kept
    # separate above so it can be calibrated easily.
    # ========================================================

    a2 = t2
    a3 = t2 + t3
    a4 = t2 + t3 + t4


    # ========================================================
    # RADIAL DISTANCE FROM BASE
    # ========================================================

    radial = (
        L2_X * math.cos(a2)
        + L3_X * math.cos(a3)
        + L4_X * math.cos(a4)
    )


    # ========================================================
    # HEIGHT
    # ========================================================

    z = (
        Z_BASE
        + L1_Z
        + L2_Z * math.sin(a2)
        + L3_X * math.sin(a3)
        + L4_X * math.sin(a4)
    )


    # ========================================================
    # X / Y FROM BASE ROTATION
    # ========================================================

    x = radial * math.cos(t1)
    y = radial * math.sin(t1)


    return x, y, z


# ============================================================
# TEST WITH YOUR EXCEL ROW
# ============================================================

if __name__ == "__main__":

    # Example:
    # Index 369
    ID11 = 351
    ID12 = 1
    ID13 = 1
    ID14 = 90

    x, y, z = forward_kinematics(
        ID11,
        ID12,
        ID13,
        ID14
    )

    print("=" * 60)
    print("OPENMANIPULATOR-X FORWARD KINEMATICS")
    print("=" * 60)

    print()
    print("MOTOR ANGLES")
    print("-" * 60)
    print(f"ID11 = {ID11:.3f} deg")
    print(f"ID12 = {ID12:.3f} deg")
    print(f"ID13 = {ID13:.3f} deg")
    print(f"ID14 = {ID14:.3f} deg")

    theta1, theta2, theta3, theta4 = motor_to_fk_angles(
        ID11,
        ID12,
        ID13,
        ID14
    )

    print()
    print("FK ANGLES")
    print("-" * 60)
    print(f"Theta1 = {theta1:.3f} deg")
    print(f"Theta2 = {theta2:.3f} deg")
    print(f"Theta3 = {theta3:.3f} deg")
    print(f"Theta4 = {theta4:.3f} deg")

    print()
    print("END-EFFECTOR POSITION")
    print("-" * 60)
    print(f"X = {x:.3f} mm")
    print(f"Y = {y:.3f} mm")
    print(f"Z = {z:.3f} mm")

    print("=" * 60)