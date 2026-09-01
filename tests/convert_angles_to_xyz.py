
import math
import os
from openpyxl import load_workbook, Workbook


# ============================================================
# OPENMANIPULATOR-X
# ANGLES EXCEL -> XYZ EXCEL
# ============================================================

# ------------------------------------------------------------
# ROBOTIS LINK DIMENSIONS (mm)
# ------------------------------------------------------------

Z_BASE = 17.0
L1_Z   = 59.5
L2_X   = 24.0
L2_Z   = 128.0
L3_X   = 124.0
L4_X   = 126.0


# ------------------------------------------------------------
# HOME / ZERO REFERENCES
# ------------------------------------------------------------

# Home position:
#
# ID11 = 351 deg
# ID12 = 1 deg
# ID13 = 1 deg
# ID14 = 90 deg
#
# At home:
#
# X = 143.523 mm
# Y = 0.000 mm
# Z = 208.985 mm
# ------------------------------------------------------------

ID11_CENTER = 351.0
ID12_ZERO   = 0.0
ID13_ZERO   = 0.0
ID14_ZERO   = 0.0


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

    theta1 = normalize_angle(id11 - ID11_CENTER)

    theta2 = normalize_angle(id12 - ID12_ZERO)

    theta3 = normalize_angle(id13 - ID13_ZERO)

    theta4 = normalize_angle(id14 - ID14_ZERO)

    return theta1, theta2, theta3, theta4


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(id11, id12, id13, id14):

    theta1, theta2, theta3, theta4 = motor_to_fk_angles(
        id11,
        id12,
        id13,
        id14
    )

    # Degrees -> radians
    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4 = math.radians(theta4)

    # --------------------------------------------------------
    # PLANAR ANGLES
    # --------------------------------------------------------

    a2 = t2
    a3 = t2 + t3
    a4 = t2 + t3 + t4

    # --------------------------------------------------------
    # RADIAL DISTANCE
    # --------------------------------------------------------

    radial = (
        L2_X * math.cos(a2)
        + L3_X * math.cos(a3)
        + L4_X * math.cos(a4)
    )

    # --------------------------------------------------------
    # HEIGHT
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
# FILE SETTINGS
# ============================================================

INPUT_FILE = "openmanipulator_teaching_angles.xlsx"

OUTPUT_FILE = "openmanipulator_trajectory_xyz.xlsx"


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("=" * 70)
    print("OPENMANIPULATOR-X")
    print("ANGLES EXCEL -> XYZ EXCEL")
    print("=" * 70)

    # --------------------------------------------------------
    # CHECK INPUT FILE
    # --------------------------------------------------------

    if not os.path.exists(INPUT_FILE):

        print()
        print("ERROR")
        print("-" * 70)
        print(f"Input file not found:")
        print(f"  {INPUT_FILE}")
        print()
        print("Put trajectory_angles.xlsx in the same folder")
        print("as this Python script.")
        print("=" * 70)

        raise SystemExit


    # --------------------------------------------------------
    # LOAD ANGLE EXCEL
    # --------------------------------------------------------

    input_wb = load_workbook(INPUT_FILE)
    input_ws = input_wb.active

    print()
    print(f"Reading: {INPUT_FILE}")


    # --------------------------------------------------------
    # CREATE NEW XYZ EXCEL
    # --------------------------------------------------------

    output_wb = Workbook()
    output_ws = output_wb.active

    output_ws.title = "XYZ Trajectory"


    # --------------------------------------------------------
    # CREATE HEADERS
    # --------------------------------------------------------

    output_ws.append([
        "Index",
        "Time",
        "X",
        "Y",
        "Z"
    ])


    # --------------------------------------------------------
    # READ EVERY ANGLE ROW
    # --------------------------------------------------------

    processed = 0

    for row in input_ws.iter_rows(min_row=2, values_only=True):

        if not row:
            continue

        # Expected input:
        #
        # Index | Time | ID11 | ID12 | ID13 | ID14
        #

        index = row[0]
        time_value = row[1]

        id11 = row[2]
        id12 = row[3]
        id13 = row[4]
        id14 = row[5]

        # Skip incomplete rows
        if None in (id11, id12, id13, id14):
            continue

        # ----------------------------------------------------
        # FK
        # ----------------------------------------------------

        x, y, z = forward_kinematics(
            float(id11),
            float(id12),
            float(id13),
            float(id14)
        )

        # ----------------------------------------------------
        # WRITE ONLY XYZ DATA
        # ----------------------------------------------------

        output_ws.append([
            index,
            time_value,
            round(x, 3),
            round(y, 3),
            round(z, 3)
        ])

        processed += 1


    # --------------------------------------------------------
    # SAVE NEW FILE
    # --------------------------------------------------------

    output_wb.save(OUTPUT_FILE)


    # --------------------------------------------------------
    # FINISHED
    # --------------------------------------------------------

    print()
    print("=" * 70)
    print("DONE")
    print("=" * 70)

    print()
    print(f"Rows processed : {processed}")
    print(f"Output file    : {OUTPUT_FILE}")

    print()
    print("The new Excel file contains:")
    print("  Index")
    print("  Time")
    print("  X")
    print("  Y")
    print("  Z")

    print()
    print("Original angle file was NOT modified.")

    print("=" * 70)

