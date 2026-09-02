import math
import serial
import time


# ============================================================
# OPENMANIPULATOR-X
# MANUAL XYZ -> IK -> MOTORS
# ============================================================

SERIAL_PORT = "COM7"
BAUDRATE = 115200


# ============================================================
# HOME
# ============================================================

HOME_X = 143.523
HOME_Y = 0.000
HOME_Z = 208.985

HOME_ID11 = 351.000
HOME_ID12 = 1.000
HOME_ID13 = 1.000
HOME_ID14 = 90.000


# ============================================================
# LINK DIMENSIONS
# ============================================================

L1 = 77.0
L2 = 130.0
L3 = 124.0


# ============================================================
# IK
# ============================================================

def inverse_kinematics(x, y, z):

    # ----------------------------
    # Joint 1
    # ----------------------------

    theta1 = math.degrees(
        math.atan2(y, x)
    )

    # ----------------------------
    # Horizontal distance
    # ----------------------------

    r = math.sqrt(
        x * x +
        y * y
    )

    # ----------------------------
    # Vertical distance
    # ----------------------------

    pz = z - L1

    # ----------------------------
    # Elbow calculation
    # ----------------------------

    D = (
        r * r +
        pz * pz -
        L2 * L2 -
        L3 * L3
    ) / (
        2.0 * L2 * L3
    )

    # Check reachability
    if D < -1.0 or D > 1.0:
        raise ValueError(
            f"Target is outside the IK workspace. D={D:.4f}"
        )

    D = max(-1.0, min(1.0, D))

    # ----------------------------
    # Elbow-down solution
    # ----------------------------

    theta3 = math.atan2(
        math.sqrt(1.0 - D * D),
        D
    )

    theta2 = math.atan2(
        pz,
        r
    ) - math.atan2(
        L3 * math.sin(theta3),
        L2 + L3 * math.cos(theta3)
    )

    theta2 = math.degrees(theta2)
    theta3 = math.degrees(theta3)

    # End-effector orientation
    theta4 = 90.0

    # ========================================================
    # MOTOR CONVENTION
    # ========================================================

    motor11 = HOME_ID11 + theta1
    motor12 = theta2
    motor13 = theta3
    motor14 = theta4

    return (
        motor11,
        motor12,
        motor13,
        motor14
    )


# ============================================================
# SEND MOVE
# ============================================================

def send_move(ser, a11, a12, a13, a14):

    command = (
        f"MOVE,"
        f"{a11:.3f},"
        f"{a12:.3f},"
        f"{a13:.3f},"
        f"{a14:.3f}\n"
    )

    print()
    print("Sending:")
    print(command.strip())

    ser.write(command.encode("ascii"))
    ser.flush()

    # Small delay so OpenCR can process command
    time.sleep(0.05)


# ============================================================
# HOME
# ============================================================

def go_home(ser):

    print()
    print("=" * 60)
    print("MOVING HOME")
    print("=" * 60)

    print(
        f"HOME = "
        f"({HOME_ID11:.3f}, "
        f"{HOME_ID12:.3f}, "
        f"{HOME_ID13:.3f}, "
        f"{HOME_ID14:.3f})"
    )

    send_move(
        ser,
        HOME_ID11,
        HOME_ID12,
        HOME_ID13,
        HOME_ID14
    )

    time.sleep(1.0)

    print("HOME COMMAND SENT.")


# ============================================================
# MOVE TO XYZ
# ============================================================

def move_xyz(ser, x, y, z):

    print()
    print("=" * 60)
    print("TARGET XYZ")
    print("=" * 60)

    print(f"X = {x:.3f} mm")
    print(f"Y = {y:.3f} mm")
    print(f"Z = {z:.3f} mm")

    try:

        (
            a11,
            a12,
            a13,
            a14
        ) = inverse_kinematics(
            x,
            y,
            z
        )

    except ValueError as e:

        print()
        print("IK ERROR")
        print(e)

        return

    print()
    print("IK MOTOR ANGLES")
    print("-" * 60)

    print(f"ID11 = {a11:.3f} deg")
    print(f"ID12 = {a12:.3f} deg")
    print(f"ID13 = {a13:.3f} deg")
    print(f"ID14 = {a14:.3f} deg")

    send_move(
        ser,
        a11,
        a12,
        a13,
        a14
    )

    print()
    print("MOVE COMMAND SENT.")


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 60)
    print("OPENMANIPULATOR-X")
    print("MANUAL XYZ -> IK -> MOTORS")
    print("=" * 60)

    print()
    print("Controls:")
    print("  P = Enter XYZ target")
    print("  H = Home")
    print("  Q = Quit")

    print()
    print("Opening", SERIAL_PORT, "...")

    try:

        ser = serial.Serial(
            SERIAL_PORT,
            BAUDRATE,
            timeout=1,
            write_timeout=2
        )

    except Exception as e:

        print()
        print("SERIAL ERROR:")
        print(e)
        return

    time.sleep(2)

    # Clear old data
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("OpenCR connected.")

    # ========================================================
    # START AT HOME
    # ========================================================

    go_home(ser)

    # ========================================================
    # INTERACTIVE LOOP
    # ========================================================

    while True:

        print()
        print("=" * 60)
        print("COMMAND")
        print("=" * 60)

        choice = input(
            "Enter P for position, H for home, Q to quit: "
        ).strip().upper()

        # ----------------------------------------------------
        # HOME
        # ----------------------------------------------------

        if choice == "H":

            go_home(ser)

        # ----------------------------------------------------
        # POSITION
        # ----------------------------------------------------

        elif choice == "P":

            try:

                print()

                x = float(
                    input("Enter X (mm): ")
                )

                y = float(
                    input("Enter Y (mm): ")
                )

                z = float(
                    input("Enter Z (mm): ")
                )

                move_xyz(
                    ser,
                    x,
                    y,
                    z
                )

            except ValueError:

                print()
                print("Please enter valid numbers.")

        # ----------------------------------------------------
        # QUIT
        # ----------------------------------------------------

        elif choice == "Q":

            print()
            print("Closing serial connection...")

            ser.close()

            print("Done.")

            break

        else:

            print("Unknown command.")


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()