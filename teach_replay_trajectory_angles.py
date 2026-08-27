import serial
import time
import os

from openpyxl import Workbook


# ============================================================
# OPENMANIPULATOR-X
# PYTHON TEACHING / ANGLE STORAGE
# ============================================================
#
# OpenCR:
#     Reads RAW motor positions
#     Sends TRAJ messages
#
# Python:
#     RAW -> DEGREES
#
# Excel:
#     Stores DEGREES
#
# ============================================================


# ============================================================
# SERIAL CONFIGURATION
# ============================================================

COM_PORT = "COM7"
BAUD_RATE = 115200


# ============================================================
# EXCEL
# ============================================================

EXCEL_FILE = "openmanipulator_teaching_angles.xlsx"


# ============================================================
# DYNAMIXEL ANGLE CONVERSION
# ============================================================

DXL_RESOLUTION = 4096.0
DXL_FULL_ROTATION = 360.0


def raw_to_degrees(raw):
    """
    Convert DYNAMIXEL RAW position to encoder angle.
    """

    return (
        raw *
        DXL_FULL_ROTATION /
        DXL_RESOLUTION
    )


# ============================================================
# CREATE EXCEL
# ============================================================

def create_excel():

    workbook = Workbook()

    worksheet = workbook.active

    worksheet.title = "Teaching Angles"


    worksheet.append([
        "Index",
        "Time (s)",
        "ID11 (deg)",
        "ID12 (deg)",
        "ID13 (deg)",
        "ID14 (deg)",
        "ID15 (deg)"
    ])


    return workbook, worksheet


# ============================================================
# STORE ANGLE SAMPLE
# ============================================================

def store_trajectory_sample(
    worksheet,
    index,
    time_value,
    raw11,
    raw12,
    raw13,
    raw14,
    raw15
):

    # --------------------------------------------------------
    # RAW -> DEGREES
    # --------------------------------------------------------

    angle11 = raw_to_degrees(raw11)
    angle12 = raw_to_degrees(raw12)
    angle13 = raw_to_degrees(raw13)
    angle14 = raw_to_degrees(raw14)
    angle15 = raw_to_degrees(raw15)


    # --------------------------------------------------------
    # STORE DEGREES
    # --------------------------------------------------------

    worksheet.append([
        index,
        round(time_value, 3),
        round(angle11, 2),
        round(angle12, 2),
        round(angle13, 2),
        round(angle14, 2),
        round(angle15, 2)
    ])


    # --------------------------------------------------------
    # DISPLAY
    # --------------------------------------------------------

    print(
        f"TEACH | "
        f"Index={index} | "
        f"Time={time_value:.3f}s | "
        f"ID11={angle11:.2f}° | "
        f"ID12={angle12:.2f}° | "
        f"ID13={angle13:.2f}° | "
        f"ID14={angle14:.2f}° | "
        f"ID15={angle15:.2f}°"
    )


# ============================================================
# PARSE TRAJECTORY
# ============================================================

def parse_trajectory_line(
    line,
    worksheet
):

    line = line.strip()


    if not line.startswith("TRAJ,"):
        return False


    try:

        parts = line.split(",")


        if len(parts) != 8:

            print(
                "ERROR: Invalid TRAJ message:"
            )

            print(line)

            return False


        # ----------------------------------------------------
        # Read RAW trajectory
        # ----------------------------------------------------

        index = int(parts[1])

        time_value = float(parts[2])

        raw11 = int(parts[3])
        raw12 = int(parts[4])
        raw13 = int(parts[5])
        raw14 = int(parts[6])
        raw15 = int(parts[7])


        # ----------------------------------------------------
        # Convert and store
        # ----------------------------------------------------

        store_trajectory_sample(
            worksheet,
            index,
            time_value,
            raw11,
            raw12,
            raw13,
            raw14,
            raw15
        )


        return True


    except ValueError:

        print(
            "ERROR: Could not parse TRAJ:"
        )

        print(line)

        return False


# ============================================================
# SEND COMMAND
# ============================================================

def send_command(
    ser,
    command
):

    ser.write(
        (command + "\n").encode()
    )

    print(
        f">>> {command}"
    )


# ============================================================
# TEACH
# ============================================================

def teach():

    print()
    print("==========================================")
    print(" OPENMANIPULATOR-X")
    print(" TEACHING ANGLE STORAGE")
    print(" RAW -> DEGREES -> EXCEL")
    print("==========================================")
    print()


    # --------------------------------------------------------
    # Open Serial
    # --------------------------------------------------------

    try:

        ser = serial.Serial(
            COM_PORT,
            BAUD_RATE,
            timeout=0.1
        )

    except serial.SerialException as error:

        print()
        print(
            "ERROR: Could not open serial port."
        )

        print(error)

        return


    # --------------------------------------------------------
    # Allow serial connection to initialize
    # --------------------------------------------------------

    time.sleep(2)


    # --------------------------------------------------------
    # Clear old serial data
    # --------------------------------------------------------

    ser.reset_input_buffer()


    # --------------------------------------------------------
    # Create Excel
    # --------------------------------------------------------

    workbook, worksheet = create_excel()


    # --------------------------------------------------------
    # Start teaching
    # --------------------------------------------------------

    send_command(
        ser,
        "TRAJECTORY,START"
    )


    print()
    print("Teaching started.")
    print()
    print("Move the robot manually.")
    print()
    print("Press ENTER to stop teaching.")
    print()


    # --------------------------------------------------------
    # Non-blocking keyboard approach
    # --------------------------------------------------------

    import threading

    stop_event = threading.Event()


    def wait_for_enter():

        input()

        stop_event.set()


    keyboard_thread = threading.Thread(
        target=wait_for_enter,
        daemon=True
    )

    keyboard_thread.start()


    # --------------------------------------------------------
    # Receive TRAJ messages
    # --------------------------------------------------------

    try:

        while not stop_event.is_set():

            if ser.in_waiting:

                line = (
                    ser.readline()
                    .decode(
                        errors="ignore"
                    )
                    .strip()
                )


                if not line:
                    continue


                # --------------------------------------------
                # Display OpenCR messages
                # --------------------------------------------

                print(
                    f"<<< {line}"
                )


                # --------------------------------------------
                # TRAJ = actual teaching sample
                # --------------------------------------------

                if line.startswith("TRAJ,"):

                    parse_trajectory_line(
                        line,
                        worksheet
                    )


            else:

                time.sleep(0.005)


    except KeyboardInterrupt:

        print()
        print(
            "Teaching interrupted."
        )


    finally:

        # ----------------------------------------------------
        # Stop teaching
        # ----------------------------------------------------

        send_command(
            ser,
            "TRAJECTORY,END"
        )


        time.sleep(0.5)


        # ----------------------------------------------------
        # Save Excel
        # ----------------------------------------------------

        workbook.save(
            EXCEL_FILE
        )


        # ----------------------------------------------------
        # Close serial
        # ----------------------------------------------------

        ser.close()


        print()
        print("==========================================")
        print(" TEACHING FINISHED")
        print("==========================================")
        print()

        print(
            f"Saved file:"
        )

        print(
            os.path.abspath(
                EXCEL_FILE
            )
        )

        print()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    teach()