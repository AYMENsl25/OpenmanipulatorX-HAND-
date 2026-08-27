import serial
import time
import threading
import queue
import os

from openpyxl import Workbook, load_workbook


# ============================================================
# OPENMANIPULATOR-X
# PYTHON TEACH / REPLAY CONTROLLER
# ============================================================
#
# Python responsibilities:
#
#   - Send commands to OpenCR
#   - Receive taught trajectory
#   - Save trajectory to Excel
#   - Load trajectory from Excel
#   - Send trajectory from Excel to OpenCR
#
# Arduino responsibilities:
#
#   - Dynamixel communication
#   - Motor safety
#   - Position control
#   - Teaching
#   - Replay
#
# ============================================================


# ============================================================
# SERIAL SETTINGS
# ============================================================

SERIAL_PORT = "COM7"

# IMPORTANT:
# Change COM7 if your OpenCR appears on another COM port.

BAUDRATE = 115200

SERIAL_TIMEOUT = 0.1


# ============================================================
# FILE SETTINGS
# ============================================================

TRAJECTORY_FILE = "openmanipulator_trajectory.xlsx"


# ============================================================
# TEACHING SETTINGS
# ============================================================

TEACH_DURATION_SECONDS = 10.0

TEACH_SAMPLE_INTERVAL_SECONDS = 0.05


# ============================================================
# GLOBAL SERIAL OBJECT
# ============================================================

ser = None


# ============================================================
# SERIAL MESSAGE QUEUE
# ============================================================

serial_queue = queue.Queue()


# ============================================================
# PROGRAM STATE
# ============================================================

running = True


# ============================================================
# CURRENT TRAJECTORY
# ============================================================

trajectory = []


# ============================================================
# CONNECT TO OPENCR
# ============================================================

def connect():

    global ser

    print()
    print("=" * 60)
    print("OPENMANIPULATOR-X PYTHON CONTROLLER")
    print("=" * 60)

    print()
    print(f"Opening serial port: {SERIAL_PORT}")

    try:

        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=SERIAL_TIMEOUT
        )

        # Give OpenCR time to reset after opening serial.

        time.sleep(2.0)

        # Clear any old serial data.

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print(
            f"Connected successfully to {SERIAL_PORT}"
        )

        print()

    except serial.SerialException as error:

        print()
        print("ERROR: Could not open OpenCR serial port.")
        print(error)
        print()

        print(
            "Check:"
        )

        print(
            "1. OpenCR USB cable"
        )

        print(
            "2. Correct COM port"
        )

        print(
            "3. Arduino Serial Monitor is closed"
        )

        print(
            "4. Another Python program is not using the port"
        )

        raise


# ============================================================
# SERIAL READER THREAD
# ============================================================

def serial_reader():

    global running

    while running:

        try:

            if ser is None:

                time.sleep(0.01)

                continue

            if ser.in_waiting > 0:

                raw_line = ser.readline()

                if not raw_line:

                    continue

                line = raw_line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if line:

                    serial_queue.put(line)

            else:

                time.sleep(0.005)

        except serial.SerialException as error:

            print(
                f"\nSERIAL ERROR: {error}"
            )

            break

        except Exception as error:

            print(
                f"\nSERIAL READER ERROR: {error}"
            )

            break


# ============================================================
# SEND RAW COMMAND
# ============================================================

def send_command(command):

    if ser is None:

        print(
            "ERROR: OpenCR is not connected."
        )

        return False

    try:

        command = str(command)

        # Arduino accepts one command per line.

        message = command + "\n"

        ser.write(
            message.encode("ascii")
        )

        ser.flush()

        return True

    except serial.SerialException as error:

        print(
            f"SERIAL WRITE ERROR: {error}"
        )

        return False


# ============================================================
# GET QUEUED SERIAL MESSAGE
# ============================================================

def get_serial_message(timeout=0.1):

    try:

        return serial_queue.get(
            timeout=timeout
        )

    except queue.Empty:

        return None


# ============================================================
# DRAIN SERIAL QUEUE
# ============================================================

def drain_serial_queue():

    messages = []

    while True:

        try:

            messages.append(
                serial_queue.get_nowait()
            )

        except queue.Empty:

            break

    return messages


# ============================================================
# PARSE TRAJECTORY MESSAGE
# ============================================================

def parse_trajectory_message(line):

    # Expected:
    #
    # TRAJ,index,time,id11,id12,id13,id14,id15

    if not line.startswith("TRAJ,"):

        return None

    parts = line.split(",")

    if len(parts) != 8:

        print(
            f"WARNING: Invalid TRAJ message: {line}"
        )

        return None

    try:

        sample = {

            "index": int(parts[1]),

            "time": float(parts[2]),

            "id11": int(parts[3]),

            "id12": int(parts[4]),

            "id13": int(parts[5]),

            "id14": int(parts[6]),

            "id15": int(parts[7])
        }

        return sample

    except ValueError:

        print(
            f"WARNING: Could not parse TRAJ message: {line}"
        )

        return None


# ============================================================
# RECEIVE TEACHING TRAJECTORY
# ============================================================

def teach():

    global trajectory

    print()
    print("=" * 60)
    print("STARTING MANUAL TEACHING")
    print("=" * 60)

    print()
    print("Arduino will:")
    print("  1. Start a countdown")
    print("  2. Turn OFF torque")
    print("  3. Allow manual movement")
    print("  4. Record ID11-ID15")
    print("  5. Send samples to Python")
    print("  6. Synchronize goals")
    print("  7. Turn torque ON")

    print()

    # Remove old messages.

    drain_serial_queue()

    # Start teaching.

    send_command("T")

    trajectory = []

    teaching_started = False

    teaching_finished = False

    start_wait = time.time()

    timeout = (
        TEACH_DURATION_SECONDS
        + 20.0
    )

    while not teaching_finished:

        if (
            time.time() - start_wait
            > timeout
        ):

            print()
            print(
                "ERROR: Teaching timeout."
            )

            break

        line = get_serial_message(
            timeout=0.1
        )

        if line is None:

            continue

        # ----------------------------------------------------
        # TRAJECTORY SAMPLE
        # ----------------------------------------------------

        sample = parse_trajectory_message(
            line
        )

        if sample is not None:

            trajectory.append(
                sample
            )

            teaching_started = True

            if (
                len(trajectory) % 20
                == 0
            ):

                print(
                    f"\rReceived samples: "
                    f"{len(trajectory)}",
                    end=""
                )

            continue

        # ----------------------------------------------------
        # NORMAL ARDUINO MESSAGE
        # ----------------------------------------------------

        print(
            f"[ARDUINO] {line}"
        )

        if (
            "TEACH MODE FINISHED"
            in line
        ):

            teaching_finished = True

    print()

    # --------------------------------------------------------
    # REMOVE DUPLICATE SAMPLE INDEXES
    # --------------------------------------------------------

    unique = {}

    for sample in trajectory:

        unique[
            sample["index"]
        ] = sample

    trajectory = [
        unique[index]
        for index in sorted(unique)
    ]

    # --------------------------------------------------------
    # SAVE
    # --------------------------------------------------------

    print()

    print("=" * 60)

    print(
        f"TEACHING RECEIVED: {len(trajectory)} SAMPLES"
    )

    print("=" * 60)

    if len(trajectory) < 2:

        print(
            "WARNING: Not enough trajectory samples."
        )

        return

    save_trajectory_excel(
        trajectory
    )

    print_trajectory(
        trajectory
    )


# ============================================================
# SAVE TRAJECTORY TO EXCEL
# ============================================================

def save_trajectory_excel(
    data,
    filename=TRAJECTORY_FILE
):

    if not data:

        print(
            "ERROR: Trajectory is empty."
        )

        return False

    workbook = Workbook()

    sheet = workbook.active

    sheet.title = "Trajectory"

    # --------------------------------------------------------
    # HEADER
    # --------------------------------------------------------

    sheet.append(
        [
            "Index",
            "Time",
            "ID11",
            "ID12",
            "ID13",
            "ID14",
            "ID15"
        ]
    )

    # --------------------------------------------------------
    # DATA
    # --------------------------------------------------------

    for sample in data:

        sheet.append(
            [
                sample["index"],
                sample["time"],
                sample["id11"],
                sample["id12"],
                sample["id13"],
                sample["id14"],
                sample["id15"]
            ]
        )

    # --------------------------------------------------------
    # FORMATTING
    # --------------------------------------------------------

    sheet.freeze_panes = "A2"

    sheet.column_dimensions["A"].width = 12
    sheet.column_dimensions["B"].width = 12
    sheet.column_dimensions["C"].width = 12
    sheet.column_dimensions["D"].width = 12
    sheet.column_dimensions["E"].width = 12
    sheet.column_dimensions["F"].width = 12
    sheet.column_dimensions["G"].width = 12

    # --------------------------------------------------------
    # SAVE
    # --------------------------------------------------------

    try:

        workbook.save(
            filename
        )

    except Exception as error:

        print(
            f"ERROR saving Excel: {error}"
        )

        return False

    print()
    print("=" * 60)
    print("TRAJECTORY SAVED")
    print("=" * 60)

    print(
        f"File: {os.path.abspath(filename)}"
    )

    print(
        f"Samples: {len(data)}"
    )

    print()

    return True


# ============================================================
# LOAD TRAJECTORY FROM EXCEL
# ============================================================

def load_trajectory_excel(
    filename=TRAJECTORY_FILE
):

    if not os.path.exists(filename):

        print()
        print(
            f"ERROR: File not found: {filename}"
        )

        return []

    try:

        workbook = load_workbook(
            filename,
            data_only=True
        )

    except Exception as error:

        print(
            f"ERROR opening Excel file: {error}"
        )

        return []

    if "Trajectory" not in workbook.sheetnames:

        print(
            "ERROR: Excel file does not contain "
            "'Trajectory' sheet."
        )

        return []

    sheet = workbook["Trajectory"]

    data = []

    # Skip header.

    for row in sheet.iter_rows(
        min_row=2,
        values_only=True
    ):

        if not row:

            continue

        if row[0] is None:

            continue

        try:

            sample = {

                "index": int(row[0]),

                "time": float(row[1]),

                "id11": int(row[2]),

                "id12": int(row[3]),

                "id13": int(row[4]),

                "id14": int(row[5]),

                "id15": int(row[6])
            }

            data.append(
                sample
            )

        except (
            ValueError,
            TypeError,
            IndexError
        ):

            print(
                f"WARNING: Invalid Excel row: {row}"
            )

    print()
    print("=" * 60)
    print("TRAJECTORY LOADED")
    print("=" * 60)

    print(
        f"File: {os.path.abspath(filename)}"
    )

    print(
        f"Samples: {len(data)}"
    )

    return data


# ============================================================
# PRINT TRAJECTORY
# ============================================================

def print_trajectory(
    data,
    maximum=20
):

    if not data:

        print(
            "Trajectory is empty."
        )

        return

    print()

    print(
        "Index       Time       ID11       ID12"
        "       ID13       ID14       ID15"
    )

    print(
        "-" * 80
    )

    for sample in data[:maximum]:

        print(
            f"{sample['index']:5d} "
            f"{sample['time']:10.3f} "
            f"{sample['id11']:10d} "
            f"{sample['id12']:10d} "
            f"{sample['id13']:10d} "
            f"{sample['id14']:10d} "
            f"{sample['id15']:10d}"
        )

    if len(data) > maximum:

        print()

        print(
            f"... {len(data) - maximum} additional samples"
        )


# ============================================================
# VALIDATE TRAJECTORY IN PYTHON
# ============================================================

def validate_trajectory(
    data
):

    if not data:

        print(
            "ERROR: Empty trajectory."
        )

        return False

    # --------------------------------------------------------
    # Expected raw ranges
    #
    # Extended position range of Dynamixel.
    # --------------------------------------------------------

    extended_min = -1048575

    extended_max = 1048575

    # --------------------------------------------------------
    # Gripper normalized range
    # --------------------------------------------------------

    gripper_open = 90.0

    gripper_closed = 234.0

    counts_per_degree = (
        4096.0 / 360.0
    )

    gripper_open_raw = round(
        gripper_open *
        counts_per_degree
    )

    gripper_closed_raw = round(
        gripper_closed *
        counts_per_degree
    )

    for index, sample in enumerate(data):

        # ----------------------------------------------------
        # Check arm extended raw range
        # ----------------------------------------------------

        for key in (
            "id11",
            "id12",
            "id13",
            "id14"
        ):

            value = sample[key]

            if (
                value < extended_min
                or value > extended_max
            ):

                print(
                    f"ERROR: Sample {index}, "
                    f"{key} outside extended range."
                )

                return False

        # ----------------------------------------------------
        # Check gripper raw normalized angle
        # ----------------------------------------------------

        raw15 = sample["id15"]

        normalized15 = (
            raw15 % 4096
        )

        if normalized15 < 0:

            normalized15 += 4096

        if (
            normalized15
            < gripper_open_raw - 46
            or
            normalized15
            > gripper_closed_raw + 46
        ):

            print(
                f"WARNING: Sample {index} "
                f"has ID15 outside 90-234 degrees."
            )

            print(
                f"ID15 raw = {raw15}"
            )

            print(
                "Trajectory validation failed."
            )

            return False

    print(
        "Trajectory validation: PASS"
    )

    return True


# ============================================================
# SEND EXCEL TRAJECTORY TO ARDUINO
# ============================================================

def send_trajectory_to_arduino(
    data
):

    if not data:

        print(
            "ERROR: No trajectory to send."
        )

        return False

    if not validate_trajectory(
        data
    ):

        print(
            "Trajectory was not sent."
        )

        return False

    print()
    print("=" * 60)
    print("SENDING TRAJECTORY TO OPENCR")
    print("=" * 60)

    # --------------------------------------------------------
    # Clear old Arduino trajectory
    # --------------------------------------------------------

    drain_serial_queue()

    # --------------------------------------------------------
    # Start transmission
    # --------------------------------------------------------

    if not send_command(
        "TRAJECTORY,START"
    ):

        return False

    time.sleep(0.2)

    # --------------------------------------------------------
    # Send samples
    # --------------------------------------------------------

    total = len(data)

    for count, sample in enumerate(data):

        line = (
            f"TRAJ,"
            f"{sample['index']},"
            f"{sample['time']:.3f},"
            f"{sample['id11']},"
            f"{sample['id12']},"
            f"{sample['id13']},"
            f"{sample['id14']},"
            f"{sample['id15']}"
        )

        if not send_command(
            line
        ):

            return False

        # ----------------------------------------------------
        # Wait for acknowledgement.
        #
        # Arduino sends:
        #
        # TRAJ_OK,index
        #
        # ----------------------------------------------------

        acknowledgement_received = False

        wait_start = time.time()

        while (
            time.time() - wait_start
            < 1.0
        ):

            response = get_serial_message(
                timeout=0.05
            )

            if response is None:

                continue

            if response.startswith(
                "TRAJ_OK,"
            ):

                acknowledgement_received = True

                break

            print(
                f"[ARDUINO] {response}"
            )

        if not acknowledgement_received:

            print()
            print(
                f"ERROR: No acknowledgement "
                f"for sample {count}."
            )

            return False

        if (
            count % 20 == 0
            or count == total - 1
        ):

            print(
                f"\rSent "
                f"{count + 1}/{total} samples",
                end=""
            )

    print()

    # --------------------------------------------------------
    # End transmission
    # --------------------------------------------------------

    send_command(
        "TRAJECTORY,END"
    )

    # --------------------------------------------------------
    # Wait for confirmation
    # --------------------------------------------------------

    start_time = time.time()

    while (
        time.time() - start_time
        < 5.0
    ):

        response = get_serial_message(
            timeout=0.1
        )

        if response is None:

            continue

        print(
            f"[ARDUINO] {response}"
        )

        if (
            "TRAJECTORY RECEIVED"
            in response
        ):

            print()
            print(
                "Trajectory successfully transferred "
                "to OpenCR."
            )

            return True

    print()
    print(
        "WARNING: Did not receive final "
        "trajectory confirmation."
    )

    return False


# ============================================================
# REPLAY EXCEL TRAJECTORY
# ============================================================

def replay_excel_trajectory(
    forward=True
):

    global trajectory

    # --------------------------------------------------------
    # Load trajectory
    # --------------------------------------------------------

    trajectory = load_trajectory_excel()

    if not trajectory:

        return

    # --------------------------------------------------------
    # Send trajectory
    # --------------------------------------------------------

    success = send_trajectory_to_arduino(
        trajectory
    )

    if not success:

        return

    # --------------------------------------------------------
    # Tell Arduino to replay.
    #
    # Y = forward
    # U = backward
    # --------------------------------------------------------

    if forward:

        print()
        print(
            "Starting FORWARD replay..."
        )

        send_command(
            "Y"
        )

    else:

        print()
        print(
            "Starting BACKWARD replay..."
        )

        send_command(
            "U"
        )


# ============================================================
# PRINT HELP
# ============================================================

def print_help():

    print()
    print("=" * 65)
    print("OPENMANIPULATOR-X PYTHON CONTROLLER")
    print("=" * 65)

    print()

    print("ARM JOINTS")
    print("  Q / A       ID11 +1 / -1 degree")
    print("  W / Z       ID12 +1 / -1 degree")
    print("  E / D       ID13 +1 / -1 degree")
    print("  R / F       ID14 +1 / -1 degree")

    print()

    print("ARM LIMITS")
    print("  1           ID11 -> 261 deg")
    print("  2           ID11 -> 81 deg")
    print("  3           ID12 -> 83 deg")
    print("  4           ID12 -> 240 deg")
    print("  5           ID13 -> 90 deg")
    print("  6           ID13 -> 240 deg")
    print("  7           ID14 -> 125 deg")
    print("  8           ID14 -> 258 deg")

    print()

    print("GRIPPER")
    print("  O           Fully open")
    print("  K           Fully close")
    print("  [           Open 5 degrees")
    print("  ]           Close 5 degrees")

    print()

    print("ROBOT")
    print("  H           Home")
    print("  P           Show positions")
    print("  X           Torque OFF")
    print("  B           Torque ON")

    print()

    print("TEACH / REPLAY")
    print("  T           Manual teaching")
    print("  Y           Replay forward")
    print("  U           Replay backward")
    print("  C           Clear trajectory")

    print()

    print("EXCEL")
    print("  save        Save current Python trajectory")
    print("  load        Load trajectory from Excel")
    print("  send        Send Excel trajectory to OpenCR")
    print("  excel_y     Load + send + replay forward")
    print("  excel_u     Load + send + replay backward")
    print("  show        Show current trajectory")

    print()

    print("  help        Show this menu")
    print("  exit        Exit Python")

    print()
    print("=" * 65)
    print()


# ============================================================
# MAIN COMMAND LOOP
# ============================================================

def main():

    global running
    global trajectory

    # --------------------------------------------------------
    # Connect
    # --------------------------------------------------------

    connect()

    # --------------------------------------------------------
    # Start serial reader
    # --------------------------------------------------------

    reader_thread = threading.Thread(
        target=serial_reader,
        daemon=True
    )

    reader_thread.start()

    # --------------------------------------------------------
    # Give Arduino time
    # --------------------------------------------------------

    time.sleep(0.5)

    # --------------------------------------------------------
    # Print help
    # --------------------------------------------------------

    print_help()

    # --------------------------------------------------------
    # Main loop
    # --------------------------------------------------------

    while running:

        # ----------------------------------------------------
        # Print asynchronous Arduino messages.
        # ----------------------------------------------------

        while True:

            try:

                message = (
                    serial_queue.get_nowait()
                )

                print(
                    f"[ARDUINO] {message}"
                )

            except queue.Empty:

                break

        # ----------------------------------------------------
        # User command
        # ----------------------------------------------------

        try:

            command = input(
                "\nCommand > "
            ).strip()

        except KeyboardInterrupt:

            break

        except EOFError:

            break

        if not command:

            continue

        # ====================================================
        # EXIT
        # ====================================================

        if command.lower() == "exit":

            break

        # ====================================================
        # HELP
        # ====================================================

        elif command.lower() == "help":

            print_help()

        # ====================================================
        # TEACH
        # ====================================================

        elif command.lower() == "t":

            teach()

        # ====================================================
        # SAVE
        # ====================================================

        elif command.lower() == "save":

            save_trajectory_excel(
                trajectory
            )

        # ====================================================
        # LOAD
        # ====================================================

        elif command.lower() == "load":

            trajectory = (
                load_trajectory_excel()
            )

        # ====================================================
        # SHOW
        # ====================================================

        elif command.lower() == "show":

            print_trajectory(
                trajectory
            )

        # ====================================================
        # SEND
        # ====================================================

        elif command.lower() == "send":

            if not trajectory:

                trajectory = (
                    load_trajectory_excel()
                )

            if trajectory:

                send_trajectory_to_arduino(
                    trajectory
                )

        # ====================================================
        # EXCEL FORWARD
        # ====================================================

        elif command.lower() == "excel_y":

            replay_excel_trajectory(
                forward=True
            )

        # ====================================================
        # EXCEL BACKWARD
        # ====================================================

        elif command.lower() == "excel_u":

            replay_excel_trajectory(
                forward=False
            )

        # ====================================================
        # NORMAL ARDUINO COMMAND
        # ====================================================

        elif len(command) == 1:

            send_command(
                command
            )

        # ====================================================
        # UNKNOWN
        # ====================================================

        else:

            print(
                "Unknown command."
            )

            print(
                "Type 'help' for commands."
            )

    # --------------------------------------------------------
    # Shutdown
    # --------------------------------------------------------

    running = False

    time.sleep(0.2)

    if ser is not None:

        try:

            ser.close()

        except Exception:

            pass

    print()
    print(
        "Python controller stopped."
    )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()