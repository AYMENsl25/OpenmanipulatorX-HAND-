"""Command-line calibration and test client for the OpenCR sketch."""

from __future__ import annotations

import argparse
import msvcrt
import time
import serial

from kinematics_calibration import (
    Joints,
    calibration_ready,
    forward_kinematics,
    inverse_kinematics,
    joints_to_raw,
    raw_to_joints,
)


class OpenCR:
    def __init__(self, port: str):
        self.serial = serial.Serial(port, 115200, timeout=3, write_timeout=3)
        time.sleep(2)
        self.serial.reset_input_buffer()

    def command(self, text: str) -> str:
        self.serial.write((text.strip() + "\n").encode("ascii"))
        self.serial.flush()
        response = self.serial.readline().decode("ascii", errors="replace").strip()
        if not response:
            raise RuntimeError("OpenCR did not respond")
        if response.startswith("ERROR"):
            raise RuntimeError(response)
        return response

    def read_raw(self) -> dict[int, int]:
        parts = self.command("READ_RAW").split(",")
        if len(parts) != 6 or parts[0] != "RAW":
            raise RuntimeError(f"Malformed response: {parts}")
        return dict(zip((11, 12, 13, 14, 15), map(int, parts[1:]), strict=True))


def show_reading(raw: dict[int, int]) -> None:
    print("ID  RAW   MOTOR_DEG")
    for motor_id in (11, 12, 13, 14, 15):
        print(f"{motor_id:2d}  {raw[motor_id]:4d}  {raw[motor_id] * 360.0 / 4096:9.3f}")
    try:
        joints = raw_to_joints(raw)
    except ValueError as exc:
        print(f"Joint/FK unavailable: {exc}")
        return
    pose = forward_kinematics(joints)
    print(f"q={joints.as_tuple()}")
    print(f"FK actual: X={pose.x:.3f} Y={pose.y:.3f} Z={pose.z:.3f} pitch={pose.tool_pitch_deg:.3f}")


def test_ik(values: list[float]) -> None:
    result = inverse_kinematics(*values)
    print(f"success={result.success} reason={result.reason.value} message={result.message}")
    if not result.success:
        return
    assert result.joints and result.reconstructed
    print(f"IK q={result.joints.as_tuple()}")
    print(f"FK check: X={result.reconstructed.x:.6f} Y={result.reconstructed.y:.6f} Z={result.reconstructed.z:.6f}")
    print(f"error: dx={result.dx:.9f} dy={result.dy:.9f} dz={result.dz:.9f} total={result.error_mm:.9f} mm")
    if calibration_ready():
        print(f"automatic-safe RAW targets={joints_to_raw(result.joints)}")
    else:
        print("RAW targets blocked: measured J2-J4 calibration is incomplete")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="OpenCR COM port; omit for offline IK")
    parser.add_argument("--test-ik", nargs=4, type=float, metavar=("X", "Y", "Z", "PITCH"))
    args = parser.parse_args()
    if args.test_ik:
        test_ik(args.test_ik)
        return
    if not args.port:
        parser.error("use --port COMx or --test-ik X Y Z PITCH")
    board = OpenCR(args.port)
    print(board.command("PING"))
    print("\nPress one key (no Enter needed):")
    print("  R = read motors")
    print("  H = capture HOME candidate (torque must be off)")
    print("  O = torque off")
    print("  N = torque on / hold current position")
    print("  Q = quit\n")
    while True:
        print("[R/H/O/N/Q] > ", end="", flush=True)
        key = msvcrt.getwch().upper()
        print(key)
        try:
            if key == "R":
                show_reading(board.read_raw())
            elif key == "H":
                print(board.command("CAPTURE_HOME"))
            elif key == "O":
                print(board.command("TORQUE_OFF"))
            elif key == "N":
                print(board.command("TORQUE_ON"))
            elif key == "Q":
                break
            else:
                print("Unknown key. Use R, H, O, N or Q.")
        except (RuntimeError, serial.SerialException) as exc:
            print(f"COMMAND FAILED: {exc}")
            print("The program is still running; correct the hardware issue and press R again.")


if __name__ == "__main__":
    main()
