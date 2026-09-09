"""Serial protocol client for the OpenCR firmware."""

from __future__ import annotations

import time
import math
from dataclasses import dataclass
from threading import Event

import serial
from serial.tools import list_ports

import config
from kinematics import (
    MotorAngles,
    fk_to_motor_angles,
    forward_kinematics,
    inverse_kinematics,
    normalize_angle,
    validate_motor_angles,
)
from trajectory import TrajectoryPoint


class ControllerError(RuntimeError):
    pass


@dataclass
class ControllerStatus:
    connected: bool = False
    torque_on: bool = False


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


class OpenCRController:
    def __init__(self) -> None:
        self._serial: serial.Serial | None = None
        self.status = ControllerStatus()

    def connect(self, port: str) -> None:
        if self._serial and self._serial.is_open:
            return
        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=config.SERIAL_BAUDRATE,
                timeout=config.SERIAL_TIMEOUT_SECONDS,
                write_timeout=config.SERIAL_TIMEOUT_SECONDS,
            )
            time.sleep(2.0)
            self._serial.reset_input_buffer()
            response = self._command("PING")
            if response != "OK,PONG":
                raise ControllerError(f"Unexpected PING response: {response}")
            self.status.connected = True
            self.status.torque_on = self.get_torque_state()
        except serial.SerialException as exc:
            self._serial = None
            raise ControllerError(f"Invalid COM port or serial failure: {exc}") from exc

    def disconnect(self) -> None:
        if self._serial:
            self._serial.close()
        self._serial = None
        self.status = ControllerStatus()

    def _require_serial(self) -> serial.Serial:
        if not self._serial or not self._serial.is_open:
            raise ControllerError("Serial disconnected")
        return self._serial

    def _read_line(self, timeout: float | None = None) -> str:
        port = self._require_serial()
        old_timeout = port.timeout
        if timeout is not None:
            port.timeout = timeout
        try:
            raw = port.readline()
        finally:
            port.timeout = old_timeout
        if not raw:
            raise ControllerError("OpenCR not responding")
        try:
            return raw.decode("ascii", errors="replace").strip()
        except UnicodeDecodeError as exc:
            raise ControllerError("Malformed response from OpenCR") from exc

    def _command(self, message: str, timeout: float | None = None) -> str:
        port = self._require_serial()
        try:
            port.write((message.strip() + "\n").encode("ascii"))
            port.flush()
        except serial.SerialException as exc:
            raise ControllerError(f"Serial write failed: {exc}") from exc

        while True:
            response = self._read_line(timeout)
            if not response:
                continue
            if response.startswith("ERROR"):
                raise ControllerError(response)
            return response

    def get_torque_state(self) -> bool:
        response = self._command("TORQUE_STATUS")
        if response == "TORQUE:ON":
            return True
        if response == "TORQUE:OFF":
            return False
        raise ControllerError(f"Malformed torque response: {response}")

    def torque_on(self) -> None:
        response = self._command("TORQUE_ON")
        if response != "TORQUE:ON":
            raise ControllerError(f"Malformed torque response: {response}")
        self.status.torque_on = True

    def torque_off(self) -> None:
        response = self._command("TORQUE_OFF")
        if response != "TORQUE:OFF":
            raise ControllerError(f"Malformed torque response: {response}")
        self.status.torque_on = False

    def read_motor_angles(self) -> MotorAngles:
        response = self._command("READ_ANGLES")
        parts = response.split(",")
        if len(parts) != 5 or parts[0] != "ANGLES":
            raise ControllerError(f"Malformed angle response: {response}")
        try:
            return MotorAngles(*(float(value) for value in parts[1:5]))
        except ValueError as exc:
            raise ControllerError(f"Malformed numeric angle response: {response}") from exc

    def move_motor_angles(self, motors: MotorAngles) -> None:
        if not self.status.torque_on:
            raise ControllerError("Cannot move: Torque is OFF")
        validate_motor_angles(motors)
        command = "MOVE_MOTORS,{:.3f},{:.3f},{:.3f},{:.3f}".format(*motors.as_tuple())
        response = self._command(command)
        if response != "MOVING":
            raise ControllerError(f"Expected MOVING, received: {response}")
        done = self._read_line(config.MOVE_TIMEOUT_SECONDS)
        if done != "DONE":
            raise ControllerError(f"Movement failed or malformed response: {done}")
        self._verify_motor_targets(motors)

    def _verify_motor_targets(self, target: MotorAngles) -> None:
        actual = self.read_motor_angles()
        errors = tuple(
            abs(normalize_angle(actual_angle - target_angle))
            for actual_angle, target_angle in zip(actual.as_tuple(), target.as_tuple(), strict=True)
        )
        largest_error = max(errors)
        if largest_error <= config.POST_MOVE_TOLERANCE_DEGREES:
            return
        index = errors.index(largest_error)
        motor_id = (11, 12, 13, 14)[index]
        raise ControllerError(
            f"Motor target not reached: ID{motor_id} commanded={target.as_tuple()[index]:.3f} deg, "
            f"actual={actual.as_tuple()[index]:.3f} deg, error={largest_error:.3f} deg"
        )

    def _move_named_pose(self, command: str, target: MotorAngles, label: str) -> None:
        if not self.status.torque_on:
            raise ControllerError("Cannot move: Torque is OFF")
        response = self._command(command)
        if response != "MOVING":
            raise ControllerError(f"Expected MOVING, received: {response}")
        done = self._read_line(config.MOVE_TIMEOUT_SECONDS)
        if done != "DONE":
            raise ControllerError(f"{label} movement failed or malformed response: {done}")
        self._verify_motor_targets(target)

    def move_rest(self) -> None:
        self._move_named_pose("REST", MotorAngles(
            config.HOME_ID11,
            config.HOME_ID12,
            config.HOME_ID13,
            config.HOME_ID14,
        ), "REST")

    def move_work(self) -> None:
        self._move_named_pose("WORK", MotorAngles(*config.WORK_MOTOR_DEGREES), "WORK")

    def move_home(self) -> None:
        """Compatibility alias: the old HOME command is now named REST."""
        self.move_rest()

    def start_teaching(self) -> None:
        self.torque_off()
        response = self._command("START_TEACH")
        if response != "TEACHING":
            raise ControllerError(f"Unexpected teaching response: {response}")

    def stop_teaching(self) -> list[TrajectoryPoint]:
        port = self._require_serial()
        try:
            port.write(b"STOP_TEACH\n")
            port.flush()
        except serial.SerialException as exc:
            raise ControllerError(f"Serial write failed: {exc}") from exc

        points: list[TrajectoryPoint] = []
        while True:
            response = self._read_line(config.MOVE_TIMEOUT_SECONDS)
            if response.startswith("TRAJ,"):
                parts = response.split(",")
                if len(parts) != 9:
                    raise ControllerError(f"Malformed trajectory point: {response}")
                try:
                    time_s = float(parts[1])
                    joints = tuple(float(value) for value in parts[2:6])
                    xyz = tuple(float(value) for value in parts[6:9])
                except ValueError as exc:
                    raise ControllerError(f"Malformed trajectory number: {response}") from exc
                from kinematics import JointAngles, XYZ
                joint_degrees = JointAngles(*(math.degrees(value) for value in joints))
                motors = fk_to_motor_angles(joint_degrees)
                # Python owns the calibrated FK.  Recompute XYZ here instead of
                # trusting values produced by an older firmware equation.
                points.append(TrajectoryPoint(time_s, motors, forward_kinematics(motors)))
            elif response.startswith("TRAJ_END,"):
                return points
            elif response.startswith("ERROR"):
                raise ControllerError(response)

    def move_xyz_point(self, point, reference=None) -> None:
        result = inverse_kinematics(point.xyz.x, point.xyz.y, point.xyz.z, reference=reference)
        validate_motor_angles(result.motor_angles)
        self.move_motor_angles(result.motor_angles)

    def send_xyz_trajectory(self, points: list[tuple[float, float, float, float]]) -> None:
        if not self.status.torque_on:
            raise ControllerError("Cannot play: Torque is OFF")
        port = self._require_serial()
        for time_s, x, y, z in points:
            message = f"XYZ,{time_s:.3f},{x:.3f},{y:.3f},{z:.3f}\n"
            try:
                port.write(message.encode("ascii"))
            except serial.SerialException as exc:
                raise ControllerError(f"Serial write failed: {exc}") from exc
        port.write(b"XYZ_END\n")
        port.flush()
        response = self._read_line(config.MOVE_TIMEOUT_SECONDS)
        if response != "PLAYING":
            raise ControllerError(f"Playback was not accepted: {response}")

    def stop(self) -> None:
        # Write-only emergency request: the playback worker owns the serial
        # reader and will receive STOPPED. Reading here would race that worker.
        port = self._require_serial()
        try:
            port.write(b"STOP\n")
            port.flush()
        except serial.SerialException as exc:
            raise ControllerError(f"STOP write failed: {exc}") from exc

    def play_xyz_trajectory(self, points, speed_scale: float, stop_event: Event) -> None:
        """Replay XYZ points only; compute fresh IK targets for every point."""
        reference = None
        if not points:
            raise ControllerError("Cannot play an empty XYZ trajectory")
        playback_started = time.monotonic()
        first_time = points[0].time_s
        for point in points:
            if stop_event.is_set():
                return
            scheduled = playback_started + max(0.0, point.time_s - first_time) / speed_scale
            while not stop_event.is_set() and time.monotonic() < scheduled:
                time.sleep(min(0.02, scheduled - time.monotonic()))
            if stop_event.is_set():
                return
            result = inverse_kinematics(point.xyz.x, point.xyz.y, point.xyz.z, reference=reference)
            reference = result.joint_angles
            validate_motor_angles(result.motor_angles)
            self.move_motor_angles(result.motor_angles)
