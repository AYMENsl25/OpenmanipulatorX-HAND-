"""Serial protocol client for the OpenCR firmware."""

from __future__ import annotations

import time
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from threading import Event, RLock

import serial
from serial.tools import list_ports

import config
from experiment_frame import forward_kinematics_physical, inverse_kinematics_physical
from kinematics import (
    JointAngles,
    MotorAngles,
    XYZ,
    fk_to_motor_angles,
    forward_kinematics,
    motor_to_fk_angles,
    normalize_angle,
    raw_to_degrees,
    validate_motor_angles,
)
from trajectory import TrajectoryPoint


class ControllerError(RuntimeError):
    pass


@dataclass
class ControllerStatus:
    connected: bool = False
    torque_on: bool = False


@dataclass(frozen=True)
class RobotState:
    timestamp_utc: str
    raw_positions: tuple[int, int, int, int] | None
    motors: MotorAngles
    joints: JointAngles
    physical_xyz: XYZ
    internal_xyz: XYZ


@dataclass(frozen=True)
class MotorTelemetry:
    motor_id: int
    position_raw: int
    velocity_raw: int
    current_raw: int
    pwm_raw: int
    voltage_raw: int
    temperature_c: int
    hardware_error: int
    moving: int
    moving_status: int

    @property
    def velocity_rpm(self) -> float:
        # XM430 Present Velocity unit from the ROBOTIS control table.
        return self.velocity_raw * 0.229

    @property
    def current_ma(self) -> float:
        # XM430 Present Current unit from the ROBOTIS control table.
        return self.current_raw * 2.69

    @property
    def pwm_percent(self) -> float:
        # Present PWM is signed and 885 represents 100 percent.
        return self.pwm_raw * 100.0 / 885.0

    @property
    def voltage_v(self) -> float:
        return self.voltage_raw * 0.1


@dataclass(frozen=True)
class TelemetrySnapshot:
    timestamp_utc: str
    state: RobotState
    motors: tuple[MotorTelemetry, MotorTelemetry, MotorTelemetry, MotorTelemetry]


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


class OpenCRController:
    def __init__(self) -> None:
        self._serial: serial.Serial | None = None
        # One request/response transaction owns the serial stream at a time.
        # The lock is re-entrant because move verification performs READ_ANGLES.
        self._io_lock = RLock()
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
            self.verify_firmware_joint_limits()
            self.verify_firmware_capabilities()
            self.status.connected = True
            self.status.torque_on = self.get_torque_state()
        except (serial.SerialException, ControllerError) as exc:
            if self._serial:
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass
            self._serial = None
            self.status = ControllerStatus()
            if isinstance(exc, ControllerError):
                raise
            raise ControllerError(f"Invalid COM port or serial failure: {exc}") from exc

    def disconnect(self) -> None:
        with self._io_lock:
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
        with self._io_lock:
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

    def read_firmware_joint_limits(
        self,
    ) -> tuple[tuple[tuple[float, float], ...], float]:
        try:
            response = self._command("JOINT_LIMITS")
        except ControllerError as exc:
            if "UNKNOWN_COMMAND" in str(exc):
                raise ControllerError(
                    "OpenCR firmware is outdated and cannot report its joint limits. "
                    "Upload the current OpenManipulatorXYZController.ino, then reconnect."
                ) from exc
            raise
        parts = response.split(",")
        if len(parts) != 11 or parts[0] != "JOINT_LIMITS" or parts[9] != "RAW_TOLERANCE":
            raise ControllerError(
                "OpenCR joint-limit protocol is outdated. Upload the current "
                f"OpenManipulatorXYZController.ino. Received: {response}"
            )
        try:
            values = tuple(float(value) for value in parts[1:9])
            raw_tolerance = float(parts[10])
        except ValueError as exc:
            raise ControllerError(f"Malformed numeric joint-limit response: {response}") from exc
        limits = tuple((values[index], values[index + 1]) for index in range(0, 8, 2))
        return limits, raw_tolerance

    def verify_firmware_joint_limits(self) -> None:
        actual, raw_tolerance = self.read_firmware_joint_limits()
        expected = tuple(
            (config.FK_JOINT_LIMITS[name].minimum, config.FK_JOINT_LIMITS[name].maximum)
            for name in ("theta1", "theta2", "theta3", "theta4")
        )
        mismatches = [
            f"q{index}: OpenCR={found[0]:.1f}..{found[1]:.1f}, "
            f"Python={wanted[0]:.1f}..{wanted[1]:.1f}"
            for index, (found, wanted) in enumerate(zip(actual, expected, strict=True), start=1)
            if max(abs(found[0] - wanted[0]), abs(found[1] - wanted[1])) > 0.01
        ]
        if mismatches:
            raise ControllerError(
                "OpenCR/Python joint-limit mismatch. Upload the current firmware: "
                + "; ".join(mismatches)
            )
        if abs(raw_tolerance - config.JOINT_LIMIT_RAW_TOLERANCE_DEGREES) > 0.002:
            raise ControllerError(
                "OpenCR/Python encoder limit tolerance mismatch: "
                f"OpenCR={raw_tolerance:.3f} deg, "
                f"Python={config.JOINT_LIMIT_RAW_TOLERANCE_DEGREES:.3f} deg"
            )

    def verify_firmware_capabilities(self) -> None:
        try:
            response = self._command("CAPABILITIES")
        except ControllerError as exc:
            if "UNKNOWN_COMMAND" in str(exc):
                raise ControllerError(
                    "OpenCR firmware is the stable version without repeatability telemetry. "
                    "Upload the OpenManipulatorXYZController.ino from the new "
                    "openmanipulator_repeatability_project, then reconnect."
                ) from exc
            raise
        capabilities = set(response.split(",")[1:]) if response.startswith("CAPABILITIES,") else set()
        if "TELEMETRY_V1" not in capabilities:
            raise ControllerError(
                f"OpenCR firmware does not advertise TELEMETRY_V1. Received: {response}"
            )

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

    def _read_motor_angles_once(self) -> MotorAngles:
        response = self._command("READ_ANGLES")
        parts = response.split(",")
        if len(parts) != 5 or parts[0] != "ANGLES":
            raise ControllerError(f"Malformed angle response: {response}")
        try:
            return MotorAngles(*(float(value) for value in parts[1:5]))
        except ValueError as exc:
            raise ControllerError(f"Malformed numeric angle response: {response}") from exc

    def read_motor_angles(self) -> MotorAngles:
        """Read all four encoders, retrying transient read failures safely."""
        last_error: ControllerError | None = None
        for attempt in range(1, config.ANGLE_READ_ATTEMPTS + 1):
            try:
                return self._read_motor_angles_once()
            except ControllerError as exc:
                last_error = exc
                if attempt < config.ANGLE_READ_ATTEMPTS:
                    time.sleep(config.ANGLE_RETRY_DELAY_SECONDS)
        raise ControllerError(
            f"READ_ANGLES failed after {config.ANGLE_READ_ATTEMPTS} attempts: {last_error}"
        ) from last_error

    def read_robot_state(self) -> RobotState:
        """Read all encoders once and derive joints plus both coordinate frames."""
        raw_positions = None
        motors = None
        firmware_joints = None
        last_error: ControllerError | None = None
        for attempt in range(1, config.ANGLE_READ_ATTEMPTS + 1):
            try:
                response = self._command("READ_STATE")
                parts = response.split(",")
                if len(parts) != 13 or parts[0] != "STATE":
                    raise ControllerError(f"Malformed state response: {response}")
                raw_positions = tuple(int(value) for value in parts[1:5])
                motors = MotorAngles(*(float(value) for value in parts[5:9]))
                firmware_joints = JointAngles(*(float(value) for value in parts[9:13]))
                break
            except (ControllerError, ValueError) as exc:
                last_error = ControllerError(str(exc))
                if "UNKNOWN_COMMAND" in str(exc):
                    # Backward-compatible path for the previously uploaded sketch.
                    motors = self.read_motor_angles()
                    break
                if attempt < config.ANGLE_READ_ATTEMPTS:
                    time.sleep(config.ANGLE_RETRY_DELAY_SECONDS)
        if motors is None:
            raise ControllerError(
                f"READ_STATE failed after {config.ANGLE_READ_ATTEMPTS} attempts: {last_error}"
            ) from last_error
        joints = motor_to_fk_angles(motors)
        if firmware_joints is not None:
            mismatch = max(
                abs(a - b)
                for a, b in zip(joints.as_tuple(), firmware_joints.as_tuple(), strict=True)
            )
            if mismatch > 0.25:
                raise ControllerError(
                    f"OpenCR/Python calibrated-joint mismatch: maximum {mismatch:.3f} deg"
                )
        return RobotState(
            timestamp_utc=datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            raw_positions=raw_positions,
            motors=motors,
            joints=joints,
            physical_xyz=forward_kinematics_physical(motors),
            internal_xyz=forward_kinematics(motors),
        )

    def read_telemetry(self) -> TelemetrySnapshot:
        """Read one coherent diagnostic snapshot for all four arm motors."""
        response = self._command("READ_TELEMETRY")
        parts = response.split(",")
        values_per_motor = 10
        expected_parts = 1 + 4 * values_per_motor
        if len(parts) != expected_parts or parts[0] != "TELEMETRY_V1":
            raise ControllerError(f"Malformed telemetry response: {response}")

        readings: list[MotorTelemetry] = []
        try:
            for offset in range(1, expected_parts, values_per_motor):
                values = tuple(int(value) for value in parts[offset : offset + values_per_motor])
                readings.append(MotorTelemetry(*values))
        except (TypeError, ValueError) as exc:
            raise ControllerError(f"Malformed numeric telemetry response: {response}") from exc

        expected_ids = (11, 12, 13, 14)
        actual_ids = tuple(reading.motor_id for reading in readings)
        if actual_ids != expected_ids:
            raise ControllerError(
                f"Telemetry motor order mismatch: expected {expected_ids}, received {actual_ids}"
            )

        raw_positions = tuple(reading.position_raw for reading in readings)
        motors = MotorAngles(*(raw_to_degrees(raw) for raw in raw_positions))
        joints = motor_to_fk_angles(motors)
        timestamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        state = RobotState(
            timestamp_utc=timestamp,
            raw_positions=raw_positions,
            motors=motors,
            joints=joints,
            physical_xyz=forward_kinematics_physical(motors),
            internal_xyz=forward_kinematics(motors),
        )
        return TelemetrySnapshot(
            timestamp_utc=timestamp,
            state=state,
            motors=tuple(readings),
        )

    def move_motor_angles(self, motors: MotorAngles) -> None:
        with self._io_lock:
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

    def set_motion_speed(self, scale: float) -> None:
        if not math.isfinite(scale) or not 0.25 <= scale <= 1.0:
            raise ValueError("Motion speed scale must be between 0.25 and 1.0")
        response = self._command(f"SET_SPEED,{scale:.3f}")
        if response != "OK,SPEED":
            raise ControllerError("Speed setting rejected. Upload the updated repeatability OpenCR sketch. Received: " + response)

    def _verify_motor_targets(self, target: MotorAngles) -> None:
        actual = self.read_motor_angles()
        errors = tuple(
            abs(normalize_angle(actual_angle - target_angle))
            for actual_angle, target_angle in zip(actual.as_tuple(), target.as_tuple(), strict=True)
        )
        largest_error = max(errors)
        self.last_motor_tracking_errors = errors
        if largest_error <= config.POST_MOVE_TOLERANCE_DEGREES:
            return
        index = errors.index(largest_error)
        motor_id = (11, 12, 13, 14)[index]
        raise ControllerError(
            f"Motor target not reached: ID{motor_id} commanded={target.as_tuple()[index]:.3f} deg, "
            f"actual={actual.as_tuple()[index]:.3f} deg, error={largest_error:.3f} deg"
        )

    def _move_named_pose(self, command: str, target: MotorAngles, label: str) -> None:
        with self._io_lock:
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
        with self._io_lock:
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
                    joint_degrees = JointAngles(*(math.degrees(value) for value in joints))
                    motors = fk_to_motor_angles(joint_degrees)
                    # Python owns the calibrated FK. Recompute physical XYZ here.
                    points.append(TrajectoryPoint(time_s, motors, forward_kinematics_physical(motors)))
                elif response.startswith("TRAJ_END,"):
                    return points
                elif response.startswith("ERROR"):
                    raise ControllerError(response)

    def move_xyz_point(self, point, reference=None) -> None:
        result = inverse_kinematics_physical(
            point.xyz.x, point.xyz.y, point.xyz.z, reference=reference
        ).ik
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
            result = inverse_kinematics_physical(
                point.xyz.x, point.xyz.y, point.xyz.z, reference=reference
            ).ik
            reference = result.joint_angles
            validate_motor_angles(result.motor_angles)
            self.move_motor_angles(result.motor_angles)
