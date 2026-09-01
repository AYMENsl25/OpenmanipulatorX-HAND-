"""Serial protocol client for the OpenCR firmware."""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

import config
from kinematics import MotorAngles, validate_motor_angles


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

    def move_home(self) -> None:
        if not self.status.torque_on:
            raise ControllerError("Cannot move: Torque is OFF")
        response = self._command("HOME")
        if response != "MOVING":
            raise ControllerError(f"Expected MOVING, received: {response}")
        done = self._read_line(config.MOVE_TIMEOUT_SECONDS)
        if done != "DONE":
            raise ControllerError(f"Home movement failed or malformed response: {done}")
