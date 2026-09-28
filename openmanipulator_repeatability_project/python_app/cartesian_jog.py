"""Shared, supervised Cartesian stream used by the keyboard tab."""
from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import config
from experiment_frame import forward_kinematics_physical
from kinematics import (KinematicsError, MotorAngles, XYZ, motor_to_fk_angles,
                        normalize_angle, validate_motor_angles, validate_xyz_target)
from pick_place_panel import _segment
from rear_reference_frame import axis_to_rear_xyz, rear_to_axis_xyz
from serial_controller import ControllerError, OpenCRController

INPUT_MAX_AGE_S = 0.30
POLL_MS = 25
STREAM_PERIOD_S = 0.05
JOG_SPEED_MM_S = 20.0
JOG_ACCEL_MM_S_PER_TICK = 4.0
JOG_MAX_LEAD_MM = 3.0
JOG_HELD_DRIFT_MM = 3.0
JOG_MAX_MOTOR_LEAD_DEG = 3.5


def normalized_xyz_input(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Keep diagonal stick movement no faster than one full-axis command."""
    magnitude = math.sqrt(x * x + y * y + z * z)
    divisor = max(1.0, magnitude)
    return x / divisor, y / divisor, z / divisor


def advance_xyz_target(commanded: XYZ, measured: XYZ,
                       vector: tuple[float, float, float],
                       velocity: tuple[float, float, float],
                       speed_scale: float, elapsed_s: float
                       ) -> tuple[XYZ, tuple[float, float, float]]:
    """Advance requested axes; never rebase held axes on sagging FK."""
    if not 0.25 <= speed_scale <= 2.0 or not 0 < elapsed_s <= 0.2:
        raise ValueError("Invalid XYZ jog speed or timing")
    old = commanded.x, commanded.y, commanded.z
    feedback = measured.x, measured.y, measured.z
    next_values = []
    next_velocity = []
    for requested, current_speed, hold, actual in zip(
            vector, velocity, old, feedback, strict=True):
        wanted_speed = requested * JOG_SPEED_MM_S * speed_scale
        if requested == 0.0:
            smoothed_speed = 0.0
            goal = hold
        else:
            if wanted_speed * current_speed < 0:
                current_speed = 0.0
            delta_speed = max(-JOG_ACCEL_MM_S_PER_TICK,
                              min(JOG_ACCEL_MM_S_PER_TICK, wanted_speed - current_speed))
            smoothed_speed = current_speed + delta_speed
            goal = hold + smoothed_speed * elapsed_s
            goal = max(actual - JOG_MAX_LEAD_MM,
                       min(actual + JOG_MAX_LEAD_MM, goal))
        next_values.append(goal)
        next_velocity.append(smoothed_speed)
    return XYZ(*next_values), tuple(next_velocity)


def plan_xyz_jog_step(actual: MotorAngles, previous_motors: MotorAngles,
                      commanded: XYZ, velocity: tuple[float, float, float],
                      vector: tuple[float, float, float], speed_scale: float,
                      elapsed_s: float
                      ) -> tuple[XYZ, MotorAngles, tuple[float, float, float]]:
    """One IK-checked stream target within workspace and motor lead limits."""
    measured = axis_to_rear_xyz(forward_kinematics_physical(actual))
    proposed, next_velocity = advance_xyz_target(
        commanded, measured, vector, velocity, speed_scale, elapsed_s)
    if proposed.z < config.GROUND_Z_MM:
        raise ControllerError("XYZ ground limit")
    validate_xyz_target(rear_to_axis_xyz(proposed))
    path, _, _ = _segment(measured, proposed, actual,
                          motor_to_fk_angles(actual))
    if len(path) != 1:
        raise ControllerError("XYZ jog would need a multi-leg path; change direction")
    motors = path[0]
    check_jog_arc(actual, motors)
    check_jog_arc(previous_motors, motors)
    motor_lead = max(abs(normalize_angle(a - b)) for a, b in zip(
        motors.as_tuple(), actual.as_tuple(), strict=True))
    if motor_lead > JOG_MAX_MOTOR_LEAD_DEG:
        raise ControllerError(
            f"XYZ jog needs {motor_lead:.2f}° motor lead; reduce speed or change direction")
    return proposed, motors, next_velocity


def check_jog_arc(start: MotorAngles, end: MotorAngles) -> None:
    """Keep the short direct motor segment inside joint, ground and workspace limits."""
    angles = start.as_tuple()
    deltas = tuple(normalize_angle(b - a) for a, b in zip(
        angles, end.as_tuple(), strict=True))
    for n in range(1, 11):
        fraction = n / 10.0
        sample = MotorAngles(*(float((a + fraction * d) % 360.0)
                               for a, d in zip(angles, deltas, strict=True)))
        validate_motor_angles(sample)
        xyz = forward_kinematics_physical(sample)
        if xyz.z < config.GROUND_Z_MM:
            raise ControllerError(f"Ground limit along jog path: TCP Z={xyz.z:.1f} mm")
        validate_xyz_target(xyz)


class CartesianJogPanel(ttk.Frame):
    """Lifecycle and motion worker for keyboard Cartesian jogging."""

    def __init__(self, parent, *, controller: OpenCRController, busy_callback,
                 log_callback=None, move_work_callback=None):
        super().__init__(parent, padding=12)
        self.controller = controller
        self.busy_callback = busy_callback
        self.log_callback = log_callback
        self.move_work_callback = move_work_callback
        self.status = tk.StringVar(value="Movement OFF")
        self.inputs = tk.StringVar(value="No keys held")
        self.speed = tk.DoubleVar(value=1.25)
        self._speed_value = 1.25
        self.armed = False
        self.running = False
        self.closed = False
        self._sample = (0.0, (0.0, 0.0, 0.0), False, False)
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._build()
        self.after(POLL_MS, self._poll)

    def _build(self):
        raise NotImplementedError

    def _poll(self):
        raise NotImplementedError

    def _check_input(self):
        raise NotImplementedError

    def arm(self):
        try:
            self._check_input()
            if self.busy_callback():
                raise ValueError("Stop the active robot operation first")
            if not self.controller.status.connected or not self.controller.status.torque_on:
                raise ValueError("Connect OpenCR and turn torque ON first")
            state = self.controller.read_robot_state()
            outside_ik = [name for name, limit in config.IK_SOLVER_JOINT_LIMITS.items()
                          if not limit.contains(getattr(state.joints, name))]
            if outside_ik:
                raise ValueError("Current pose is outside the Cartesian jog range. "
                                 "If at SCAN, press MOVE TO WORK first. "
                                 f"Out-of-range joints: {', '.join(outside_ik)}")
            response = self.controller._command("CAPABILITIES")
            if "GAMEPAD_STREAM_V1" not in response.split(",")[1:]:
                raise ValueError("Upload the included GAMEPAD_STREAM_V1 OpenCR .ino, then reconnect")
            self._speed_value = float(self.speed.get())
            if not math.isfinite(self._speed_value) or not 0.25 <= self._speed_value <= 2.0:
                raise ValueError("Speed must be 0.25–2.0")
            self._cancel.clear()
            self.armed = True
            self.status.set("Keyboard enabled")
        except (ValueError, ControllerError) as exc:
            self.status.set(str(exc))

    def move_to_work(self):
        if self.running or self.busy_callback():
            self.status.set("Stop the active robot motion before moving to WORK")
            return
        if self.move_work_callback is None:
            self.status.set("WORK action is unavailable")
            return
        self.disarm()
        self.move_work_callback()
        self.status.set("Check the robot is at WORK, then enable keyboard again")

    def disarm(self):
        self.armed = False
        self._cancel.set()
        self.status.set("Robot movement OFF")

    def close(self):
        self.closed = True
        self.disarm()

    def _work(self):
        stream_active = False
        velocity = (0.0, 0.0, 0.0)
        commanded = None
        previous_tick = None
        held_drift_samples = 0
        try:
            self.controller.begin_exclusive_motion()
            while self.armed and not self._cancel.is_set():
                tick = time.monotonic()
                with self._lock:
                    stamp, vector, enabled, _ = self._sample
                if time.monotonic() - stamp > INPUT_MAX_AGE_S or not enabled:
                    self._cancel.set()
                    break
                if not any(vector):
                    break
                if not stream_active:
                    actual = self.controller.start_jog_stream()
                    stream_active = True
                    previous = actual
                    commanded = axis_to_rear_xyz(forward_kinematics_physical(actual))
                    previous_tick = tick - STREAM_PERIOD_S
                elapsed = min(0.2, max(0.001, tick - previous_tick))
                proposed, motors, velocity = plan_xyz_jog_step(
                    actual, previous, commanded, velocity, vector,
                    self._speed_value, elapsed)
                with self._lock:
                    fresh_stamp, fresh_vector, fresh_enable, _ = self._sample
                if (self._cancel.is_set() or not fresh_enable or not any(fresh_vector)
                        or time.monotonic() - fresh_stamp > INPUT_MAX_AGE_S):
                    break
                actual = self.controller.update_jog_stream(motors)
                previous = motors
                commanded = proposed
                previous_tick = tick
                measured = axis_to_rear_xyz(forward_kinematics_physical(actual))
                drift = tuple(abs(a - b) for a, b in zip(
                    (commanded.x, commanded.y, commanded.z),
                    (measured.x, measured.y, measured.z), strict=True))
                held_drift = max((drift[i] for i in range(3) if vector[i] == 0.0),
                                 default=0.0)
                held_drift_samples = (held_drift_samples + 1
                                      if held_drift > JOG_HELD_DRIFT_MM else 0)
                if held_drift_samples >= 3:
                    raise ControllerError(
                        f"A held XYZ axis drifted {held_drift:.1f} mm; motion stopped")
                self._cancel.wait(max(0.0, STREAM_PERIOD_S - (time.monotonic() - tick)))
        except (ControllerError, KinematicsError, ValueError, OSError) as exc:
            self.after(0, lambda message=str(exc): self.status.set(
                f"Keyboard motion stopped: {message}"))
            self._cancel.set()
            self.armed = False
        finally:
            if stream_active:
                try:
                    self.controller.stop_jog_stream()
                except (ControllerError, OSError) as exc:
                    self.armed = False
                    self.after(0, lambda message=str(exc): self.status.set(message))
            self.controller.end_exclusive_motion()
            self.running = False
