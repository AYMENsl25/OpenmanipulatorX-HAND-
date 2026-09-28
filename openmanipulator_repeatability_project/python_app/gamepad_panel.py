"""Supervised gamepad control for one joint or a commanded Cartesian TCP."""
from __future__ import annotations

import math
import csv
import threading
import time
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from datetime import datetime

import config
from app_paths import PROJECT_ROOT
from cartesian_jog import (JOG_HELD_DRIFT_MM, advance_xyz_target,
                           check_jog_arc, normalized_xyz_input,
                           plan_xyz_jog_step)
from experiment_frame import forward_kinematics_physical
from kinematics import (KinematicsError, MotorAngles, motor_to_fk_angles,
                        XYZ, normalize_angle, normalize_motor_angle, validate_motor_angles,
                        validate_xyz_target)
from rear_reference_frame import axis_to_rear_xyz
from serial_controller import ControllerError, OpenCRController

try:
    import pygame
except ImportError:
    pygame = None

IDS = (11, 12, 13, 14)
NAMES = ("base / q1", "shoulder / q2", "elbow / q3", "wrist / q4")
POLL_MS = 25
INPUT_MAX_AGE_S = 0.30
STREAM_PERIOD_S = 0.05
JOINT_SPEED_DEG_S = 10.0  # Speed 2.0 reaches, but does not exceed, OpenCR's 20 deg/s cap.
JOINT_ACCEL_DEG_S_PER_TICK = 3.0
TARGET_LOOKAHEAD_S = 0.10
MAX_COMMAND_LEAD_DEG = 3.0  # Below OpenCR's 5-degree tracking limit.
MAX_HELD_DRIFT_DEG = 1.5
GRIPPER_OPEN_RAW = 1800
GRIPPER_CLOSE_RAW = 2650
GRIPPER_CURRENT_CUTOFF_RAW = 200
DEAD_ZONE = 0.15


@dataclass(frozen=True)
class ControllerMapping:
    x_axis: int
    y_axis: int
    z_axis: int
    previous_button: int
    next_button: int
    gripper_button: int
    work_button: int
    stop_button: int

    def axes(self) -> tuple[int, int, int]:
        return self.x_axis, self.y_axis, self.z_axis

    def buttons(self) -> tuple[int, int, int, int, int]:
        return (self.previous_button, self.next_button, self.gripper_button,
                self.work_button, self.stop_button)


def stick_command(value: float, invert: bool = True) -> float:
    """Normalized signed rate, with a center dead zone."""
    value = -value if invert else value
    if not math.isfinite(value) or abs(value) <= DEAD_ZONE:
        return 0.0
    scaled = min(1.0, (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE))
    return math.copysign(scaled ** 1.3, value)


def smooth_joint_speed(previous_speed: float, direction: float,
                       speed_scale: float) -> float:
    """Rate-limit a joint jog, but drop old momentum on stick reversal."""
    wanted = direction * JOINT_SPEED_DEG_S * speed_scale
    if previous_speed * wanted < 0:
        previous_speed = 0.0
    return previous_speed + max(-JOINT_ACCEL_DEG_S_PER_TICK,
                                min(JOINT_ACCEL_DEG_S_PER_TICK,
                                    wanted - previous_speed))


def joint_jog_target(actual: MotorAngles, index: int, direction: float,
                     speed_scale: float) -> MotorAngles:
    """Validate the next direct motor target and entire short motor arc."""
    return advance_joint_target(actual, actual, index, direction, speed_scale,
                                TARGET_LOOKAHEAD_S)


def advance_joint_target(actual: MotorAngles, previous: MotorAngles, index: int,
                         direction: float, speed_scale: float,
                         elapsed_s: float) -> MotorAngles:
    """Integrate the requested angle; keep it close enough to measured feedback."""
    if index not in range(4) or not math.isfinite(speed_scale) or not 0.25 <= speed_scale <= 2.0:
        raise ValueError("Invalid selected motor or speed")
    if not math.isfinite(direction) or abs(direction) > 1.0 or not 0 < elapsed_s <= 0.2:
        raise ValueError("Invalid joystick input")
    # The other three position-controlled joints must retain their latched
    # goals. Replacing them with encoder feedback every tick makes a loaded
    # shoulder drift downward while the base is jogging.
    values = list(previous.as_tuple())
    prior = previous.as_tuple()[index]
    # Discard target backlog on reversal; the new direction must take effect now.
    measured_selected = actual.as_tuple()[index]
    if normalize_angle(prior - measured_selected) * direction < 0:
        prior = measured_selected
    desired = normalize_motor_angle(prior +
                                    direction * JOINT_SPEED_DEG_S * speed_scale * elapsed_s)
    lead = normalize_angle(desired - measured_selected)
    values[index] = normalize_motor_angle(measured_selected +
                                          max(-MAX_COMMAND_LEAD_DEG,
                                              min(MAX_COMMAND_LEAD_DEG, lead)))
    target = MotorAngles(*values)
    validate_motor_angles(target)
    check_jog_arc(actual, target)
    return target


class JointJogLog:
    """Write one recoverable CSV row per command/feedback sample."""

    def __init__(self, directory=None):
        path = (directory or PROJECT_ROOT / "logs") / f"joint_jog_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("x", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(("time_local", "event", "selected_id", "stick_rate",
                               "speed_scale", *(f"command_id{i}_deg" for i in IDS),
                               *(f"measured_id{i}_deg" for i in IDS),
                               *(f"joint_q{i}_deg" for i in range(1, 5)),
                               "tcp_x_mm", "tcp_y_mm", "tcp_z_mm"))
        self._file.flush()

    def record(self, event: str, selected_id: int, direction: float,
               speed: float, command: MotorAngles, measured: MotorAngles):
        xyz = forward_kinematics_physical(measured)
        joints = motor_to_fk_angles(measured)
        self._writer.writerow((datetime.now().isoformat(timespec="milliseconds"), event,
                               selected_id, f"{direction:.3f}", f"{speed:.2f}",
                               *command.as_tuple(), *measured.as_tuple(),
                               *joints.as_tuple(), xyz.x, xyz.y, xyz.z))
        self._file.flush()

    def close(self):
        self._file.close()


class XYZJogLog:
    """Record commanded and measured TCP separately during Cartesian jogging."""

    def __init__(self):
        path = PROJECT_ROOT / "logs" / f"xyz_gamepad_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("x", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(("time_local", "event", "input_x", "input_y", "input_z",
                               "command_x_rear_mm", "command_y_mm", "command_z_mm",
                               "measured_x_rear_mm", "measured_y_mm", "measured_z_mm",
                               *(f"command_id{i}_deg" for i in IDS),
                               *(f"measured_id{i}_deg" for i in IDS)))
        self._file.flush()

    def record(self, event: str, vector: tuple[float, float, float],
               command: XYZ, motors: MotorAngles, actual: MotorAngles):
        measured = axis_to_rear_xyz(forward_kinematics_physical(actual))
        self._writer.writerow((datetime.now().isoformat(timespec="milliseconds"), event,
                               *vector, command.x, command.y, command.z,
                               measured.x, measured.y, measured.z,
                               *motors.as_tuple(), *actual.as_tuple()))
        self._file.flush()

    def close(self):
        self._file.close()


class GamepadPanel(ttk.Frame):
    def __init__(self, parent, *, controller: OpenCRController, busy_callback,
                 log_callback=None, move_work_callback=None):
        super().__init__(parent, padding=12)
        self.controller = controller
        self.busy_callback = busy_callback
        self.log_callback = log_callback
        self.move_work_callback = move_work_callback
        self.status = tk.StringVar(value="Controller disconnected; movement OFF")
        self.inputs = tk.StringVar(value="Connect controller to see axes and buttons")
        self.selected_text = tk.StringVar(value="Selected: ID11 — base / q1")
        self.mode = tk.StringVar(value="MOTOR")
        self.mode_help = tk.StringVar(value="MOTOR: B/X choose ID; right stick up/down jogs one joint.")
        self.feedback = tk.StringVar(value="Motor angle: —   Calibrated joint angle: —")
        self.coords_text = tk.StringVar(value="XYZ target/readback: —")
        self.hold_status = tk.StringVar(value="Other joints: position hold active during jogging")
        self.log_path = tk.StringVar(value="Jog log: created when movement starts")
        self.gripper_status = tk.StringVar(value="ID15 gripper: ready when OpenCR is connected")
        self.device_index = tk.StringVar(value="0")
        self.stick_axis = tk.StringVar(value="3")
        self.y_axis = tk.StringVar(value="2")
        self.z_axis = tk.StringVar(value="1")
        self.invert_stick = tk.BooleanVar(value=True)
        self.invert_y = tk.BooleanVar(value=True)
        self.invert_z = tk.BooleanVar(value=True)
        self.previous_button = tk.StringVar(value="2")  # F710 X
        self.next_button = tk.StringVar(value="1")      # F710 B
        self.gripper_button = tk.StringVar(value="0")   # F710 A
        self.work_button = tk.StringVar(value="3")      # F710 Y
        self.stop_button = tk.StringVar(value="7")      # F710 Start
        self.speed = tk.DoubleVar(value=1.25)
        self._speed_value = 1.25
        self.selected_index = 0
        self.device = None
        self.device_instance_id = None
        self.armed = False
        self.running = False
        self.gripper_busy = False
        self.closed = False
        self._sample = (0.0, 0.0, (0.0, 0.0, 0.0))
        self._last_polled_at = 0.0
        self._pressed_buttons = set()
        self._pending_selection = 0
        self._blocked_direction = None
        self._last_motors = None
        self._capture = None
        self._capture_baseline = ()
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._build()
        self.after(POLL_MS, self._poll)

    def _build(self):
        self.columnconfigure(0, weight=1)
        ttk.Label(self, text="Controller movement", font=("TkDefaultFont", 14, "bold")).grid(
            row=0, column=0, sticky="w")
        ttk.Label(self, textvariable=self.mode_help,
                  wraplength=950).grid(row=1, column=0, sticky="w", pady=7)
        controls = ttk.Frame(self)
        controls.grid(row=2, column=0, sticky="w", pady=6)
        ttk.Label(controls, text="Controller index").pack(side="left")
        ttk.Entry(controls, textvariable=self.device_index, width=4).pack(side="left", padx=5)
        ttk.Button(controls, text="CONNECT CONTROLLER", command=self.connect_device).pack(side="left", padx=5)
        ttk.Button(controls, text="ENABLE CONTROLLER", command=self.arm).pack(side="left", padx=5)
        ttk.Button(controls, text="STOP / DISABLE", command=self.disarm).pack(side="left", padx=5)
        ttk.Label(controls, text="Move by:").pack(side="left", padx=(18, 4))
        ttk.Radiobutton(controls, text="MOTOR angles", variable=self.mode,
                        value="MOTOR", command=self._mode_changed).pack(side="left", padx=3)
        ttk.Radiobutton(controls, text="XYZ", variable=self.mode,
                        value="XYZ", command=self._mode_changed).pack(side="left", padx=3)
        mapping = ttk.LabelFrame(self, text="Mapping (Logitech F710 defaults)", padding=8)
        mapping.grid(row=3, column=0, sticky="ew", pady=8)
        for column, (label, variable) in enumerate((
            ("Right vertical / X", self.stick_axis),
            ("Right horizontal / Y", self.y_axis),
            ("Left vertical / Z", self.z_axis),
        )):
            ttk.Label(mapping, text=label).grid(row=0, column=column, padx=6)
            ttk.Entry(mapping, textvariable=variable, width=5).grid(row=1, column=column, padx=6)
        for column, (label, kind) in enumerate((
            ("Detect X axis", "x_axis"), ("Detect Y axis", "y_axis"),
            ("Detect Z axis", "z_axis"),
        )):
            ttk.Button(mapping, text=label, command=lambda name=kind: self.capture(name)).grid(
                row=2, column=column, pady=4)
        for column, (label, variable) in enumerate((
            ("X / Square previous", self.previous_button),
            ("B / Circle next", self.next_button),
            ("A / Cross gripper", self.gripper_button),
            ("Y / Triangle WORK", self.work_button),
            ("STOP / Start", self.stop_button),
        )):
            ttk.Label(mapping, text=label).grid(row=3, column=column, padx=4)
            ttk.Entry(mapping, textvariable=variable, width=5).grid(row=4, column=column, padx=4)
        for column, (label, kind) in enumerate((
            ("Detect X", "previous"), ("Detect B", "next"),
            ("Detect A", "gripper"), ("Detect Y", "work"),
            ("Detect STOP", "stop"),
        )):
            ttk.Button(mapping, text=label, command=lambda name=kind: self.capture(name)).grid(
                row=5, column=column, pady=4)
        for column, (label, variable) in enumerate((
            ("Invert X", self.invert_stick), ("Invert Y", self.invert_y),
            ("Invert Z", self.invert_z))):
            ttk.Checkbutton(mapping, text=label, variable=variable).grid(
                row=6, column=column, sticky="w")
        ttk.Label(mapping, text="Speed (0.25–2.0)").grid(row=6, column=3)
        ttk.Spinbox(mapping, from_=0.25, to=2.0, increment=0.25,
                    textvariable=self.speed, width=6).grid(row=6, column=4)
        ttk.Label(self, textvariable=self.selected_text, font=("TkDefaultFont", 12, "bold")).grid(
            row=4, column=0, sticky="w", pady=5)
        ttk.Label(self, textvariable=self.feedback).grid(row=5, column=0, sticky="w")
        ttk.Label(self, textvariable=self.coords_text, wraplength=950).grid(
            row=13, column=0, sticky="w", pady=3)
        ttk.Label(self, textvariable=self.hold_status, wraplength=950).grid(
            row=12, column=0, sticky="w", pady=3)
        ttk.Label(self, textvariable=self.inputs, wraplength=950).grid(row=6, column=0, sticky="w", pady=7)
        ttk.Label(self, textvariable=self.status, wraplength=950).grid(row=7, column=0, sticky="w", pady=5)
        ttk.Label(self, text="MOTOR can start at SCAN. XYZ needs a reachable pose (Y requests WORK). "
                  "Joint, ground, workspace and checked-path limits remain active.",
                  wraplength=950).grid(row=8, column=0, sticky="w")
        ttk.Label(self, textvariable=self.log_path, wraplength=950).grid(
            row=9, column=0, sticky="w", pady=4)
        gripper = ttk.LabelFrame(self, text="ID15 gripper — click only when the arm is stationary", padding=8)
        gripper.grid(row=10, column=0, sticky="ew", pady=7)
        ttk.Button(gripper, text="OPEN ID15 — 1800 RAW",
                   command=lambda: self.command_gripper(GRIPPER_OPEN_RAW)).pack(side="left", padx=5)
        ttk.Button(gripper, text="CLOSE ID15 — 2650 RAW",
                   command=lambda: self.command_gripper(GRIPPER_CLOSE_RAW)).pack(side="left", padx=5)
        ttk.Label(gripper, textvariable=self.gripper_status, wraplength=500).pack(side="left", padx=10)
        limits = ", ".join(
            f"ID{motor_id} {config.FK_JOINT_LIMITS[f'theta{i + 1}'].minimum:+.0f}°"
            f"..{config.FK_JOINT_LIMITS[f'theta{i + 1}'].maximum:+.0f}°"
            for i, motor_id in enumerate(IDS))
        ttk.Label(self, text=f"Calibrated joint limits: {limits}. All four joints remain torque ON to hold the arm.",
                  wraplength=950).grid(row=11, column=0, sticky="w", pady=4)

    def _mapping(self) -> ControllerMapping:
        values = tuple(int(field.get()) for field in (
            self.stick_axis, self.y_axis, self.z_axis,
            self.previous_button, self.next_button, self.gripper_button,
            self.work_button, self.stop_button))
        mapping = ControllerMapping(*values)
        if any(value < 0 for value in values) or len(set(mapping.axes())) != 3 or len(set(mapping.buttons())) != 5:
            raise ValueError("Use three distinct axes and five distinct button numbers")
        return mapping

    def _mode_changed(self):
        self.disarm()
        if self.mode.get() == "XYZ":
            self.mode_help.set("XYZ: right stick forward/back = X, left/right = Y; "
                               "left stick up/down = Z. A toggles gripper; Y requests WORK when stopped.")
            self.selected_text.set("XYZ mode — move to WORK before enabling if at SCAN")
        else:
            self.mode_help.set("MOTOR: X/B choose ID11–ID14; right stick up/down "
                               "jogs that joint. A toggles gripper; Y requests WORK when stopped.")
            self._show_selected()
        self.status.set("Mode changed. Re-enable control after the previous jog has stopped.")

    def _show_selected(self):
        self.selected_text.set(f"Selected: ID{IDS[self.selected_index]} — {NAMES[self.selected_index]}")
        if self._last_motors is not None:
            self._show_angles(self._last_motors)
        else:
            self.feedback.set("Motor angle: read OpenCR after enabling joint control")

    def _show_angles(self, motors: MotorAngles):
        self._last_motors = motors
        index = self.selected_index
        joint = motor_to_fk_angles(motors).as_tuple()[index]
        motor = motors.as_tuple()[index]
        self.feedback.set(f"ID{IDS[index]} motor angle: {motor:.2f}°   "
                          f"Calibrated joint angle: {joint:+.2f}°")

    def connect_device(self):
        if pygame is None:
            self.status.set("pygame-ce is missing from this installation")
            return
        self.disarm()
        if self.running:
            self.status.set("Wait for the previous motor jog to stop before reconnecting")
            return
        try:
            if self.device is not None:
                self.device.quit()
            pygame.display.init()
            pygame.joystick.init()
            index = int(self.device_index.get())
            if not 0 <= index < pygame.joystick.get_count():
                raise ValueError(f"Controller {index} not present; found {pygame.joystick.get_count()}")
            pygame.event.pump()
            self.device = pygame.joystick.Joystick(index)
            self.device.init()
            self.device_instance_id = self.device.get_instance_id()
            self._pressed_buttons.clear()
            self._capture = None
            self.status.set(f"Connected: {self.device.get_name()}. Check inputs, then enable joint control.")
        except (ValueError, pygame.error) as exc:
            self.device = None
            self.status.set(str(exc))

    def capture(self, control: str):
        if self.device is None or not self.device.get_init():
            self.status.set("Connect the controller before detecting controls")
            return
        self.disarm()
        pygame.event.pump()
        self._capture = control
        self._capture_baseline = tuple(self.device.get_axis(i)
                                       for i in range(self.device.get_numaxes()))
        self.status.set("Move only the joystick axis you want to detect"
                        if control.endswith("_axis") else
                        f"Press physical {self._control_name(control)}")

    @staticmethod
    def _control_name(control: str) -> str:
        return {"previous": "X / Square", "next": "B / Circle",
                "gripper": "A / Cross", "work": "Y / Triangle",
                "stop": "STOP / Start"}[control]

    def arm(self):
        try:
            if self.running:
                raise ValueError("Wait for the previous motor jog to stop")
            if self.gripper_busy:
                raise ValueError("Wait for the ID15 gripper command to finish")
            mapping = self._mapping()
            if self.device is None or not self.device.get_init():
                raise ValueError("Connect the controller first")
            if time.monotonic() - self._last_polled_at > INPUT_MAX_AGE_S:
                raise ValueError("No recent controller input; check the live readout")
            if max(mapping.axes()) >= self.device.get_numaxes() or max(mapping.buttons()) >= self.device.get_numbuttons():
                raise ValueError("Mapped axis or button exceeds controller inputs")
            if self.busy_callback():
                raise ValueError("Stop the active robot operation first")
            if not self.controller.status.connected or not self.controller.status.torque_on:
                raise ValueError("Connect OpenCR and turn torque ON first")
            state = self.controller.read_robot_state()
            validate_motor_angles(state.motors)
            if state.physical_xyz.z < config.GROUND_Z_MM:
                raise ValueError("Current TCP is below the configured ground plane")
            validate_xyz_target(state.physical_xyz)
            if self.mode.get() == "XYZ":
                outside_ik = [name for name, limit in config.IK_SOLVER_JOINT_LIMITS.items()
                              if not limit.contains(getattr(state.joints, name))]
                if outside_ik:
                    raise ValueError("XYZ mode needs a reachable starting pose. "
                                     "From SCAN, press Y to move to WORK first; "
                                     f"outside IK limits: {', '.join(outside_ik)}")
            response = self.controller._command("CAPABILITIES")
            if "GAMEPAD_STREAM_V1" not in response.split(",")[1:]:
                raise ValueError("Upload included GAMEPAD_STREAM_V1 OpenCR .ino and reconnect")
            self._speed_value = float(self.speed.get())
            if not math.isfinite(self._speed_value) or not 0.25 <= self._speed_value <= 2.0:
                raise ValueError("Speed must be 0.25–2.0")
            self._show_angles(state.motors)
            self._capture = None
            self._blocked_direction = None
            self._cancel.clear()
            self.armed = True
            self.status.set("XYZ control enabled. Center sticks to stop."
                            if self.mode.get() == "XYZ" else
                            "Motor control enabled. Right stick moves selected ID; center to stop.")
        except (ValueError, ControllerError, KinematicsError) as exc:
            self.status.set(str(exc))

    def command_gripper(self, target_raw: int | None):
        """Run an explicit ID15 command, never concurrently with an arm jog."""
        if self.running or self.gripper_busy:
            self.gripper_status.set("Center the stick and wait for arm motion to stop first")
            return
        if self.busy_callback():
            self.gripper_status.set("Stop the other robot operation first")
            return
        if not self.controller.status.connected or not self.controller.status.torque_on:
            self.gripper_status.set("Connect OpenCR and turn torque ON first")
            return
        self.disarm()
        self.gripper_busy = True
        self.gripper_status.set(
            "ID15 reading position for toggle…" if target_raw is None
            else f"ID15 moving toward {target_raw} RAW…")
        threading.Thread(target=self._gripper_work, args=(target_raw,), daemon=True).start()

    def _gripper_work(self, target_raw: int | None):
        try:
            self.controller.begin_exclusive_motion()
            try:
                self.controller.configure_gripper(GRIPPER_OPEN_RAW, GRIPPER_CLOSE_RAW)
                before = self.controller.read_gripper_raw()
                if target_raw is None:
                    midpoint = (GRIPPER_OPEN_RAW + GRIPPER_CLOSE_RAW) / 2
                    target_raw = GRIPPER_CLOSE_RAW if before < midpoint else GRIPPER_OPEN_RAW
                result = self.controller.move_gripper(target_raw, GRIPPER_CURRENT_CUTOFF_RAW)
                after = self.controller.read_gripper_raw()
            finally:
                self.controller.end_exclusive_motion()
            self.after(0, lambda: self.gripper_status.set(
                f"ID15 {result.outcome}: {before} → {after} RAW "
                f"(requested {target_raw}; current {result.current_raw} RAW)"))
        except (ControllerError, ValueError, OSError) as exc:
            self.after(0, lambda message=str(exc): self.gripper_status.set(
                f"ID15 stopped: {message}; inspect jaws before continuing"))
        finally:
            self.gripper_busy = False

    def disarm(self):
        self.armed = False
        self._cancel.set()
        self.status.set("Controller movement OFF")

    def close(self):
        self.closed = True
        self.disarm()
        if self.device is not None:
            self.device.quit()

    def move_to_work(self):
        """Y is an explicit, confirmed WORK request, never an automatic jog step."""
        if self.running or self.gripper_busy or self.busy_callback():
            self.status.set("Center both sticks and wait for the active motion before pressing Y")
            return
        if not self.controller.status.connected or not self.controller.status.torque_on:
            self.status.set("Connect OpenCR and turn torque ON before requesting WORK")
            return
        if self.move_work_callback is None:
            self.status.set("WORK action is unavailable")
            return
        self.disarm()
        self.move_work_callback()  # Existing GUI asks the operator to confirm.
        self.status.set("WORK requested. Verify the pose, then enable the controller again.")

    def _poll(self):
        if self.closed:
            return
        try:
            if self.device is None:
                self.inputs.set("No controller. Click CONNECT CONTROLLER.")
            else:
                events = pygame.event.get()
                if not self.device.get_init() or any(
                    event.type == pygame.JOYDEVICEREMOVED and
                    event.instance_id == self.device_instance_id for event in events
                ):
                    raise ControllerError("Controller disconnected")
                axes = tuple(round(self.device.get_axis(i), 2) for i in range(self.device.get_numaxes()))
                buttons = {i for i in range(self.device.get_numbuttons()) if self.device.get_button(i)}
                edge = buttons - self._pressed_buttons
                self._pressed_buttons = buttons
                self._last_polled_at = time.monotonic()
                if self._capture in ("x_axis", "y_axis", "z_axis") and len(axes) == len(self._capture_baseline):
                    changes = [abs(a - b) for a, b in zip(axes, self._capture_baseline, strict=True)]
                    if changes and max(changes) >= 0.5:
                        {"x_axis": self.stick_axis, "y_axis": self.y_axis,
                         "z_axis": self.z_axis}[self._capture].set(str(changes.index(max(changes))))
                        self._capture = None
                        self.status.set("Axis assigned. Check the live signs and invert if needed.")
                elif self._capture in ("previous", "next", "gripper", "work", "stop") and edge:
                    name = self._capture
                    chosen = min(edge)
                    {"previous": self.previous_button, "next": self.next_button,
                     "gripper": self.gripper_button, "work": self.work_button,
                     "stop": self.stop_button}[name].set(str(chosen))
                    self._capture = None
                    edge.clear()  # Capturing a button never actuates it.
                    self.status.set(f"{self._control_name(name)} assigned to button {chosen}")
                mapping = None
                if self._capture is None:
                    try:
                        mapping = self._mapping()
                        if max(mapping.axes()) >= len(axes) or max(mapping.buttons()) >= self.device.get_numbuttons():
                            raise ValueError("Mapped control exceeds available controller inputs")
                    except ValueError as exc:
                        if self.armed:
                            self.disarm()
                        self.status.set(f"Correct controller mapping: {exc}")
                        mapping = None
                if mapping is not None:
                    x = stick_command(axes[mapping.x_axis], self.invert_stick.get())
                    y = stick_command(axes[mapping.y_axis], self.invert_y.get())
                    z = stick_command(axes[mapping.z_axis], self.invert_z.get())
                    vector = normalized_xyz_input(x, y, z)
                    centered = not any(vector)
                    focused = (self.winfo_viewable() and
                               self.winfo_toplevel().focus_displayof() is not None)
                    if mapping.stop_button in buttons and self.armed:
                        self.disarm()
                        self.status.set("STOP button pressed; controller OFF")
                    elif mapping.gripper_button in edge or mapping.work_button in edge:
                        if not centered or not focused or self.running:
                            self.status.set("Center both sticks, focus this tab, and stop jogging before A or Y")
                        elif mapping.gripper_button in edge and mapping.work_button in edge:
                            self.status.set("Press only one action button at a time")
                        elif mapping.gripper_button in edge:
                            self.command_gripper(None)
                        else:
                            self.move_to_work()
                    if self.mode.get() == "MOTOR":
                        delta = (int(mapping.next_button in edge) -
                                 int(mapping.previous_button in edge))
                        if delta:
                            if not centered or self.running:
                                self.status.set("Center both sticks and stop before changing motor ID")
                            else:
                                self.selected_index = (self.selected_index + delta) % len(IDS)
                                self._show_selected()
                                self.status.set(f"Selected ID{IDS[self.selected_index]}")
                    if self.armed:
                        if not focused:
                            raise ControllerError("Controller tab lost focus")
                        if self.busy_callback() or not self.controller.status.connected or not self.controller.status.torque_on:
                            raise ControllerError("Another operation started or OpenCR disconnected")
                        speed = float(self.speed.get())
                        if not math.isfinite(speed) or not 0.25 <= speed <= 2.0:
                            raise ValueError("Speed must be 0.25–2.0")
                        self._speed_value = speed
                        moving = any(vector) if self.mode.get() == "XYZ" else bool(x)
                        if not moving:
                            self._blocked_direction = None
                        with self._lock:
                            self._sample = (time.monotonic(), x, vector)
                        blocked = (self._blocked_direction ==
                                   (self.selected_index, math.copysign(1, x))) if x else False
                        if not moving:
                            self._cancel.set()
                        elif not self.running and not blocked:
                            self._cancel.clear()
                            self.running = True
                            worker = self._work_xyz if self.mode.get() == "XYZ" else self._work
                            threading.Thread(target=worker, daemon=True).start()
                    self.inputs.set(
                        f"Mode={self.mode.get()} | X={x:+.2f}, Y={y:+.2f}, Z={z:+.2f} | "
                        f"axes {axes}; pressed buttons {tuple(sorted(buttons))}")
                else:
                    self.inputs.set(f"Axes {axes}; pressed buttons {tuple(sorted(buttons))}; edit mapping")
        except Exception as exc:
            self.disarm()
            self.device = None
            self.device_instance_id = None
            self.inputs.set("Controller input lost. Reconnect controller.")
            self.status.set(f"{type(exc).__name__}: {exc}; motor movement OFF")
            if self.log_callback:
                try:
                    self.log_callback(f"Joint controller input failure: {type(exc).__name__}: {exc}")
                except Exception:
                    pass
        self.after(POLL_MS, self._poll)

    def _work(self):
        stream_active = False
        chosen_index = self.selected_index
        count = 0
        jog_log = None
        target = None
        actual = None
        previous_tick = None
        initial_angle = None
        direction_start = None
        direction_sign = None
        held_drift_samples = 0
        joint_speed = 0.0
        try:
            self.controller.begin_exclusive_motion()
            while self.armed and not self._cancel.is_set():
                tick = time.monotonic()
                with self._lock:
                    stamp, direction, _ = self._sample
                if time.monotonic() - stamp > INPUT_MAX_AGE_S or not direction:
                    break
                if not stream_active:
                    actual = self.controller.start_jog_stream()
                    stream_active = True
                    target = actual
                    initial_angle = actual.as_tuple()[chosen_index]
                    direction_start = tick
                    direction_sign = math.copysign(1.0, direction)
                    previous_tick = tick - STREAM_PERIOD_S
                    jog_log = JointJogLog()
                    jog_log.record("START", IDS[chosen_index], direction, self._speed_value,
                                   target, actual)
                    self.after(0, lambda path=str(jog_log.path): self.log_path.set(f"Jog log: {path}"))
                    self.after(0, lambda: self.status.set(
                        f"Moving ID{IDS[chosen_index]}; center stick to stop"))
                try:
                    elapsed = min(0.2, max(0.001, tick - previous_tick))
                    joint_speed = smooth_joint_speed(joint_speed, direction, self._speed_value)
                    ramped_direction = joint_speed / (JOINT_SPEED_DEG_S * self._speed_value)
                    target = advance_joint_target(actual, target, chosen_index,
                                                  ramped_direction, self._speed_value, elapsed)
                except (KinematicsError, ValueError, ControllerError) as exc:
                    self._blocked_direction = (chosen_index, math.copysign(1, direction))
                    self.after(0, lambda message=str(exc): self.status.set(
                        f"ID{IDS[chosen_index]} limit: {message}. Center or reverse stick."))
                    break
                with self._lock:
                    fresh_stamp, fresh_direction, _ = self._sample
                if (self._cancel.is_set() or not fresh_direction or
                        time.monotonic() - fresh_stamp > INPUT_MAX_AGE_S):
                    break
                actual = self.controller.update_jog_stream(target)
                self._last_motors = actual
                previous_tick = tick
                count += 1
                jog_log.record("UPDATE", IDS[chosen_index], direction, self._speed_value,
                               target, actual)
                other_drift = [(IDS[i], abs(normalize_angle(measured - held)))
                               for i, (measured, held) in enumerate(zip(
                                   actual.as_tuple(), target.as_tuple(), strict=True))
                               if i != chosen_index]
                drifting_id, drift_deg = max(other_drift, key=lambda item: item[1])
                held_drift_samples = (held_drift_samples + 1
                                      if drift_deg > MAX_HELD_DRIFT_DEG else 0)
                if held_drift_samples >= 3:
                    raise ControllerError(
                        f"ID{drifting_id} drifted {drift_deg:.2f}° while held; "
                        "arm motion stopped. Check torque, supply, and jog CSV")
                if count % 4 == 0:
                    motor = actual.as_tuple()[chosen_index]
                    joint = motor_to_fk_angles(actual).as_tuple()[chosen_index]
                    command = target.as_tuple()[chosen_index]
                    self.after(0, lambda m=motor, q=joint, c=command: self.feedback.set(
                        f"ID{IDS[chosen_index]} commanded: {c:.2f}°   "
                        f"measured: {m:.2f}°   joint: {q:+.2f}°"))
                    held_text = ", ".join(
                        f"ID{motor_id}: {drift:.2f}° drift"
                        for motor_id, drift in other_drift)
                    self.after(0, lambda message=held_text: self.hold_status.set(
                        f"Other joints held: {message}"))
                sign = math.copysign(1.0, direction)
                if sign != direction_sign:
                    direction_sign = sign
                    initial_angle = actual.as_tuple()[chosen_index]
                    direction_start = tick
                motion = normalize_angle(actual.as_tuple()[chosen_index] - initial_angle)
                if tick - direction_start >= 1.5:
                    commanded = normalize_angle(target.as_tuple()[chosen_index] - initial_angle)
                    if motion * sign < -0.7:
                        raise ControllerError(
                            f"ID{IDS[chosen_index]} moved opposite the stick; inspect the jog CSV")
                    if commanded * sign >= 2.0 and motion * sign < 0.25:
                        raise ControllerError(
                            f"ID{IDS[chosen_index]} did not follow its command; "
                            "check torque, power, and jog CSV")
                    initial_angle = actual.as_tuple()[chosen_index]
                    direction_start = tick
                self._cancel.wait(max(0.0, STREAM_PERIOD_S - (time.monotonic() - tick)))
        except (ControllerError, KinematicsError, ValueError, OSError) as exc:
            self.armed = False
            self._cancel.set()
            self.after(0, lambda message=str(exc): self.status.set(
                f"Motor movement stopped: {message}"))
        finally:
            if stream_active:
                try:
                    self.controller.stop_jog_stream()
                    if jog_log is not None and actual is not None and target is not None:
                        jog_log.record("STOP", IDS[chosen_index], 0.0, self._speed_value,
                                       target, actual)
                except (ControllerError, OSError) as exc:
                    self.armed = False
                    self.after(0, lambda message=str(exc): self.status.set(
                        f"Motor STOP failed: {message}"))
            if jog_log is not None:
                jog_log.close()
            self.controller.end_exclusive_motion()
            self.running = False
            if self.armed and self._blocked_direction is None and not self._pending_selection:
                self.after(0, lambda: self.status.set(
                    f"ID{IDS[self.selected_index]} holding; right stick moves selected motor")
                    if self.armed and not self.running else None)

    def _work_xyz(self):
        """Stream checked XYZ goals, holding non-commanded coordinates fixed."""
        stream_active = False
        jog_log = None
        actual = None
        motors = None
        target = None
        count = 0
        held_drift_samples = 0
        velocity = (0.0, 0.0, 0.0)
        previous_tick = None
        try:
            self.controller.begin_exclusive_motion()
            while self.armed and not self._cancel.is_set():
                tick = time.monotonic()
                with self._lock:
                    stamp, _, vector = self._sample
                if tick - stamp > INPUT_MAX_AGE_S or not any(vector):
                    break
                if not stream_active:
                    actual = self.controller.start_jog_stream()
                    stream_active = True
                    motors = actual
                    target = axis_to_rear_xyz(forward_kinematics_physical(actual))
                    previous_tick = tick - STREAM_PERIOD_S
                    jog_log = XYZJogLog()
                    jog_log.record("START", vector, target, motors, actual)
                    self.after(0, lambda path=str(jog_log.path): self.log_path.set(
                        f"XYZ jog log: {path}"))
                elapsed = min(0.2, max(0.001, tick - previous_tick))
                proposed, next_motors, velocity = plan_xyz_jog_step(
                    actual, motors, target, velocity, vector,
                    self._speed_value, elapsed)
                with self._lock:
                    fresh_stamp, _, fresh_vector = self._sample
                if (self._cancel.is_set() or
                        time.monotonic() - fresh_stamp > INPUT_MAX_AGE_S or
                        not any(fresh_vector)):
                    break
                actual = self.controller.update_jog_stream(next_motors)
                motors = next_motors
                target = proposed
                previous_tick = tick
                count += 1
                jog_log.record("UPDATE", vector, target, motors, actual)
                measured = axis_to_rear_xyz(forward_kinematics_physical(actual))
                drift = tuple(abs(a - b) for a, b in zip(
                    (target.x, target.y, target.z),
                    (measured.x, measured.y, measured.z), strict=True))
                held_drift = max((drift[i] for i in range(3) if vector[i] == 0.0),
                                 default=0.0)
                held_drift_samples = (held_drift_samples + 1
                                      if held_drift > JOG_HELD_DRIFT_MM else 0)
                if held_drift_samples >= 3:
                    raise ControllerError(
                        f"A held XYZ axis drifted {held_drift:.1f} mm; "
                        "motion stopped. Check load, torque and XYZ jog log")
                if count % 4 == 0:
                    self._last_motors = actual
                    self.after(0, lambda c=target, m=measured: self.coords_text.set(
                        f"Target XYZ rear=({c.x:.1f}, {c.y:.1f}, {c.z:.1f}) mm | "
                        f"measured=({m.x:.1f}, {m.y:.1f}, {m.z:.1f}) mm"))
                self._cancel.wait(max(0.0, STREAM_PERIOD_S - (time.monotonic() - tick)))
        except (ControllerError, KinematicsError, ValueError, OSError) as exc:
            self.armed = False
            self._cancel.set()
            self.after(0, lambda message=str(exc): self.status.set(
                f"XYZ motion stopped: {message}"))
        finally:
            if stream_active:
                try:
                    self.controller.stop_jog_stream()
                    if jog_log is not None and actual is not None:
                        jog_log.record("STOP", (0.0, 0.0, 0.0), target, motors, actual)
                except (ControllerError, OSError) as exc:
                    self.armed = False
                    self.after(0, lambda message=str(exc): self.status.set(
                        f"XYZ STOP failed: {message}"))
            if jog_log is not None:
                jog_log.close()
            self.controller.end_exclusive_motion()
            self.running = False
