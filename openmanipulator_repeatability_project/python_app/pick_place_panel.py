"""Supervised camera-XY pick/place trials. No Z or ID15 calibration is inferred."""

from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from threading import Event
from tkinter import messagebox, ttk

import config
from experiment_frame import forward_kinematics_physical, inverse_kinematics_physical
from kinematics import (JointAngles, KinematicsError, MotorAngles, XYZ,
                        forward_kinematics, motor_to_fk_angles, normalize_angle,
                        validate_ik_solution, validate_motor_angles)
from rear_reference_frame import axis_to_rear_xyz, rear_to_axis_xyz
from serial_controller import ControllerError, OpenCRController, RobotState


OPEN_BAND_MIN_RAW = 1250
OPEN_BAND_MAX_RAW = 2200
DEFAULT_OPEN_RAW = 1800
OPEN_TARGET_TOLERANCE_RAW = 100
DEFAULT_CLOSE_RAW = 2650
MAX_GRIPPER_TRAVEL_RAW = 1500
MAX_CURRENT_CUTOFF_RAW = 200


def phase_speed(scale: float, phase: str) -> float:
    """Full selected speed for transfer; ease contact and camera transitions."""
    if not math.isfinite(scale) or not 0.25 <= scale <= 2.0:
        raise ValueError("Pick/place speed must be 0.25–2.0")
    if phase in ("LOWER_TO_PICK", "LOWER_TO_PLACE", "LIFT", "RETRACT"):
        return min(scale, 0.65)
    if phase in ("MOVE_TO_WORK", "ALIGN_WORK_FOR_SCAN", "RETURN_TO_SCAN"):
        return min(scale, 0.5)
    return scale


@dataclass(frozen=True)
class CubeTrial:
    source_id: str
    source_x_rear_mm: float
    source_y_mm: float
    destination_x_rear_mm: float
    destination_y_mm: float
    pixel_u: int
    pixel_v: int
    xy_method: str
    captured_at: str


@dataclass(frozen=True)
class PickSettings:
    pick_z_mm: float
    place_z_mm: float
    travel_z_mm: float
    open_raw: int
    close_raw: int
    current_limit_raw: int
    speed_scale: float = config.PICK_PLACE_DEFAULT_SPEED_SCALE


@dataclass(frozen=True)
class PickStep:
    cube_index: int
    phase: str
    motors: tuple[MotorAngles, ...] = ()


def work_planning_state() -> RobotState:
    """Synthetic WORK state for offline route preflight; sends no commands."""
    work = MotorAngles(*config.WORK_MOTOR_DEGREES)
    return RobotState(
        "", None, work, motor_to_fk_angles(work),
        forward_kinematics_physical(work), forward_kinematics(work))


def sampled_reachable_z_intervals(axis_x_mm: float, y_mm: float
                                  ) -> tuple[tuple[int, int], ...]:
    """Diagnostic-only 1 mm samples; never substitute for path preflight."""
    intervals: list[list[int]] = []
    for sampled_z in range(0, 301):
        try:
            inverse_kinematics_physical(axis_x_mm, y_mm, float(sampled_z))
        except KinematicsError:
            continue
        if not intervals or sampled_z > intervals[-1][1] + 1:
            intervals.append([sampled_z, sampled_z])
        else:
            intervals[-1][1] = sampled_z
    return tuple((low, high) for low, high in intervals)


def _format_z_intervals(intervals: tuple[tuple[int, int], ...]) -> str:
    return ", ".join(f"{low}–{high}" for low, high in intervals) or "none"


def minimum_travel_z(pick_z: float, place_z: float, start_z: float) -> float:
    """Keep transfer above contact heights without adding clearance to WORK."""
    return max(start_z, pick_z + 20.0, place_z + 20.0)


def format_arm_target_readback(target: MotorAngles, actual: MotorAngles) -> str:
    """Summarize a failed move without treating a readback as a successful move."""
    return "; ".join(
        f"ID{motor_id} target={goal:.2f}°, read={observed:.2f}°, "
        f"error={abs(normalize_angle(observed - goal)):.2f}°"
        for motor_id, goal, observed in zip(
            (11, 12, 13, 14), target.as_tuple(), actual.as_tuple(), strict=True)
    )


def _segment(start: XYZ, end: XYZ, motors: MotorAngles,
             joints: JointAngles, depth: int = 0
             ) -> tuple[tuple[MotorAngles, ...], MotorAngles, JointAngles]:
    """Plan one smooth firmware move; split only if its joint arc dips too low."""
    distance = math.dist((start.x, start.y, start.z), (end.x, end.y, end.z))
    if distance > 750:
        raise ValueError("One travel leg exceeds 750 mm; split the experiment")
    axis = rear_to_axis_xyz(end)
    result = inverse_kinematics_physical(axis.x, axis.y, axis.z, reference=joints).ik
    validate_ik_solution(result)
    validate_motor_angles(result.motor_angles)
    deltas = tuple(normalize_angle(goal - before) for goal, before in zip(
        result.motor_angles.as_tuple(), motors.as_tuple(), strict=True))
    count = max(2, math.ceil(max(abs(value) for value in deltas) / 2.0))
    minimum_z = min(start.z, end.z) - 5.0
    arc_safe = True
    for n in range(1, count):
        ratio = n / count
        sample = MotorAngles(*(float((before + ratio * delta) % 360.0)
                               for before, delta in zip(
                                   motors.as_tuple(), deltas, strict=True)))
        try:
            validate_motor_angles(sample)
            rear = axis_to_rear_xyz(forward_kinematics_physical(sample))
        except KinematicsError:
            arc_safe = False
            break
        if rear.z < minimum_z:
            arc_safe = False
            break
    if arc_safe:
        return (result.motor_angles,), result.motor_angles, result.joint_angles
    if depth >= 3 or distance < 10.0:
        raise ValueError(
            f"Direct joint arc would dip below the requested TCP height near "
            f"rear XYZ ({end.x:.1f},{end.y:.1f},{end.z:.1f}); change travel Z/path")
    midpoint = XYZ((start.x + end.x) / 2.0,
                   (start.y + end.y) / 2.0,
                   (start.z + end.z) / 2.0)
    first, middle_motors, middle_joints = _segment(start, midpoint, motors, joints, depth + 1)
    second, final_motors, final_joints = _segment(
        midpoint, end, middle_motors, middle_joints, depth + 1)
    return first + second, final_motors, final_joints


def check_destination_reachability(x_rear_mm: float, y_mm: float,
                                   place_z_mm: float, travel_z_mm: float) -> None:
    """Check both required drop poses without claiming physical clearance."""
    for label, z_mm in (("above-place/travel", travel_z_mm),
                        ("release/place", place_z_mm)):
        axis = rear_to_axis_xyz(XYZ(x_rear_mm, y_mm, z_mm))
        try:
            inverse_kinematics_physical(axis.x, axis.y, axis.z)
        except KinematicsError as exc:
            x_within_soft_range = (
                float(config.SOFT_WORKSPACE["x_min_mm"]) <= axis.x <=
                float(config.SOFT_WORKSPACE["x_max_mm"])
            )
            x_explanation = (
                "X is already inside the configured software X range; increasing "
                "that range will not make this Y/Z pose reachable. "
                if x_within_soft_range else ""
            )
            y_within_soft_range = (
                float(config.SOFT_WORKSPACE["y_min_mm"]) <= y_mm <=
                float(config.SOFT_WORKSPACE["y_max_mm"])
            )
            y_explanation = (
                "Y is already inside the configured software Y range; widening "
                "that range will not make this X/Z pose reachable. "
                if y_within_soft_range else ""
            )
            height_explanation = ""
            if label == "above-place/travel":
                # A diagnostic sample, never a replacement motion plan. It
                # distinguishes a reachable low release from an unreachable
                # top-down approach without relaxing any joint limits.
                intervals = sampled_reachable_z_intervals(axis.x, axis.y)
                if intervals:
                    ranges = _format_z_intervals(intervals)
                    work_z = forward_kinematics_physical(
                        MotorAngles(*config.WORK_MOTOR_DEGREES)).z
                    minimum_top_down = minimum_travel_z(0.0, place_z_mm, work_z)
                    height_explanation = (
                        f"At this X/Y, sampled reachable TCP Z intervals "
                        f"from 0–300 mm are {ranges} mm; "
                        f"the top-down route needs at least Z={minimum_top_down:.1f} "
                        f"mm (at least WORK Z and 20 mm above place). Lowering travel Z below that "
                        f"clearance is not a safe workaround. "
                    )
            raise ValueError(
                f"Destination {label} is unreachable: X={x_rear_mm:.1f} mm from "
                f"ID11 rear (= {axis.x:.1f} mm from ID11 axis), "
                f"Y={y_mm:.1f}, Z={z_mm:.1f} mm. This is a real IK/geometry "
                f"limit, not a camera-calibration error. {x_explanation}{y_explanation}"
                f"{height_explanation}Choose a different "
                f"physically clear drop location or measured Z; do not widen "
                f"joint limits. Detail: {exc}"
            ) from exc


def plan_pick_place(trials: tuple[CubeTrial, ...], settings: PickSettings,
                    state) -> tuple[PickStep, ...]:
    """Preflight direct goals and the firmware's interpolated joint arcs."""
    if not trials:
        raise ValueError("Queue at least one cube")
    phase_speed(settings.speed_scale, "ABOVE_PICK")
    if not all(math.isfinite(value) for value in (
            settings.pick_z_mm, settings.place_z_mm, settings.travel_z_mm)):
        raise ValueError("Pick, place, and travel Z must be finite millimetres")
    required_z = minimum_travel_z(settings.pick_z_mm, settings.place_z_mm,
                                  state.physical_xyz.z)
    if settings.travel_z_mm < required_z:
        raise ValueError(f"Travel Z must clear pick/place TCP heights by 20 mm and "
                         f"be at least the start height; minimum {required_z:.1f} mm")
    current = axis_to_rear_xyz(state.physical_xyz)
    motors = state.motors
    joints = state.joints
    plan: list[PickStep] = []

    def move(index: int, phase: str, target: XYZ) -> None:
        nonlocal current, motors, joints
        try:
            path, motors, joints = _segment(current, target, motors, joints)
        except KinematicsError as exc:
            axis = rear_to_axis_xyz(target)
            phase_explanation = ""
            if phase == "ABOVE_PICK":
                ranges = _format_z_intervals(
                    sampled_reachable_z_intervals(axis.x, axis.y))
                phase_explanation = (
                    "This is the PICK approach above the detected cube, not "
                    "the destination. Changing destination Y cannot fix this "
                    f"pick-side point. Sampled reachable TCP Z intervals at "
                    f"the cube X/Y (0–300 mm scan): {ranges} mm. "
                )
                remedy = ("Reposition the cube or use a different measured "
                          "travel height only after verifying real object and "
                          "path clearance; then preview again. ")
            elif phase == "ABOVE_PLACE":
                phase_explanation = (
                    "This is the PLACE approach above the destination. "
                )
                remedy = "Change the destination or measured Z, then preview again. "
            else:
                remedy = "Change the measured pose/path, then preview again. "
            raise ValueError(
                f"Cube {index} {phase}: requested rear-frame XYZ "
                f"({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) mm "
                f"= ID11-axis XYZ ({axis.x:.1f}, {axis.y:.1f}, {axis.z:.1f}) mm "
                f"cannot be reached by the configured IK/path. {phase_explanation}The failing point "
                f"may be between the start and target. {remedy}Detail: {exc}"
            ) from exc
        plan.append(PickStep(index, phase, path))
        current = target

    for index, trial in enumerate(trials, 1):
        coordinates = (trial.source_x_rear_mm, trial.source_y_mm,
                       trial.destination_x_rear_mm, trial.destination_y_mm)
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError(f"Cube {index} has a non-finite X/Y coordinate")
        plan.append(PickStep(index, "OPEN_BEFORE_PICK"))
        move(index, "RAISE_TO_TRAVEL", XYZ(current.x, current.y, settings.travel_z_mm))
        move(index, "ABOVE_PICK", XYZ(trial.source_x_rear_mm, trial.source_y_mm,
                                      settings.travel_z_mm))
        move(index, "LOWER_TO_PICK", XYZ(trial.source_x_rear_mm, trial.source_y_mm,
                                         settings.pick_z_mm))
        plan.append(PickStep(index, "CLOSE_AND_CONFIRM"))
        move(index, "LIFT", XYZ(trial.source_x_rear_mm, trial.source_y_mm,
                                settings.travel_z_mm))
        move(index, "ABOVE_PLACE", XYZ(trial.destination_x_rear_mm,
                                       trial.destination_y_mm, settings.travel_z_mm))
        move(index, "LOWER_TO_PLACE", XYZ(trial.destination_x_rear_mm,
                                          trial.destination_y_mm, settings.place_z_mm))
        plan.append(PickStep(index, "RELEASE"))
        move(index, "RETRACT", XYZ(trial.destination_x_rear_mm,
                                   trial.destination_y_mm, settings.travel_z_mm))
    work_rear = axis_to_rear_xyz(forward_kinematics_physical(
        MotorAngles(*config.WORK_MOTOR_DEGREES)))
    move(len(trials), "RETURN_TO_WORK", work_rear)
    plan.append(PickStep(len(trials), "ALIGN_WORK_FOR_SCAN"))
    plan.append(PickStep(len(trials), "RETURN_TO_SCAN"))
    return tuple(plan)


class PickPlacePanel(ttk.Frame):
    def __init__(self, parent: tk.Widget, *, controller: OpenCRController,
                 vision_panel, log_callback=None, busy_callback=None) -> None:
        super().__init__(parent, padding=8)
        self.controller = controller
        self.vision_panel = vision_panel
        self.log_callback = log_callback
        self.busy_callback = busy_callback
        self.queue: list[CubeTrial] = []
        self.pick_z = tk.StringVar()
        self.place_z = tk.StringVar()
        self.travel_z = tk.StringVar()
        self.destination_x = tk.StringVar()
        self.destination_y = tk.StringVar()
        self.speed_scale = tk.StringVar(value=str(config.PICK_PLACE_DEFAULT_SPEED_SCALE))
        self.pause_after_grasp = tk.BooleanVar(value=True)
        self.open_raw = tk.StringVar(value=str(DEFAULT_OPEN_RAW))
        self.close_raw = tk.StringVar(value=str(DEFAULT_CLOSE_RAW))
        self.current_limit = tk.StringVar(value=str(MAX_CURRENT_CUTOFF_RAW))
        self.gripper_readout = tk.StringVar(value="ID15 RAW not read")
        self.status = tk.StringVar(value="IDLE — no robot motion")
        self.path_verified = tk.BooleanVar(value=False)
        self.stop_requested = Event()
        self.grip_continue = Event()
        self.awaiting_grip_confirmation = Event()
        self._motor_move_active = Event()
        self.running = False
        self._build()

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        canvas = tk.Canvas(self, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)
        body = ttk.Frame(canvas, padding=4)
        body.columnconfigure(0, weight=1)
        window = canvas.create_window((0, 0), window=body, anchor="nw")
        body.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))

        ttk.Label(body, text="Camera X/Y + operator-measured Z — supervised pick/place",
                  font=("TkDefaultFont", 11, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(body, text=(
            "In tab 4, select a VALID cube row, then click 'PICK IT UP (SELECTED CUBE)…' or add it below. "
            "Camera does not measure cube Z. All X values are from ID11 rear. "
            "Enter measured TCP heights and ID15 RAW limits; confirm the shown ID15 values on your hardware. "
            "Start at CENTER SCAN. Motion uses smooth firmware interpolation with direct target poses."),
            wraplength=950, justify="left").grid(row=1, column=0, sticky="ew", pady=(2, 8))

        queue_box = ttk.LabelFrame(body, text="1  Cube order and destinations", padding=6)
        queue_box.grid(row=2, column=0, sticky="ew", pady=4)
        self.table = ttk.Treeview(queue_box, columns=("Order", "Cube", "Pick X rear", "Pick Y",
                                                      "Place X rear", "Place Y", "XY source"),
                                  show="headings", height=5)
        for col in self.table["columns"]:
            self.table.heading(col, text=col)
            self.table.column(col, width=115, anchor="center", stretch=True)
        self.table.grid(row=0, column=0, columnspan=8, sticky="ew")
        queue_scroll = ttk.Scrollbar(queue_box, orient="vertical", command=self.table.yview)
        queue_scroll.grid(row=0, column=8, sticky="ns")
        self.table.configure(yscrollcommand=queue_scroll.set)
        queue_box.columnconfigure(0, weight=1)
        ttk.Label(queue_box, text="Destination X rear / Y (mm):").grid(row=1, column=0, sticky="w")
        ttk.Entry(queue_box, textvariable=self.destination_x, width=9).grid(row=1, column=1)
        ttk.Entry(queue_box, textvariable=self.destination_y, width=9).grid(row=1, column=2)
        ttk.Button(queue_box, text="ADD SELECTED CUBE", command=self.add_cube).grid(row=1, column=3, padx=3)
        ttk.Button(queue_box, text="MOVE UP", command=self.move_up).grid(row=1, column=4, padx=3)
        ttk.Button(queue_box, text="REMOVE", command=self.remove_cube).grid(row=1, column=5, padx=3)

        heights = ttk.LabelFrame(body, text="2  Measured TCP Z heights & motion speed", padding=6)
        heights.grid(row=3, column=0, sticky="ew", pady=4)
        for col, label, variable in ((0, "Pick TCP Z (mm)", self.pick_z),
                                     (2, "Place TCP Z (mm)", self.place_z),
                                     (4, "Travel TCP Z (mm)", self.travel_z),
                                     (6, f"Speed (0.25–{config.PICK_PLACE_MAXIMUM_SPEED_SCALE:g})", self.speed_scale)):
            ttk.Label(heights, text=label).grid(row=0, column=col, sticky="w", padx=3)
            if col != 6:
                ttk.Entry(heights, textvariable=variable, width=9).grid(row=0, column=col + 1, padx=3)
        ttk.Spinbox(heights, from_=0.25, to=2.0, increment=0.1,
                    textvariable=self.speed_scale, width=9).grid(row=0, column=7, padx=3)
        ttk.Label(heights, text="1× normal; 2× faster travel. Contact moves stay slower. Travel Z is editable.").grid(
            row=1, column=0, columnspan=6, sticky="w", pady=4)
        ttk.Button(heights, text="CHECK DESTINATION", command=self.check_destination).grid(
            row=1, column=6, columnspan=2, padx=3)

        grip = ttk.LabelFrame(body, text="3  ID15 gripper calibration — RAW encoder counts", padding=6)
        grip.grid(row=4, column=0, sticky="ew", pady=4)
        for col, label, variable in ((0, "Open target RAW", self.open_raw),
                                     (2, "Cube grasp RAW", self.close_raw),
                                     (4, "Current cutoff RAW (max 200)", self.current_limit)):
            ttk.Label(grip, text=label).grid(row=0, column=col, sticky="w", padx=3)
            ttk.Entry(grip, textvariable=variable, width=10).grid(row=0, column=col + 1, padx=3)
        ttk.Button(grip, text="READ ID15 RAW", command=self.read_gripper).grid(row=1, column=0,
                                                                                 columnspan=2, sticky="w", pady=4)
        ttk.Label(grip, textvariable=self.gripper_readout).grid(row=1, column=2, columnspan=4, sticky="w")
        ttk.Label(grip, text="Open target may be 1250–2200 RAW; 1800 is the shorter-open default. "
                  "At each pick the jaw is moved to the chosen opening. Current cutoff is a sensor value, "
                  "NOT the jaw position; confirm the grip visually before lifting.",
                  wraplength=950).grid(row=2, column=0, columnspan=6, sticky="w")

        actions = ttk.LabelFrame(body, text="4  Preflight, run, and stop", padding=6)
        actions.grid(row=5, column=0, sticky="ew", pady=4)
        ttk.Checkbutton(actions, text="I measured Z/gripper limits and verified path is clear",
                        variable=self.path_verified).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(actions, text="Pause after grasp for manual confirmation",
                        variable=self.pause_after_grasp).grid(row=0, column=2, columnspan=2, sticky="w")
        ttk.Button(actions, text="PREVIEW PATH (NO MOTION)", command=self.preview_plan).grid(
            row=1, column=0, sticky="ew", padx=3, pady=4)
        ttk.Button(actions, text="RUN SUPERVISED PICK/PLACE", command=self.start).grid(
            row=1, column=1, sticky="ew", padx=3, pady=4)
        ttk.Button(actions, text="CONFIRM GRIP & CONTINUE", command=self.confirm_grip).grid(
            row=1, column=2, sticky="ew", padx=3, pady=4)
        ttk.Button(actions, text="STOP", command=self.stop).grid(
            row=1, column=3, sticky="ew", padx=3, pady=4)
        ttk.Label(body, textvariable=self.status, wraplength=950, justify="left").grid(
            row=6, column=0, sticky="ew", pady=(6, 0))

    def _refresh_queue(self) -> None:
        for row in self.table.get_children():
            self.table.delete(row)
        for n, trial in enumerate(self.queue, 1):
            self.table.insert("", "end", iid=str(n - 1), values=(
                n, trial.source_id, f"{trial.source_x_rear_mm:.1f}",
                f"{trial.source_y_mm:.1f}", f"{trial.destination_x_rear_mm:.1f}",
                f"{trial.destination_y_mm:.1f}", trial.xy_method))

    def add_cube(self) -> None:
        try:
            if self.running:
                raise ValueError("Stop the experiment before editing the queue")
            x, y = float(self.destination_x.get()), float(self.destination_y.get())
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("Destination X/Y must be finite millimetres")
            item = self.vision_panel.capture_selected_cube()
            if any(math.dist((item["x_rear_mm"], item["y_mm"]),
                             (old.source_x_rear_mm, old.source_y_mm)) < 15.0 for old in self.queue):
                raise ValueError("This cube or a nearby cube is already queued")
            self.queue.append(CubeTrial(
                item["object_id"], item["x_rear_mm"], item["y_mm"], x, y,
                item["pixel_u"], item["pixel_v"], item["xy_method"], item["captured_at"]))
            self._refresh_queue()
            self.status.set(f"Queued {len(self.queue)} cube(s). Order is top to bottom.")
        except (ControllerError, ValueError) as exc:
            messagebox.showerror("Pick queue", str(exc))

    def _selected_index(self) -> int:
        selected = self.table.selection()
        if len(selected) != 1:
            raise ValueError("Select one queued cube")
        return int(selected[0])

    def move_up(self) -> None:
        try:
            if self.running:
                raise ValueError("Cannot change the order while running")
            index = self._selected_index()
            if index > 0:
                self.queue[index - 1], self.queue[index] = self.queue[index], self.queue[index - 1]
                self._refresh_queue()
                self.table.selection_set(str(index - 1))
        except ValueError as exc:
            messagebox.showerror("Pick queue", str(exc))

    def remove_cube(self) -> None:
        try:
            if self.running:
                raise ValueError("Cannot change the queue while running")
            self.queue.pop(self._selected_index())
            self._refresh_queue()
        except ValueError as exc:
            messagebox.showerror("Pick queue", str(exc))

    def read_gripper(self) -> None:
        try:
            raw = self.controller.read_gripper_raw()
            self.gripper_readout.set(f"ID15 present RAW = {raw}")
        except ControllerError as exc:
            messagebox.showerror("ID15 read", str(exc))

    def quick_pick_dialog(self, after_queue=None) -> None:
        """Modal dialog from camera tab: collects destination XY and 3 Z values, then queues or runs."""
        if self.running:
            messagebox.showwarning(
                "Pick/Place Busy",
                "A pick/place experiment is currently running. Stop it before queuing a new cube.",
                parent=self.winfo_toplevel(),
            )
            return

        try:
            item = self.vision_panel.capture_selected_cube()
        except (ControllerError, ValueError) as exc:
            messagebox.showwarning(
                "Pick Selected Cube",
                f"Cannot capture selected cube from camera:\n\n{exc}\n\n"
                "Tip: Ensure the camera is streaming, DETECT is enabled, and a valid cube row is selected in the table.",
                parent=self.winfo_toplevel(),
            )
            return

        dialog = tk.Toplevel(self.winfo_toplevel())
        dialog.title(f"Quick Pick & Place — {item['object_id']}")
        dialog.transient(self.winfo_toplevel())
        dialog.grab_set()
        dialog.resizable(False, False)

        frame = ttk.Frame(dialog, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")

        # 1. Detected Cube Information
        info_box = ttk.LabelFrame(frame, text="1  Detected Cube Location (Camera)", padding=8)
        info_box.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        info_text = (
            f"Cube: {item['object_id']}   |   XY Method: {item.get('xy_method', 'CALIBRATED')}\n"
            f"Pick X (ID11 rear): {item['x_rear_mm']:.1f} mm   |   Pick Y: {item['y_mm']:.1f} mm\n"
            f"Detected pixel center: (u={item['pixel_u']}, v={item['pixel_v']})"
        )
        ttk.Label(info_box, text=info_text, justify="left", font=("TkDefaultFont", 9, "bold")).grid(
            row=0, column=0, sticky="w", padx=4, pady=2
        )

        # 2. Destination Coordinates (mm)
        dest_box = ttk.LabelFrame(frame, text="2  Destination Position (mm)", padding=8)
        dest_box.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        cur_dx = self.destination_x.get().strip()
        cur_dy = self.destination_y.get().strip()
        var_dx = tk.StringVar(value=cur_dx)
        var_dy = tk.StringVar(value=cur_dy)

        ttk.Label(dest_box, text="Destination X rear (mm):").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(dest_box, textvariable=var_dx, width=12).grid(row=0, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(dest_box, text="Destination Y (mm):").grid(row=0, column=2, sticky="w", padx=(16, 4), pady=2)
        ttk.Entry(dest_box, textvariable=var_dy, width=12).grid(row=0, column=3, sticky="w", padx=4, pady=2)

        # 3. Measured Z Heights (mm)
        z_box = ttk.LabelFrame(frame, text="3  Measured TCP Z Heights (Robot frame mm)", padding=8)
        z_box.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        cur_pz = self.pick_z.get().strip()
        cur_plz = self.place_z.get().strip()
        cur_tz = self.travel_z.get().strip()
        var_pz = tk.StringVar(value=cur_pz)
        var_plz = tk.StringVar(value=cur_plz)
        var_tz = tk.StringVar(value=cur_tz)

        ttk.Label(z_box, text="Pick TCP Z (mm):").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(z_box, textvariable=var_pz, width=10).grid(row=0, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(z_box, text="Place TCP Z (mm):").grid(row=0, column=2, sticky="w", padx=(16, 4), pady=2)
        ttk.Entry(z_box, textvariable=var_plz, width=10).grid(row=0, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(z_box, text="Travel TCP Z (mm):").grid(row=1, column=0, sticky="w", padx=4, pady=(6, 2))
        ttk.Entry(z_box, textvariable=var_tz, width=10).grid(row=1, column=1, sticky="w", padx=4, pady=(6, 2))
        ttk.Label(z_box, text="Speed (1× normal):").grid(row=1, column=2, padx=4, pady=6)
        ttk.Spinbox(z_box, from_=0.25, to=2.0, increment=0.1,
                    textvariable=self.speed_scale, width=10).grid(row=1, column=3, padx=4)

        work_z = forward_kinematics_physical(MotorAngles(*config.WORK_MOTOR_DEGREES)).z
        ttk.Label(
            z_box,
            text=f"Travel Z: at least WORK ({work_z:.1f} mm), and 20 mm above pick/place. Check box clearance.",
            font=("TkDefaultFont", 8, "italic"),
            foreground="#555555",
        ).grid(row=2, column=0, columnspan=4, sticky="w", padx=4, pady=(2, 2))

        # 4. Gripper opening and grasp target
        opt_box = ttk.LabelFrame(frame, text="4  ID15 gripper RAW positions", padding=8)
        opt_box.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        var_open = tk.StringVar(value=self.open_raw.get())
        var_close = tk.StringVar(value=self.close_raw.get())
        ttk.Label(opt_box, text="Open to RAW:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(opt_box, textvariable=var_open, width=9).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(opt_box, text="Close to RAW:").grid(row=0, column=2, sticky="w", padx=(16, 4), pady=2)
        ttk.Entry(opt_box, textvariable=var_close, width=9).grid(row=0, column=3, sticky="w", padx=4, pady=2)
        ttk.Label(opt_box, text="1800 is less open than 1400; 2000 is also selectable. Check cube clearance.",
                  wraplength=480).grid(row=1, column=0, columnspan=4, sticky="w", padx=4)

        def _parse_and_validate():
            try:
                dx = float(var_dx.get())
                dy = float(var_dy.get())
                pz = float(var_pz.get())
                plz = float(var_plz.get())
                tz = float(var_tz.get())
                open_raw = int(var_open.get())
                close_raw = int(var_close.get())
            except ValueError:
                messagebox.showerror(
                    "Invalid Input",
                    "Enter measured destination X/Y and all three Z heights, plus integer gripper RAW positions.",
                    parent=dialog,
                )
                return None

            if not all(math.isfinite(v) for v in (dx, dy, pz, plz, tz)):
                messagebox.showerror("Invalid Input", "All numerical values must be finite.", parent=dialog)
                return None

            if not (OPEN_BAND_MIN_RAW <= open_raw <= OPEN_BAND_MAX_RAW and
                    2000 <= close_raw <= 2700 and
                    0 < close_raw - open_raw <= MAX_GRIPPER_TRAVEL_RAW):
                messagebox.showerror("Invalid gripper RAW",
                                     "Open: 1250–2200; close: 2000–2700; close must exceed open by at most 1500 RAW.",
                                     parent=dialog)
                return None

            min_tz = minimum_travel_z(pz, plz, work_z)
            if tz < min_tz:
                messagebox.showerror(
                    "Travel Z Too Low",
                    f"Travel Z ({tz:.1f} mm) is too low.\n"
                    f"It must be at least WORK ({work_z:.1f} mm) and clear pick ({pz:.1f} mm) "
                    f"and place ({plz:.1f} mm) by 20 mm.\nMinimum required Travel Z is {min_tz:.1f} mm.",
                    parent=dialog,
                )
                return None

            try:
                check_destination_reachability(dx, dy, plz, tz)
            except ValueError as exc:
                messagebox.showerror("Destination cannot be reached", str(exc), parent=dialog)
                return None

            return dx, dy, pz, plz, tz, open_raw, close_raw

        def _apply_settings(dx, dy, pz, plz, tz, open_raw, close_raw):
            self.destination_x.set(f"{dx:.1f}")
            self.destination_y.set(f"{dy:.1f}")
            self.pick_z.set(f"{pz:.1f}")
            self.place_z.set(f"{plz:.1f}")
            self.travel_z.set(f"{tz:.1f}")
            self.open_raw.set(str(open_raw))
            self.close_raw.set(str(close_raw))

        def _on_queue():
            vals = _parse_and_validate()
            if vals is None:
                return
            dx, dy, pz, plz, tz, open_raw, close_raw = vals
            _apply_settings(dx, dy, pz, plz, tz, open_raw, close_raw)

            nearby_idx = None
            for idx, old in enumerate(self.queue):
                if math.dist((item["x_rear_mm"], item["y_mm"]), (old.source_x_rear_mm, old.source_y_mm)) < 15.0:
                    nearby_idx = idx
                    break

            trial = CubeTrial(
                item["object_id"], item["x_rear_mm"], item["y_mm"], dx, dy,
                item["pixel_u"], item["pixel_v"], item["xy_method"], item["captured_at"]
            )
            if nearby_idx is not None:
                self.queue[nearby_idx] = trial
            else:
                self.queue.append(trial)

            self._refresh_queue()
            self.status.set(f"Queued cube {trial.source_id}: pick ({trial.source_x_rear_mm:.1f}, {trial.source_y_mm:.1f}) -> place ({dx:.1f}, {dy:.1f}) mm.")
            dialog.destroy()
            if after_queue:
                after_queue()

        def _on_run_now():
            vals = _parse_and_validate()
            if vals is None:
                return
            dx, dy, pz, plz, tz, open_raw, close_raw = vals
            _apply_settings(dx, dy, pz, plz, tz, open_raw, close_raw)

            trial = CubeTrial(
                item["object_id"], item["x_rear_mm"], item["y_mm"], dx, dy,
                item["pixel_u"], item["pixel_v"], item["xy_method"], item["captured_at"]
            )
            try:
                plan_pick_place((trial,), self._settings(), work_planning_state())
            except (ControllerError, KinematicsError, ValueError) as exc:
                messagebox.showerror(
                    "Full pick/place route is not reachable",
                    f"No robot motion was sent. The complete route for this exact "
                    f"cube, destination, and Z settings failed preflight:\n\n{exc}",
                    parent=dialog)
                return
            self.queue = [trial]
            self._refresh_queue()
            self.path_verified.set(False)
            dialog.destroy()
            if after_queue:
                after_queue()
            if self.preview_plan():
                messagebox.showinfo("Review before robot motion",
                                    "The full route passed offline preflight. Review physical clearance, "
                                    "check the path-verification box, then press RUN in tab 5.",
                                    parent=self.winfo_toplevel())
            else:
                messagebox.showerror("Robot-pose preflight blocked",
                                     self.status.get(), parent=self.winfo_toplevel())

        # 5. Buttons
        btn_box = ttk.Frame(frame)
        btn_box.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        ttk.Button(btn_box, text="PICK IT UP — REVIEW", command=_on_run_now).pack(side="left", padx=4)
        ttk.Button(btn_box, text="Queue & Go to Tab", command=_on_queue).pack(side="left", padx=4)
        ttk.Button(btn_box, text="Cancel", command=dialog.destroy).pack(side="right", padx=4)

    def check_destination(self) -> None:
        """Report reachability without changing coordinates or commanding motion."""
        try:
            x, y = float(self.destination_x.get()), float(self.destination_y.get())
            place, travel = float(self.place_z.get()), float(self.travel_z.get())
            if not all(math.isfinite(v) for v in (x, y, place, travel)):
                raise ValueError("Enter finite destination X/Y and place/travel Z")
            check_destination_reachability(x, y, place, travel)
            ranges = _format_z_intervals(sampled_reachable_z_intervals(
                rear_to_axis_xyz(XYZ(x, y, 0)).x, y))
            messagebox.showinfo("Destination check",
                f"X={x:g}, Y={y:g}: place Z={place:g} and travel Z={travel:g} have IK solutions.\n"
                f"Sampled reachable heights: {ranges} mm.\n"
                "Use PREVIEW PATH to check the complete route before running.")
        except (ValueError, KinematicsError) as exc:
            messagebox.showerror("Destination check", str(exc))

    def _settings(self) -> PickSettings:
        try:
            spd_text = self.speed_scale.get().strip().split()[0]
            settings = PickSettings(
                float(self.pick_z.get()), float(self.place_z.get()),
                float(self.travel_z.get()), int(self.open_raw.get()),
                int(self.close_raw.get()), int(self.current_limit.get()),
                float(spd_text)
            )
        except ValueError as exc:
            raise ValueError("Enter numeric measured Z, integer ID15 RAW values, and speed (0.25–2.0)") from exc
        if not (0.25 <= settings.speed_scale <= config.PICK_PLACE_MAXIMUM_SPEED_SCALE):
            raise ValueError(
                f"Pick/place speed must be 0.25–{config.PICK_PLACE_MAXIMUM_SPEED_SCALE:g}"
            )
        if not OPEN_BAND_MIN_RAW <= settings.open_raw <= OPEN_BAND_MAX_RAW:
            raise ValueError("Open target must be in the firmware 1250–2200 RAW range")
        if not 2000 <= settings.close_raw <= 2700 or not 0 < settings.close_raw - settings.open_raw <= MAX_GRIPPER_TRAVEL_RAW:
            raise ValueError("Cube grasp target must be 2000–2700 RAW and at most 1500 counts from open")
        if not 1 <= settings.current_limit_raw <= MAX_CURRENT_CUTOFF_RAW:
            raise ValueError("Current cutoff is not ID15 position: use a measured value of 1–200 current RAW, not 1200 position RAW")
        return settings

    def _prepare(self):
        if self.busy_callback is not None:
            reason = self.busy_callback()
            if reason:
                raise ValueError(f"Another experiment is active: {reason}")
        if self.vision_panel.scan_running:
            raise ValueError("Stop the camera scan before a pick experiment")
        if not self.controller.status.connected or not self.controller.status.torque_on:
            raise ControllerError("Connect and turn torque ON before running")
        self.controller.require_gripper_capability()
        if not self.queue:
            raise ValueError("Add at least one camera-detected cube")
        settings = self._settings()
        state = self.controller.read_robot_state()
        work = MotorAngles(*config.WORK_MOTOR_DEGREES)
        scan = MotorAngles(*config.SCAN_MOTOR_DEGREES)
        def near(target: MotorAngles, tol: float = 5.0) -> bool:
            return max(abs(normalize_angle(a - b)) for a, b in zip(
                state.motors.as_tuple(), target.as_tuple(), strict=True)) <= tol
        if not near(scan):
            raise ValueError("Start the experiment at CENTER SCAN; move there and read encoders first")
        # Cartesian pickup planning begins at WORK, never from an improvised camera path.
        planning_state = work_planning_state()
        body = plan_pick_place(tuple(self.queue), settings, planning_state)
        planned = (PickStep(1, "OPEN_BEFORE_PICK"),
                   PickStep(1, "MOVE_TO_WORK")) + body[1:]
        return settings, state, planned

    def preview_plan(self) -> bool:
        try:
            settings, _state, planned = self._prepare()
            moves = sum(len(step.motors) if step.motors else
                        int(step.phase in ("MOVE_TO_WORK", "ALIGN_WORK_FOR_SCAN", "RETURN_TO_SCAN"))
                        for step in planned)
            transition = "CENTER SCAN → WORK → pick/place → WORK → CENTER SCAN; "
            self.status.set(
                f"PREFLIGHT ONLY: {transition}{len(self.queue)} cube(s), {moves} smooth arm moves "
                f"(speed={settings.speed_scale:.2f}x), joint arcs checked; pick Z={settings.pick_z_mm:g}, "
                f"place Z={settings.place_z_mm:g}, travel Z={settings.travel_z_mm:g} mm. "
                "No motion sent. Inspect the entire physical path before RUN.")
            return True
        except (ControllerError, KinematicsError, ValueError) as exc:
            self.status.set(f"PREFLIGHT BLOCKED: {exc}")
            return False

    def start(self) -> None:
        try:
            if self.running:
                raise ValueError("Pick/place is already running")
            if not self.path_verified.get():
                raise ValueError("Measure Z and ID15 limits, then check the verified-path box")
            settings, _state, planned = self._prepare()
            pause_desc = "Run pauses after each grasp for visual confirmation."
            if not messagebox.askyesno(
                    "Confirm pick/place motion",
                    f"This WILL MOVE ID11–ID15 for {len(self.queue)} cube(s).\n"
                    f"Camera gives XY only; your entered Z is used.\n"
                    f"ID11 rear offset {config.REAR_TO_AXIS_X_MM:g} mm is provisional.\n"
                    f"Motion speed scale: {settings.speed_scale:.2f}x.\n"
                    f"{pause_desc}\n\n"
                    "Workspace clear, emergency stop ready, and all settings measured?"):
                return
            self.stop_requested.clear()
            self.grip_continue.clear()
            self.running = True
            self.status.set("STARTING — keep hands clear; STOP is available")
            threading.Thread(target=self._worker,
                             args=(tuple(self.queue), settings, planned,
                                   self.pause_after_grasp.get()), daemon=True).start()
        except (ControllerError, KinematicsError, ValueError) as exc:
            messagebox.showerror("Pick/place preflight", str(exc))

    def confirm_grip(self) -> None:
        if self.running and self.awaiting_grip_confirmation.is_set():
            self.grip_continue.set()

    def stop(self) -> None:
        self.stop_requested.set()
        self.grip_continue.set()
        if self.running and self._motor_move_active.is_set():
            try:
                self.controller.stop()
            except ControllerError as exc:
                self.status.set(f"STOP requested; serial STOP failed: {exc}")

    def _move_gripper(self, target_raw: int, current_limit_raw: int):
        if self.stop_requested.is_set():
            raise ControllerError("Stopped by operator")
        self._motor_move_active.set()
        try:
            return self.controller.move_gripper(target_raw, current_limit_raw)
        finally:
            self._motor_move_active.clear()

    def _move_arm(self, motors: MotorAngles, phase: str) -> None:
        if self.stop_requested.is_set():
            raise ControllerError("Stopped by operator")
        self._motor_move_active.set()
        try:
            try:
                self.controller.move_motor_angles(motors)
            except ControllerError as exc:
                # A firmware TARGET_NOT_REACHED is a real failed motion, not
                # an IK preflight failure. Read once for diagnosis; never
                # retry or continue to grasp after a tracking failure.
                if "TARGET_NOT_REACHED" not in str(exc) and "Motor target not reached" not in str(exc):
                    raise
                try:
                    snapshot = self.controller.read_telemetry()
                    readback = format_arm_target_readback(motors, snapshot.state.motors)
                    base = snapshot.motors[0]
                    diagnostic = (
                        f"{readback}; ID11 telemetry: current={base.current_raw} RAW, "
                        f"PWM={base.pwm_raw} RAW, velocity={base.velocity_raw} RAW, "
                        f"voltage={base.voltage_v:.1f} V, temperature={base.temperature_c} C, "
                        f"hardware_error={base.hardware_error}"
                    )
                except (ControllerError, OSError, ValueError) as diag_exc:
                    try:
                        actual = self.controller.read_motor_angles()
                        diagnostic = format_arm_target_readback(motors, actual)
                    except (ControllerError, OSError, ValueError):
                        diagnostic = f"encoder readback unavailable ({diag_exc})"
                raise ControllerError(
                    f"{phase}: {exc}. {diagnostic}. "
                    "Pick/place sequence aborted; no automatic retry. The servo may "
                    "still be holding its last goal. Inspect the physical arm before "
                    "another command"
                ) from exc
            errors = getattr(self.controller, "last_motor_tracking_errors", ())
            if errors and max(errors) > 5.0:
                raise ControllerError(
                    f"Pick/place stopped: motor tracking error {max(errors):.2f}° exceeds 5°")
            elif errors and max(errors) > 2.0:
                self._post(f"Tracking note: motor tracking error {max(errors):.2f}°")
        finally:
            self._motor_move_active.clear()

    def _post(self, text: str) -> None:
        self.after(0, lambda: self.status.set(text))
        if self.log_callback:
            self.after(0, lambda: self.log_callback(f"Pick/place: {text}"))

    def _worker(self, trials: tuple[CubeTrial, ...], settings: PickSettings,
                planned: tuple[PickStep, ...], pause_after_grasp: bool) -> None:
        try:
            self.controller.begin_exclusive_motion()
            self.controller.set_motion_speed(settings.speed_scale)
            self._post(f"Configuring ID15: open={settings.open_raw}, grasp={settings.close_raw}, "
                       f"current cutoff={settings.current_limit_raw} RAW | Speed={settings.speed_scale:.2f}x")
            self.controller.configure_gripper(settings.open_raw, settings.close_raw)
            for step in planned:
                if self.stop_requested.is_set():
                    raise ControllerError("Stopped by operator")
                trial = trials[step.cube_index - 1]
                if step.motors or step.phase in ("MOVE_TO_WORK", "ALIGN_WORK_FOR_SCAN", "RETURN_TO_SCAN"):
                    self.controller.set_motion_speed(phase_speed(settings.speed_scale, step.phase))
                self._post(f"Cube {step.cube_index}/{len(trials)}: {step.phase}")
                if step.phase == "OPEN_BEFORE_PICK":
                    current = self.controller.read_gripper_raw()
                    if abs(current - settings.open_raw) > OPEN_TARGET_TOLERANCE_RAW:
                        result = self._move_gripper(
                            settings.open_raw, settings.current_limit_raw)
                        if result.outcome != "DONE":
                            raise ControllerError("ID15 hit current limit while opening")
                    verified = self.controller.read_gripper_raw()
                    if abs(verified - settings.open_raw) > OPEN_TARGET_TOLERANCE_RAW:
                        raise ControllerError(f"ID15 not near requested open RAW {settings.open_raw}: RAW {verified}")
                    self._post(f"Cube {step.cube_index}: ID15 open RAW {verified}")
                elif step.phase == "MOVE_TO_WORK":
                    self._motor_move_active.set()
                    try:
                        self.controller.move_work()
                        errors = getattr(self.controller, "last_motor_tracking_errors", ())
                        if errors and max(errors) > 5.0:
                            raise ControllerError(
                                f"WORK tracking error {max(errors):.2f}° exceeds 5°")
                    finally:
                        self._motor_move_active.clear()
                elif step.phase == "CLOSE_AND_CONFIRM":
                    timed_out = False
                    try:
                        result = self._move_gripper(
                            settings.close_raw, settings.current_limit_raw)
                        outcome = result.outcome
                        position_raw = result.position_raw
                    except ControllerError as exc:
                        if "GRIPPER_TIMEOUT" not in str(exc):
                            raise
                        # OpenCR holds ID15 at its last measured position on a
                        # timeout. This is NOT evidence of a secure grasp.
                        position_raw = self.controller.read_gripper_raw()
                        outcome = "TIMEOUT / HELD"
                        timed_out = True
                    if timed_out or pause_after_grasp:
                        self.grip_continue.clear()
                        self.awaiting_grip_confirmation.set()
                        warning = ("The close target was not reached and the current cutoff did not fire. "
                                   "Only continue if the cube is visibly held; otherwise STOP and "
                                   "measure the jaw RAW/current settings. " if timed_out else "")
                        self._post(f"PAUSED AFTER GRASP cube {step.cube_index}: ID15 {outcome} "
                                   f"at RAW {position_raw}. {warning}Inspect the grip, then click "
                                   "CONFIRM GRIP & CONTINUE or STOP.")
                        while not self.grip_continue.wait(0.1):
                            if self.stop_requested.is_set():
                                raise ControllerError("Stopped before lifting")
                        if self.stop_requested.is_set():
                            raise ControllerError("Stopped before lifting")
                        self.awaiting_grip_confirmation.clear()
                    else:
                        self._post(f"GRASPED cube {step.cube_index}: ID15 {outcome} "
                                   f"at RAW {position_raw}. Continuing lift...")
                        time.sleep(config.PICK_PLACE_POST_GRASP_SETTLE_SECONDS)
                elif step.phase == "RELEASE":
                    result = self._move_gripper(
                        settings.open_raw, settings.current_limit_raw)
                    time.sleep(config.PICK_PLACE_POST_RELEASE_SETTLE_SECONDS)
                    verified = self.controller.read_gripper_raw()
                    if result.outcome != "DONE" or abs(verified - settings.open_raw) > OPEN_TARGET_TOLERANCE_RAW:
                        raise ControllerError(f"ID15 did not reach release opening {settings.open_raw}: RAW {verified}")
                elif step.phase in ("ALIGN_WORK_FOR_SCAN", "RETURN_TO_SCAN"):
                    self._motor_move_active.set()
                    try:
                        if step.phase == "ALIGN_WORK_FOR_SCAN":
                            self.controller.move_work()
                        else:
                            self.controller.move_scan()
                    finally:
                        self._motor_move_active.clear()
                else:
                    for motors in step.motors:
                        if self.stop_requested.is_set():
                            raise ControllerError("Stopped by operator")
                        self._move_arm(motors, step.phase)
            self._post(f"COMPLETE at CENTER SCAN: {len(trials)} cube(s). Verify physical placement; "
                       "no trajectory or trial CSV was saved.")
        except (ControllerError, KinematicsError, OSError, ValueError) as exc:
            self._post(f"STOPPED/FAILED: {exc}. Do not assume the cube is released or the arm is clear.")
        finally:
            self.awaiting_grip_confirmation.clear()
            self.controller.end_exclusive_motion()
            self.after(0, self._finished)

    def _finished(self) -> None:
        self.running = False
        self.path_verified.set(False)
