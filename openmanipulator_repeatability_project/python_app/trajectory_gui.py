"""GUI for teach-by-hand recording and Cartesian XYZ replay."""

from __future__ import annotations

import math
import threading
import time
from threading import Event
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import config
from experiment_logging import ExperimentLogger, new_session_id
from experiment_frame import (
    experiment_points,
    forward_kinematics_physical,
    inverse_kinematics_physical,
)
from kinematics import (
    KinematicsError,
    XYZ,
    forward_kinematics,
    motor_to_fk_angles,
    normalize_angle,
    validate_ik_solution,
)
from point_experiment import (
    MEASUREMENT_STOP,
    MEASUREMENT_WARNING,
    ExperimentPlan,
    plan_named_experiment_points,
    plan_point_experiment,
    tcp_error_severity,
)
from repeatability_experiment import (
    ManualMeasurement,
    RepeatabilityRunConfig,
    RepeatabilityWorkbook,
    TouchRecord,
)
from serial_controller import ControllerError, OpenCRController, available_ports
from trajectory import (
    TrajectoryPoint,
    XYZTrajectoryPoint,
    endpoint_points,
    load_xyz_workbook,
    save_manual_poses,
    save_workbooks,
    validate_xyz_trajectory,
    xyz_endpoint_points,
)


class TrajectoryGUI(ttk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padding=12)
        self.controller = OpenCRController()
        self.port = tk.StringVar()
        self.torque = tk.StringVar(value="UNKNOWN")
        self.status = tk.StringVar(value="Disconnected")
        self.recording = tk.StringVar(value="OFF")
        self.playback = tk.StringVar(value="OFF")
        self.live_read_status = tk.StringVar(value="OFF")
        self.live_interval_ms = tk.StringVar(value=str(config.TELEMETRY_INTERVAL_MS))
        self.experiment_status = tk.StringVar(value="IDLE")
        self.elapsed = tk.StringVar(value="0.000 s")
        self.count = tk.StringVar(value="0")
        self.speed = tk.StringVar(value="0.25")
        self.target_xyz = {
            "X": tk.StringVar(value=f"{config.WORK_XYZ_MM[0]:.1f}"),
            "Y": tk.StringVar(value="0.0"),
            "Z": tk.StringVar(value=f"{config.WORK_XYZ_MM[2]:.1f}"),
        }
        self.ik_preview = tk.StringVar(value="Enter XYZ, then preview before moving")
        self.selected_experiment_point = tk.StringVar(value="P04")
        self.repeatability_repetitions = tk.StringVar(value=str(config.DEFAULT_REPETITIONS))
        self.repeatability_sample_count = tk.StringVar(value=str(config.TOUCH_SAMPLE_COUNT))
        self.repeatability_sample_interval_ms = tk.StringVar(
            value=str(round(config.TOUCH_SAMPLE_INTERVAL_SECONDS * 1000.0))
        )
        self.pause_for_manual_measurement = tk.BooleanVar(
            value=config.PAUSE_FOR_MANUAL_MEASUREMENT
        )
        self.experiment_point_selected = {
            name: tk.BooleanVar(value=name == "P04") for name in experiment_points()
        }
        self.manual_measurement_values = {
            axis: tk.StringVar(value="") for axis in ("X", "Y", "Z")
        }
        self.manual_distance_error = tk.StringVar(value="")
        self.manual_measurement_note = tk.StringVar(value="")
        self.manual_measurement_status = tk.StringVar(
            value="No repeatability touch is waiting for measurement"
        )
        self._workspace_highlight: str | None = None
        self.manual_xyz = {axis: tk.StringVar(value="---") for axis in ("X", "Y", "Z")}
        self.manual_angles = {motor_id: tk.StringVar(value="---") for motor_id in (11, 12, 13, 14)}
        self.manual_joints = {joint: tk.StringVar(value="---") for joint in ("q1", "q2", "q3", "q4")}
        self.points = []
        self.manual_points: list[TrajectoryPoint] = []
        self.xyz_points: list[XYZTrajectoryPoint] = []
        self._play_stop = Event()
        self._telemetry_enabled = False
        self._telemetry_inflight = False
        self._telemetry_failures = 0
        self._telemetry_after_id = None
        self._experiment_stop = Event()
        self._manual_measurement_continue = Event()
        self._manual_measurement_payload: ManualMeasurement | None = None
        self._manual_measurement_skipped = False
        self._pending_touch_record: TouchRecord | None = None
        self._repeatability_workbook: RepeatabilityWorkbook | None = None
        self.repeatability_window: tk.Toplevel | None = None
        self.output_dir = Path(__file__).resolve().parents[1]
        self.logger = ExperimentLogger(project_root=self.output_dir)
        self._experiment_plan: ExperimentPlan | None = None
        self._experiment_running = False
        self._experiment_motion_active = False
        self._telemetry_target = None
        self._telemetry_sample_index = 0
        self._session_started = time.monotonic()
        self._build()
        self._refresh_ports()
        self.master.protocol("WM_DELETE_WINDOW", self._close)
        self.logger.log_history(
            "gui_session_started",
            category="session",
            context={"log_directory": str(self.logger.session_dir)},
        )
        self._log(f"Session logs: {self.logger.session_dir}", persist=False)

    def _build(self) -> None:
        self.grid(sticky="nsew")
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)
        self._connection().grid(row=0, column=0, sticky="ew", pady=(0, 4))

        vertical = ttk.Panedwindow(self, orient=tk.VERTICAL)
        vertical.grid(row=1, column=0, sticky="nsew")

        upper = ttk.Panedwindow(vertical, orient=tk.HORIZONTAL)
        upper.add(self._controls(upper), weight=3)
        upper.add(self._replay(upper), weight=2)
        vertical.add(upper, weight=4)
        vertical.add(self._workspace_preview(vertical), weight=2)
        vertical.add(self._status(vertical), weight=1)

    def _frame(self, title: str, parent=None) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(parent or self, text=title, padding=8)
        frame.columnconfigure(1, weight=1)
        return frame

    def _connection(self) -> ttk.LabelFrame:
        frame = self._frame("Connection")
        ttk.Label(frame, text="COM port").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(frame, textvariable=self.port, width=15)
        self.port_box.grid(row=0, column=1, sticky="ew")
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Button(frame, text="CONNECT", command=self.connect).grid(row=1, column=0, pady=4)
        ttk.Button(frame, text="DISCONNECT", command=self.disconnect).grid(row=1, column=1, pady=4)
        ttk.Button(frame, text="TORQUE ON", command=self.torque_on).grid(row=1, column=2, padx=3)
        ttk.Button(frame, text="TORQUE OFF", command=self.torque_off).grid(row=2, column=2, padx=3)
        ttk.Button(frame, text="REST (STRAIGHT)", command=self.rest).grid(row=2, column=0, pady=4)
        ttk.Button(frame, text="WORK (DOWN)", command=self.work).grid(row=2, column=1, pady=4)
        ttk.Button(frame, text="READ ANGLES NOW", command=self.read_angles_now).grid(row=3, column=0, pady=4)
        ttk.Button(frame, text="START LIVE READ", command=self.start_live_read).grid(row=3, column=1, pady=4)
        ttk.Button(frame, text="STOP LIVE READ", command=self.stop_live_read).grid(row=3, column=2, padx=3)
        ttk.Label(frame, text="Live interval (ms)").grid(row=4, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.live_interval_ms, width=8).grid(row=4, column=1, sticky="w")
        ttk.Label(frame, textvariable=self.live_read_status).grid(row=4, column=2, sticky="w")
        return frame

    def _controls(self, parent=None) -> ttk.LabelFrame:
        frame = self._frame("Teach By Hand", parent)
        frame.columnconfigure(3, weight=1)
        ttk.Button(frame, text="START TEACHING", command=self.start_teaching).grid(row=0, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="STOP TEACHING", command=self.stop_teaching).grid(row=1, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="SAVE TRAJECTORY", command=self.save_trajectory).grid(row=2, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="CAPTURE + SAVE MANUAL POSE", command=self.read_manual_pose).grid(row=3, column=0, columnspan=2, sticky="ew", pady=3)
        self._label_value(frame, 4, "Recording", self.recording)
        self._label_value(frame, 5, "Time", self.elapsed)
        self._label_value(frame, 6, "Points", self.count)
        ttk.Label(frame, text="Motor encoder (degree | RAW)", font=("TkDefaultFont", 9, "bold")).grid(row=7, column=0, columnspan=2, sticky="w")
        ttk.Label(frame, text="Calibrated joint", font=("TkDefaultFont", 9, "bold")).grid(row=7, column=2, columnspan=2, sticky="w")
        for row, (motor_id, joint) in enumerate(zip((11, 12, 13, 14), ("q1", "q2", "q3", "q4"), strict=True), start=8):
            ttk.Label(frame, text=f"ID{motor_id}:").grid(row=row, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.manual_angles[motor_id]).grid(row=row, column=1, sticky="w")
            ttk.Label(frame, text=f"{joint}:").grid(row=row, column=2, sticky="w")
            ttk.Label(frame, textvariable=self.manual_joints[joint]).grid(row=row, column=3, sticky="w")
        for row, axis in enumerate(("X", "Y", "Z"), start=12):
            ttk.Label(frame, text=f"TCP {axis}:").grid(row=row, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.manual_xyz[axis]).grid(row=row, column=1, sticky="w")
        ttk.Label(
            frame,
            text=(
                "REST ID14=0, q4=0; WORK ID14=82.881 deg, q4=82.881 deg; "
                f"finger-center TCP~({config.WORK_XYZ_MM[0]:.1f},0,{config.WORK_XYZ_MM[2]:.1f}) mm"
            ),
        ).grid(row=15, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Button(frame, text="GO TO TEACH START", command=self.go_to_teach_start).grid(row=16, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="GO TO TEACH END", command=self.go_to_teach_end).grid(row=17, column=0, columnspan=2, sticky="ew", pady=3)
        return frame

    def _replay(self, parent=None) -> ttk.LabelFrame:
        frame = self._frame("Cartesian Replay", parent)
        ttk.Label(frame, text="Move to one XYZ point", font=("TkDefaultFont", 9, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w"
        )
        for row, axis in enumerate(("X", "Y", "Z"), start=1):
            ttk.Label(frame, text=f"Target {axis} (mm)").grid(row=row, column=0, sticky="w")
            ttk.Entry(frame, textvariable=self.target_xyz[axis], width=12).grid(row=row, column=1, sticky="ew")
        ttk.Button(frame, text="PREVIEW NEAREST IK", command=self.preview_xyz).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=3
        )
        ttk.Button(frame, text="MOVE TO XYZ", command=self.move_to_xyz).grid(
            row=5, column=0, columnspan=2, sticky="ew", pady=3
        )
        preview_label = ttk.Label(frame, textvariable=self.ik_preview, wraplength=390)
        preview_label.grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(2, 8)
        )
        frame.bind(
            "<Configure>",
            lambda event: preview_label.configure(wraplength=max(220, event.width - 24)),
        )
        ttk.Separator(frame).grid(row=7, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Button(frame, text="LOAD XYZ TRAJECTORY", command=self.load_xyz).grid(row=8, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="VALIDATE XYZ ONLY", command=self.validate).grid(row=9, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Label(frame, text="Speed scale").grid(row=10, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.speed, width=8).grid(row=10, column=1, sticky="w")
        ttk.Button(frame, text="PLAY XYZ TRAJECTORY", command=self.play).grid(row=11, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="STOP", command=self.stop).grid(row=12, column=0, columnspan=2, sticky="ew", pady=3)
        self._label_value(frame, 13, "Playback", self.playback)
        self._label_value(frame, 14, "Loaded points", self.count)
        return frame

    def _workspace_preview(self, parent=None) -> ttk.LabelFrame:
        frame = self._frame("Physical workspace: ID11 center = (0, 0)", parent)
        points = experiment_points()
        ttk.Label(frame, text="Measured point").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            frame,
            textvariable=self.selected_experiment_point,
            values=tuple(points),
            state="readonly",
            width=8,
        ).grid(row=0, column=1, sticky="w")
        ttk.Button(frame, text="LOAD POINT INTO XYZ", command=self.load_experiment_point).grid(
            row=0, column=2, padx=5, sticky="w"
        )
        ttk.Label(
            frame,
            text="+X forward, +Y right, +Z up; Z=0 is protected ground",
        ).grid(row=0, column=3, sticky="w")
        action_bar = ttk.Frame(frame)
        action_bar.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(5, 2))
        ttk.Button(action_bar, text="ANALYZE ALL POINTS", command=self.analyze_experiment_points).pack(side="left", padx=(0, 4))
        ttk.Button(action_bar, text="RUN SELECTED POINT", command=self.run_selected_point).pack(side="left", padx=4)
        ttk.Button(action_bar, text="RUN ALL REACHABLE", command=self.run_all_reachable).pack(side="left", padx=4)
        ttk.Button(action_bar, text="REPEATABILITY SETUP", command=self._open_repeatability_window).pack(side="left", padx=4)
        ttk.Button(action_bar, text="STOP POINT TEST", command=self.stop_point_experiment).pack(side="left", padx=4)
        ttk.Label(action_bar, text="Experiment:").pack(side="left", padx=(12, 2))
        ttk.Label(action_bar, textvariable=self.experiment_status).pack(side="left")

        columns = ("point", "status", "physical", "internal", "joints", "motors", "fk_error", "message")
        self.experiment_table = ttk.Treeview(frame, columns=columns, show="headings", height=7)
        headings = {
            "point": "Point", "status": "Status", "physical": "Physical XYZ mm",
            "internal": "Internal XYZ mm", "joints": "Touch q deg",
            "motors": "Touch motor deg", "fk_error": "FK error mm", "message": "Details",
        }
        widths = {"point": 55, "status": 155, "physical": 150, "internal": 150,
                  "joints": 185, "motors": 205, "fk_error": 85, "message": 310}
        for column in columns:
            self.experiment_table.heading(column, text=headings[column])
            self.experiment_table.column(column, width=widths[column], minwidth=50, stretch=column == "message")
        table_scroll = ttk.Scrollbar(frame, orient="horizontal", command=self.experiment_table.xview)
        self.experiment_table.configure(xscrollcommand=table_scroll.set)
        self.experiment_table.grid(row=2, column=0, columnspan=4, sticky="nsew", pady=(3, 0))
        table_scroll.grid(row=3, column=0, columnspan=4, sticky="ew")
        self.experiment_table.bind("<<TreeviewSelect>>", self._experiment_row_selected)
        self.workspace_canvas = tk.Canvas(
            frame,
            width=840,
            height=220,
            background="white",
            highlightthickness=0,
        )
        self.workspace_canvas.grid(row=4, column=0, columnspan=4, sticky="nsew", pady=(6, 0))
        self.workspace_canvas.bind("<Configure>", self._workspace_resized)
        frame.rowconfigure(2, weight=1)
        frame.rowconfigure(4, weight=2)
        frame.columnconfigure(3, weight=1)
        self._draw_workspace()
        return frame

    def _open_repeatability_window(self) -> None:
        if self.repeatability_window is not None and self.repeatability_window.winfo_exists():
            self.repeatability_window.deiconify()
            self.repeatability_window.lift()
            return

        window = tk.Toplevel(self.master)
        self.repeatability_window = window
        window.title("Repeatability Experiment Setup and Manual Measurement")
        window.geometry("980x520")
        window.minsize(760, 420)
        window.resizable(True, True)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(2, weight=1)

        selection = ttk.LabelFrame(window, text="Point sequence and repetitions", padding=8)
        selection.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 4))
        for index, (name, selected) in enumerate(self.experiment_point_selected.items()):
            ttk.Checkbutton(selection, text=name, variable=selected).grid(
                row=index // 6, column=index % 6, sticky="w", padx=7, pady=2
            )
        ttk.Button(selection, text="SELECT ALL", command=self._select_all_experiment_points).grid(
            row=2, column=0, padx=4, pady=(6, 0), sticky="ew"
        )
        ttk.Button(selection, text="CLEAR", command=self._clear_experiment_points).grid(
            row=2, column=1, padx=4, pady=(6, 0), sticky="ew"
        )
        ttk.Label(selection, text="Repetitions").grid(row=2, column=2, sticky="e", padx=(12, 2))
        ttk.Entry(selection, textvariable=self.repeatability_repetitions, width=6).grid(row=2, column=3, sticky="w")
        ttk.Label(selection, text="Touch samples").grid(row=2, column=4, sticky="e", padx=(12, 2))
        ttk.Entry(selection, textvariable=self.repeatability_sample_count, width=6).grid(row=2, column=5, sticky="w")
        ttk.Label(selection, text="Interval (ms)").grid(row=2, column=6, sticky="e", padx=(12, 2))
        ttk.Entry(selection, textvariable=self.repeatability_sample_interval_ms, width=6).grid(row=2, column=7, sticky="w")
        ttk.Checkbutton(
            selection,
            text="Pause at every touch for manual measurement",
            variable=self.pause_for_manual_measurement,
        ).grid(row=3, column=0, columnspan=8, sticky="w", padx=4, pady=(5, 0))
        ttk.Button(selection, text="RUN CHECKED SEQUENCE", command=self.run_repeatability_sequence).grid(
            row=4, column=0, columnspan=4, sticky="ew", padx=4, pady=(8, 0)
        )
        ttk.Button(selection, text="STOP", command=self.stop_point_experiment).grid(
            row=4, column=4, columnspan=4, sticky="ew", padx=4, pady=(8, 0)
        )

        manual = ttk.LabelFrame(window, text="Manual touch measurement (physical frame, millimetres)", padding=8)
        manual.grid(row=1, column=0, sticky="ew", padx=10, pady=4)
        ttk.Label(manual, textvariable=self.manual_measurement_status, wraplength=920).grid(
            row=0, column=0, columnspan=10, sticky="w"
        )
        for column, axis in enumerate(("X", "Y", "Z")):
            ttk.Label(manual, text=f"Measured {axis}").grid(row=1, column=column * 2, sticky="e", padx=(4, 2))
            ttk.Entry(manual, textvariable=self.manual_measurement_values[axis], width=9).grid(
                row=1, column=column * 2 + 1, sticky="w"
            )
        ttk.Label(manual, text="Distance error").grid(row=1, column=6, sticky="e", padx=(8, 2))
        ttk.Entry(manual, textvariable=self.manual_distance_error, width=9).grid(row=1, column=7, sticky="w")
        ttk.Label(manual, text="Note").grid(row=2, column=0, sticky="e", padx=(4, 2))
        ttk.Entry(manual, textvariable=self.manual_measurement_note).grid(
            row=2, column=1, columnspan=5, sticky="ew"
        )
        ttk.Button(
            manual,
            text="SAVE MEASUREMENT + CONTINUE",
            command=self.save_manual_touch_and_continue,
        ).grid(row=2, column=6, sticky="ew", padx=3)
        ttk.Button(
            manual,
            text="SKIP + CONTINUE",
            command=self.skip_manual_touch_and_continue,
        ).grid(row=2, column=7, sticky="ew", padx=3)
        ttk.Label(
            manual,
            text="Enter all X/Y/Z, or only distance error. Example: 24.8 cm = 248 mm. Do not push the arm while torque is ON.",
        ).grid(row=3, column=0, columnspan=10, sticky="w", pady=(4, 0))
        manual.columnconfigure(5, weight=1)

        instructions = tk.Text(window, height=8, wrap="word")
        instructions.grid(row=2, column=0, sticky="nsew", padx=10, pady=(4, 10))
        instructions.insert(
            "1.0",
            "Procedure\n"
            "1. Check the required points in the order P01 to P11.\n"
            "2. Set repetitions and touch samples.\n"
            "3. Connect, enable torque, and start from WORK in the main window.\n"
            "4. Press RUN CHECKED SEQUENCE and confirm the preflight.\n"
            "5. At each touch, read the automatic FK result above. Measure the real TCP. "
            "Enter physical X/Y/Z in millimetres or a measured scalar distance error, then continue.\n"
            "6. The robot retracts and returns to WORK after every point. The workbook is autosaved after each touch."
        )
        instructions.configure(state="disabled")

        def close_window() -> None:
            if self._pending_touch_record is not None:
                messagebox.showwarning(
                    "Manual measurement pending",
                    "Save or skip the current manual measurement, or press STOP, before closing this window.",
                    parent=window,
                )
                return
            window.destroy()
            self.repeatability_window = None

        window.protocol("WM_DELETE_WINDOW", close_window)

    def _draw_workspace(self, highlight=None) -> None:
        if highlight is not None:
            self._workspace_highlight = highlight
        highlight = self._workspace_highlight
        canvas = self.workspace_canvas
        canvas.delete("all")
        width = float(max(canvas.winfo_width(), 480))
        height = float(max(canvas.winfo_height(), 170))
        pad = 35.0
        x_limits = config.MEASURED_XYZ_LIMITS["x"]
        y_limits = config.MEASURED_XYZ_LIMITS["y"]

        def sx(value: float) -> float:
            return pad + (value - x_limits.minimum) * (width - 2 * pad) / (x_limits.maximum - x_limits.minimum)

        def sy(value: float) -> float:
            return height - pad - (value - y_limits.minimum) * (height - 2 * pad) / (y_limits.maximum - y_limits.minimum)

        canvas.create_rectangle(
            sx(x_limits.minimum), sy(y_limits.maximum),
            sx(x_limits.maximum), sy(y_limits.minimum), outline="#1f4e78",
        )
        canvas.create_line(
            sx(x_limits.minimum), sy(0.0), sx(x_limits.maximum), sy(0.0),
            fill="#555", arrow="last",
        )
        canvas.create_line(
            sx(0.0), sy(y_limits.minimum), sx(0.0), sy(y_limits.maximum),
            fill="#555", arrow="last",
        )
        canvas.create_oval(sx(0.0) - 6, sy(0.0) - 6, sx(0.0) + 6, sy(0.0) + 6, fill="#111")
        canvas.create_text(sx(0.0) + 7, sy(0.0) + 13, text="ID11 (0,0)", anchor="w")
        canvas.create_text(sx(x_limits.minimum), sy(0.0) + 15, text=f"{x_limits.minimum:.0f}", anchor="w")
        canvas.create_text(
            sx(x_limits.maximum), sy(0.0) + 15,
            text=f"+{x_limits.maximum:.0f} mm  +X forward", anchor="e",
        )
        canvas.create_text(sx(0.0) + 6, sy(y_limits.maximum), text="+Y right", anchor="nw")
        for name, point in experiment_points().items():
            color = "#d62728" if highlight == name else "#1976d2"
            radius = 6 if highlight == name else 4
            canvas.create_oval(
                sx(point.x) - radius, sy(point.y) - radius,
                sx(point.x) + radius, sy(point.y) + radius,
                fill=color, outline=color,
            )
            canvas.create_text(sx(point.x) + 6, sy(point.y) - 7, text=name, anchor="sw", fill=color)

    def _workspace_resized(self, _event) -> None:
        self._draw_workspace()

    def load_experiment_point(self) -> None:
        name = self.selected_experiment_point.get()
        point = experiment_points()[name]
        for axis, value in zip(("X", "Y", "Z"), (point.x, point.y, point.z), strict=True):
            self.target_xyz[axis].set(f"{value:.3f}")
        self._draw_workspace(name)
        self._log(
            f"Loaded {name}: physical XYZ=({point.x:.1f}, {point.y:.1f}, {point.z:.1f}) mm; "
            "press PREVIEW before motion"
        )

    @staticmethod
    def _tuple_text(values, digits: int = 2) -> str:
        return "(" + ", ".join(f"{value:.{digits}f}" for value in values) + ")"

    @staticmethod
    def _xyz_tuple(value) -> tuple[float, float, float]:
        return value.x, value.y, value.z

    def _experiment_row_selected(self, _event=None) -> None:
        selection = self.experiment_table.selection()
        if not selection:
            return
        name = self.experiment_table.item(selection[0], "values")[0]
        if name in experiment_points():
            self.selected_experiment_point.set(name)
            self.load_experiment_point()

    def _show_experiment_plan(self, plan: ExperimentPlan) -> None:
        self.experiment_table.delete(*self.experiment_table.get_children())
        for point in plan.points:
            touch = point.touch
            joints = self._tuple_text(touch.joint_angles.as_tuple()) if touch.joint_angles else "---"
            motors = self._tuple_text(touch.motor_angles.as_tuple()) if touch.motor_angles else "---"
            fk_error = f"{touch.fk_error_mm:.4f}" if touch.fk_error_mm is not None else "---"
            item = self.experiment_table.insert(
                "", "end", iid=point.name,
                values=(
                    point.name,
                    point.status,
                    self._tuple_text(self._xyz_tuple(point.physical_target), 1),
                    self._tuple_text(self._xyz_tuple(point.internal_target), 1),
                    joints,
                    motors,
                    fk_error,
                    point.message,
                ),
            )
            self.experiment_table.item(item, tags=("ready" if point.motion_ready else "blocked",))
        self.experiment_table.tag_configure("ready", foreground="#176b2c")
        self.experiment_table.tag_configure("blocked", foreground="#a32222")

    def analyze_experiment_points(self) -> ExperimentPlan | None:
        """Offline preflight; a connected robot only supplies nearest-solution reference."""
        try:
            reference = None
            source = "offline default"
            if self.controller.status.connected:
                state = self.controller.read_robot_state()
                reference = state.joints
                source = "current measured joints"
                self._display_robot_state(state)
                self._record_telemetry(state, event="analysis_reference")
            plan = plan_named_experiment_points(starting_reference=reference)
            self._experiment_plan = plan
            self._show_experiment_plan(plan)
            blocked = [f"{point.name}: {point.status}" for point in plan.points if not point.motion_ready]
            self.experiment_status.set(f"READY {plan.ready_count}/{len(plan.points)}")
            summary = (
                f"Point analysis from {source}: {plan.ready_count} ready, {plan.blocked_count} blocked; "
                f"approach/retract={plan.approach_clearance_mm:.1f}/{plan.retract_clearance_mm:.1f} mm"
            )
            if blocked:
                summary += "; " + "; ".join(blocked)
            self._log(summary, event="point_analysis_complete", context={"plan": plan})
            return plan
        except (ControllerError, KinematicsError, ValueError, KeyError) as exc:
            self._error(exc, event="point_analysis_failed")
            return None

    def _prepare_point_run(self, names: list[str], title: str) -> None:
        try:
            if self._experiment_running:
                raise ControllerError("A point experiment is already running")
            if not self.controller.status.connected:
                raise ControllerError("Connect to OpenCR before running a point experiment")
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON before running a point experiment")
            state = self.controller.read_robot_state()
            plan = plan_point_experiment(names=names, starting_reference=state.joints)
            self._experiment_plan = plan
            self._show_experiment_plan(plan)
            ready = [point.name for point in plan.points if point.motion_ready]
            blocked = [f"{point.name} ({point.status})" for point in plan.points if not point.motion_ready]
            if not ready:
                raise ControllerError("No selected experiment point passed all safety checks")
            details = (
                f"Will run: {', '.join(ready)}\n"
                f"Each point: approach at Z+{config.APPROACH_CLEARANCE_MM:.0f} mm, "
                f"touch Z=0, dwell {config.TOUCH_DWELL_SECONDS:.2f} s, retract.\n"
                f"TCP warning above {config.TCP_WARNING_ERROR_MM:.1f} mm; "
                f"hard stop above {config.MAXIMUM_TCP_ERROR_MM:.1f} mm.\n"
                "After every completed point, the robot returns to WORK."
            )
            if blocked:
                details += f"\n\nWill NOT run: {', '.join(blocked)}."
            details += "\n\nClear the complete workspace, keep one hand near STOP, and confirm motion."
            if not messagebox.askyesno(title, details):
                self._log("Point experiment cancelled", event="point_run_cancelled", context={"requested": names})
                return
            self._experiment_stop.clear()
            self._experiment_running = True
            self.experiment_status.set("RUNNING")
            self.logger.log_history(
                "point_run_started", category="motion",
                context={"requested": names, "ready": ready, "blocked": blocked, "plan": plan},
            )
            threading.Thread(target=self._point_run_worker, args=(ready,), daemon=True).start()
        except (ControllerError, KinematicsError, ValueError, KeyError) as exc:
            self._error(exc, event="point_run_preflight_failed", context={"requested": names})

    def run_selected_point(self) -> None:
        self._prepare_point_run([self.selected_experiment_point.get()], "Confirm selected point test")

    def run_all_reachable(self) -> None:
        self._prepare_point_run(list(experiment_points()), "Confirm all reachable point tests")

    def _select_all_experiment_points(self) -> None:
        for selected in self.experiment_point_selected.values():
            selected.set(True)

    def _clear_experiment_points(self) -> None:
        for selected in self.experiment_point_selected.values():
            selected.set(False)

    def _checked_experiment_points(self) -> list[str]:
        return [
            name for name, selected in self.experiment_point_selected.items() if selected.get()
        ]

    def _repeatability_settings(self) -> tuple[list[str], int, int, int, bool]:
        names = self._checked_experiment_points()
        if not names:
            raise ValueError("Check at least one experiment point")
        repetitions = int(self.repeatability_repetitions.get())
        sample_count = int(self.repeatability_sample_count.get())
        sample_interval_ms = int(self.repeatability_sample_interval_ms.get())
        if not 1 <= repetitions <= config.MAXIMUM_REPETITIONS:
            raise ValueError(
                f"Repetitions must be between 1 and {config.MAXIMUM_REPETITIONS}"
            )
        if not 1 <= sample_count <= config.MAXIMUM_TOUCH_SAMPLES:
            raise ValueError(
                f"Touch samples must be between 1 and {config.MAXIMUM_TOUCH_SAMPLES}"
            )
        if not 20 <= sample_interval_ms <= 5000:
            raise ValueError("Touch sample interval must be between 20 and 5000 ms")
        return (
            names,
            repetitions,
            sample_count,
            sample_interval_ms,
            bool(self.pause_for_manual_measurement.get()),
        )

    def run_repeatability_sequence(self) -> None:
        """Preflight and start the checked multi-point repeatability experiment."""
        names: list[str] = []
        try:
            if self._experiment_running:
                raise ControllerError("A point experiment is already running")
            if not self.controller.status.connected:
                raise ControllerError("Connect to OpenCR before running repeatability")
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON before running repeatability")
            names, repetitions, sample_count, sample_interval_ms, pause_manual = (
                self._repeatability_settings()
            )
            state = self.controller.read_robot_state()
            plan = plan_point_experiment(names=names, starting_reference=state.joints)
            self._experiment_plan = plan
            self._show_experiment_plan(plan)
            blocked = [
                f"{point.name} ({point.status})" for point in plan.points if not point.motion_ready
            ]
            if blocked:
                raise ControllerError(
                    "Repeatability sequence blocked because every checked point must pass: "
                    + ", ".join(blocked)
                )

            total_touches = len(names) * repetitions
            manual_text = (
                "The robot will pause and hold at every touch until you save or skip the manual measurement."
                if pause_manual
                else "No manual pause; encoder telemetry will be sampled automatically."
            )
            details = (
                f"Checked points: {', '.join(names)}\n"
                f"Repetitions: {repetitions}; total touches: {total_touches}\n"
                f"Each touch: {sample_count} telemetry samples at {sample_interval_ms} ms.\n"
                "After each point the robot retracts and returns to WORK.\n"
                f"{manual_text}\n\n"
                "Clear the full workspace, keep one hand near STOP, and confirm motion."
            )
            if not messagebox.askyesno("Confirm repeatability experiment", details):
                self._log(
                    "Repeatability experiment cancelled",
                    event="repeatability_run_cancelled",
                    context={"points": names, "repetitions": repetitions},
                )
                return

            run = RepeatabilityRunConfig(
                session_id=new_session_id(),
                selected_points=tuple(names),
                repetitions=repetitions,
                sample_count=sample_count,
                sample_interval_ms=sample_interval_ms,
                pause_for_manual_measurement=pause_manual,
            )
            workbook = RepeatabilityWorkbook(self.output_dir, run)
            workbook.add_event("repeatability_run_started", details.replace("\n", "; "))
            workbook.save()
            self._repeatability_workbook = workbook
            self._experiment_stop.clear()
            self._manual_measurement_continue.clear()
            self._experiment_running = True
            self.experiment_status.set(f"RUNNING 0/{total_touches}")
            self._log(
                f"Repeatability workbook: {workbook.path}",
                event="repeatability_run_started",
                category="motion",
                context={"run": run, "workbook": workbook.path},
            )
            threading.Thread(
                target=self._repeatability_run_worker,
                args=(names, repetitions, sample_count, sample_interval_ms, pause_manual),
                daemon=True,
            ).start()
        except (ControllerError, KinematicsError, OSError, ValueError, KeyError) as exc:
            self._error(
                exc,
                event="repeatability_run_preflight_failed",
                context={"requested": names},
            )

    def _record_full_telemetry(
        self,
        snapshot,
        *,
        cycle: int,
        point_name: str,
        touch_index: int,
        sample_index: int,
        target: XYZ,
    ) -> None:
        state = snapshot.state
        error_mm = math.dist(self._xyz_tuple(state.physical_xyz), self._xyz_tuple(target))
        motor_values = {}
        for motor in snapshot.motors:
            prefix = f"id{motor.motor_id}"
            motor_values.update(
                {
                    f"{prefix}_velocity_raw": motor.velocity_raw,
                    f"{prefix}_velocity_rpm": motor.velocity_rpm,
                    f"{prefix}_current_raw": motor.current_raw,
                    f"{prefix}_current_ma": motor.current_ma,
                    f"{prefix}_pwm_raw": motor.pwm_raw,
                    f"{prefix}_pwm_percent": motor.pwm_percent,
                    f"{prefix}_voltage_raw": motor.voltage_raw,
                    f"{prefix}_voltage_v": motor.voltage_v,
                    f"{prefix}_temperature_c": motor.temperature_c,
                    f"{prefix}_hardware_error": motor.hardware_error,
                    f"{prefix}_moving": motor.moving,
                    f"{prefix}_moving_status": motor.moving_status,
                }
            )
        self._record_telemetry(
            state,
            event="repeatability_touch_sample",
            target=target,
            position_error_mm=error_mm,
            extra={
                "cycle": cycle,
                "point": point_name,
                "touch_index": touch_index,
                "touch_sample_index": sample_index,
                **motor_values,
            },
        )

    def _move_repeatability_phase(self, point_name: str, phase, cycle: int) -> None:
        if self._experiment_stop.is_set():
            return
        self._telemetry_target = phase.physical_target
        self.after(
            0,
            self.experiment_status.set,
            f"CYCLE {cycle} {point_name} {phase.phase}",
        )
        self.logger.log_history(
            "repeatability_phase_commanded",
            category="motion",
            context={"cycle": cycle, "point": point_name, "phase": phase},
        )
        self._experiment_motion_active = True
        try:
            self.controller.move_motor_angles(phase.motor_angles)
        finally:
            self._experiment_motion_active = False
        wait_seconds = (
            config.TOUCH_DWELL_SECONDS
            if phase.phase == "TOUCH"
            else config.POINT_SETTLE_SECONDS
        )
        if self._experiment_stop.wait(wait_seconds):
            return
        if phase.phase != "TOUCH":
            state = self.controller.read_robot_state()
            error_mm = math.dist(
                self._xyz_tuple(state.physical_xyz), self._xyz_tuple(phase.physical_target)
            )
            self._record_telemetry(
                state,
                event="repeatability_phase_measured",
                target=phase.physical_target,
                position_error_mm=error_mm,
                extra={"cycle": cycle, "point": point_name, "phase": phase.phase},
            )
            self.after(0, self._display_robot_state, state, False)

    def _repeatability_run_worker(
        self,
        names: list[str],
        repetitions: int,
        sample_count: int,
        sample_interval_ms: int,
        pause_manual: bool,
    ) -> None:
        workbook = self._repeatability_workbook
        if workbook is None:
            self.after(
                0,
                self._finish_repeatability_run,
                True,
                ControllerError("Repeatability workbook was not initialized"),
            )
            return
        total_touches = len(names) * repetitions
        touch_index = 0
        try:
            for cycle in range(1, repetitions + 1):
                for name in names:
                    if self._experiment_stop.is_set():
                        break
                    before = self.controller.read_robot_state()
                    plan = plan_point_experiment(names=[name], starting_reference=before.joints)
                    point = plan.points[0]
                    if not point.motion_ready:
                        raise ControllerError(
                            f"Cycle {cycle} {name} changed to {point.status}: {point.message}"
                        )

                    self._move_repeatability_phase(name, point.approach, cycle)
                    if self._experiment_stop.is_set():
                        break
                    self._move_repeatability_phase(name, point.touch, cycle)
                    if self._experiment_stop.is_set():
                        break

                    touch_index += 1
                    samples = []
                    for sample_index in range(1, sample_count + 1):
                        if self._experiment_stop.is_set():
                            break
                        snapshot = self.controller.read_telemetry()
                        samples.append(snapshot)
                        self._record_full_telemetry(
                            snapshot,
                            cycle=cycle,
                            point_name=name,
                            touch_index=touch_index,
                            sample_index=sample_index,
                            target=point.touch_target,
                        )
                        self.after(0, self._display_robot_state, snapshot.state, False)
                        if sample_index < sample_count and self._experiment_stop.wait(
                            sample_interval_ms / 1000.0
                        ):
                            break
                    if self._experiment_stop.is_set():
                        break
                    if not samples:
                        raise ControllerError(f"No telemetry captured at cycle {cycle} {name}")

                    record = TouchRecord(
                        cycle=cycle,
                        point_name=name,
                        touch_index=touch_index,
                        commanded_physical=point.touch_target,
                        commanded_internal=point.touch.internal_target,
                        planned_joints=point.touch.joint_angles,
                        planned_motors=point.touch.motor_angles,
                        samples=samples,
                    )
                    workbook.add_touch(record)
                    fk_error = record.fk_error()
                    self.logger.log_history(
                        "repeatability_touch_sampled",
                        category="measurement",
                        context={
                            "cycle": cycle,
                            "point": name,
                            "touch_index": touch_index,
                            "mean_fk": record.mean_physical_fk(),
                            "fk_error_xyz_norm": fk_error,
                            "sample_count": len(samples),
                            "workbook": workbook.path,
                        },
                    )
                    self.after(
                        0,
                        self._show_manual_touch_pause,
                        record,
                        total_touches,
                        pause_manual,
                    )

                    severity = tcp_error_severity(fk_error[3])
                    if severity == MEASUREMENT_STOP:
                        raise ControllerError(
                            f"Cycle {cycle} {name} FK-from-encoder error {fk_error[3]:.3f} mm "
                            f"exceeds hard limit {config.MAXIMUM_TCP_ERROR_MM:.3f} mm"
                        )
                    if severity == MEASUREMENT_WARNING:
                        self.after(
                            0,
                            self._log,
                            f"WARNING: cycle {cycle} {name} FK error {fk_error[3]:.3f} mm",
                        )

                    if pause_manual:
                        self._manual_measurement_continue.clear()
                        while not self._experiment_stop.is_set():
                            if self._manual_measurement_continue.wait(0.1):
                                break
                        if self._experiment_stop.is_set():
                            break
                        manual = None if self._manual_measurement_skipped else self._manual_measurement_payload
                        workbook.update_manual(record, manual)
                    else:
                        workbook.update_manual(record, None)

                    self._move_repeatability_phase(name, point.retract, cycle)
                    if self._experiment_stop.is_set():
                        break
                    self.after(0, self.experiment_status.set, f"CYCLE {cycle} {name} RETURN WORK")
                    self._experiment_motion_active = True
                    try:
                        self.controller.move_work()
                    finally:
                        self._experiment_motion_active = False
                    if self._experiment_stop.wait(config.POINT_SETTLE_SECONDS):
                        break
                    work_state = self.controller.read_robot_state()
                    work_error = math.dist(
                        self._xyz_tuple(work_state.physical_xyz), config.WORK_XYZ_MM
                    )
                    self._record_telemetry(
                        work_state,
                        event="repeatability_return_work",
                        target=XYZ(*config.WORK_XYZ_MM),
                        position_error_mm=work_error,
                        extra={"cycle": cycle, "point": name},
                    )
                    self.after(0, self._display_robot_state, work_state, False)
                    self.after(
                        0,
                        self._log,
                        f"Cycle {cycle}/{repetitions} {name} complete; "
                        f"touch {touch_index}/{total_touches}; returned to WORK",
                    )
                if self._experiment_stop.is_set():
                    break

            stopped = self._experiment_stop.is_set()
            workbook.add_event("repeatability_run_stopped" if stopped else "repeatability_run_completed")
            workbook.save()
            self.after(0, self._finish_repeatability_run, stopped, None)
        except Exception as exc:
            try:
                workbook.add_event("repeatability_run_failed", str(exc))
                workbook.save()
            except Exception as save_exc:
                self.logger.log_error(
                    "repeatability_recovery_save_failed",
                    save_exc,
                    context={"original_error": exc},
                )
            self.after(0, self._finish_repeatability_run, True, exc)
        finally:
            self._telemetry_target = None
            self._manual_measurement_continue.set()

    def _show_manual_touch_pause(
        self, record: TouchRecord, total_touches: int, pause_manual: bool
    ) -> None:
        if pause_manual and (
            self.repeatability_window is None
            or not self.repeatability_window.winfo_exists()
        ):
            self._open_repeatability_window()
        self._pending_touch_record = record if pause_manual else None
        self._manual_measurement_payload = None
        self._manual_measurement_skipped = False
        for value in self.manual_measurement_values.values():
            value.set("")
        self.manual_distance_error.set("")
        self.manual_measurement_note.set("")
        mean_fk = record.mean_physical_fk()
        error = record.fk_error()
        action = "Measure now, then save or skip" if pause_manual else "Automatic measurement saved"
        self.manual_measurement_status.set(
            f"Touch {record.touch_index}/{total_touches}: cycle {record.cycle} {record.point_name}; "
            f"command=({record.commanded_physical.x:.1f}, {record.commanded_physical.y:.1f}, "
            f"{record.commanded_physical.z:.1f}) mm; FK mean=({mean_fk.x:.2f}, {mean_fk.y:.2f}, "
            f"{mean_fk.z:.2f}) mm; error={error[3]:.2f} mm. {action}."
        )

    def save_manual_touch_and_continue(self) -> None:
        try:
            if self._pending_touch_record is None or not self._experiment_running:
                raise ValueError("No touch is currently waiting for a manual measurement")
            text_xyz = tuple(
                self.manual_measurement_values[axis].get().strip() for axis in ("X", "Y", "Z")
            )
            provided = tuple(bool(value) for value in text_xyz)
            if any(provided) and not all(provided):
                raise ValueError("Enter all three measured X, Y, and Z values, or leave all blank")
            measured_xyz = XYZ(*(float(value) for value in text_xyz)) if all(provided) else None
            error_text = self.manual_distance_error.get().strip()
            reported_error = float(error_text) if error_text else None
            if reported_error is not None and (not math.isfinite(reported_error) or reported_error < 0.0):
                raise ValueError("Measured distance error must be a finite non-negative value")
            if measured_xyz is None and reported_error is None:
                raise ValueError("Enter measured X/Y/Z or a measured distance error; otherwise use SKIP")
            self._manual_measurement_payload = ManualMeasurement(
                measured_xyz=measured_xyz,
                reported_distance_error_mm=reported_error,
                note=self.manual_measurement_note.get().strip(),
            )
            self._manual_measurement_skipped = False
            self._pending_touch_record = None
            self.manual_measurement_status.set("Manual measurement accepted; continuing")
            self._manual_measurement_continue.set()
        except ValueError as exc:
            self._error(exc, event="manual_touch_measurement_invalid")

    def skip_manual_touch_and_continue(self) -> None:
        if self._pending_touch_record is None or not self._experiment_running:
            self._error(
                ValueError("No touch is currently waiting for a manual measurement"),
                event="manual_touch_skip_invalid",
            )
            return
        self._manual_measurement_payload = None
        self._manual_measurement_skipped = True
        self._pending_touch_record = None
        self.manual_measurement_status.set("Manual measurement skipped; continuing")
        self._manual_measurement_continue.set()

    def _finish_repeatability_run(self, stopped: bool, exc: Exception | None) -> None:
        self._experiment_running = False
        self._experiment_motion_active = False
        self._pending_touch_record = None
        workbook_path = self._repeatability_workbook.path if self._repeatability_workbook else None
        if exc is not None:
            self.experiment_status.set("ERROR / STOPPED")
            self._error(
                exc,
                event="repeatability_run_failed",
                context={"workbook": workbook_path},
            )
        elif stopped:
            self.experiment_status.set("STOPPED")
            self._log(
                f"Repeatability stopped; saved {workbook_path}",
                event="repeatability_run_stopped",
                category="motion",
            )
        else:
            self.experiment_status.set("COMPLETE")
            self._log(
                f"Repeatability complete; saved {workbook_path}",
                event="repeatability_run_completed",
                category="motion",
            )
        self.manual_measurement_status.set("No repeatability touch is waiting for measurement")

    def _point_run_worker(self, names: list[str]) -> None:
        try:
            for name in names:
                if self._experiment_stop.is_set():
                    break
                # Re-plan every point from fresh measured joints. This preserves
                # nearest-solution IK and detects state drift between points.
                before = self.controller.read_robot_state()
                plan = plan_point_experiment(names=[name], starting_reference=before.joints)
                point = plan.points[0]
                if not point.motion_ready:
                    raise ControllerError(f"{name} changed to {point.status}: {point.message}")
                for phase in (point.approach, point.touch, point.retract):
                    if self._experiment_stop.is_set():
                        break
                    self._telemetry_target = phase.physical_target
                    self.logger.log_history(
                        "point_phase_commanded", category="motion",
                        context={"point": name, "phase": phase.phase, "target": phase},
                    )
                    self.after(0, self.experiment_status.set, f"{name} {phase.phase}")
                    self._experiment_motion_active = True
                    try:
                        self.controller.move_motor_angles(phase.motor_angles)
                    finally:
                        self._experiment_motion_active = False
                    wait_seconds = (
                        config.TOUCH_DWELL_SECONDS if phase.phase == "TOUCH" else config.POINT_SETTLE_SECONDS
                    )
                    if self._experiment_stop.wait(wait_seconds):
                        break
                    actual = self.controller.read_robot_state()
                    target = phase.physical_target
                    error_mm = math.dist(self._xyz_tuple(actual.physical_xyz), self._xyz_tuple(target))
                    self._record_telemetry(
                        actual,
                        event="point_phase_measured",
                        target=target,
                        position_error_mm=error_mm,
                        extra={"point": name, "phase": phase.phase},
                    )
                    self.after(0, self._display_robot_state, actual, False)
                    self.logger.log_history(
                        "point_phase_completed", category="motion",
                        context={"point": name, "phase": phase.phase, "target": target,
                                 "actual": actual, "position_error_mm": error_mm},
                    )
                    severity = tcp_error_severity(error_mm)
                    if severity == MEASUREMENT_WARNING:
                        warning = (
                            f"WARNING: {name} {phase.phase} TCP error {error_mm:.3f} mm exceeds "
                            f"warning {config.TCP_WARNING_ERROR_MM:.3f} mm; continuing"
                        )
                        self.logger.log_history(
                            "point_phase_tolerance_warning", category="warning",
                            context={"point": name, "phase": phase.phase, "target": target,
                                     "actual": actual, "position_error_mm": error_mm},
                        )
                        self.after(0, self._log, warning)
                    elif severity == MEASUREMENT_STOP:
                        raise ControllerError(
                            f"{name} {phase.phase} TCP error {error_mm:.3f} mm exceeds "
                            f"{config.MAXIMUM_TCP_ERROR_MM:.3f} mm; experiment stopped"
                        )
                if self._experiment_stop.is_set():
                    break
                self.after(0, self.experiment_status.set, f"{name} RETURN WORK")
                self.logger.log_history(
                    "point_return_work_commanded", category="motion", context={"point": name}
                )
                self._experiment_motion_active = True
                try:
                    self.controller.move_work()
                finally:
                    self._experiment_motion_active = False
                if self._experiment_stop.wait(config.POINT_SETTLE_SECONDS):
                    break
                work_state = self.controller.read_robot_state()
                work_error_mm = math.dist(
                    self._xyz_tuple(work_state.physical_xyz), config.WORK_XYZ_MM
                )
                self._record_telemetry(
                    work_state,
                    event="point_return_work_measured",
                    target=XYZ(*config.WORK_XYZ_MM),
                    position_error_mm=work_error_mm,
                    extra={"point": name, "phase": "RETURN_WORK"},
                )
                self.after(0, self._display_robot_state, work_state, False)
                self.logger.log_history(
                    "point_return_work_completed", category="motion",
                    context={"point": name, "actual": work_state, "position_error_mm": work_error_mm},
                )
                work_severity = tcp_error_severity(work_error_mm)
                if work_severity == MEASUREMENT_STOP:
                    raise ControllerError(
                        f"Return to WORK after {name} has TCP error {work_error_mm:.3f} mm, "
                        f"above hard limit {config.MAXIMUM_TCP_ERROR_MM:.3f} mm"
                    )
                if work_severity == MEASUREMENT_WARNING:
                    self.logger.log_history(
                        "point_return_work_tolerance_warning", category="warning",
                        context={"point": name, "actual": work_state,
                                 "position_error_mm": work_error_mm},
                    )
                self.after(
                    0,
                    self._log,
                    f"{name} complete; returned to WORK (TCP error {work_error_mm:.3f} mm)",
                )
            stopped = self._experiment_stop.is_set()
            self.after(0, self._finish_point_run, stopped, None)
        except Exception as exc:
            self.after(0, self._finish_point_run, True, exc)
        finally:
            self._telemetry_target = None

    def _finish_point_run(self, stopped: bool, exc: Exception | None) -> None:
        self._experiment_running = False
        self._experiment_motion_active = False
        if exc is not None:
            self.experiment_status.set("ERROR / STOPPED")
            self._error(exc, event="point_run_failed")
        elif stopped:
            self.experiment_status.set("STOPPED")
            self._log("Point experiment stopped", event="point_run_stopped", category="motion")
        else:
            self.experiment_status.set("COMPLETE")
            self._log("All validated point phases completed", event="point_run_completed", category="motion")

    def stop_point_experiment(self) -> None:
        was_waiting_for_manual = self._pending_touch_record is not None
        was_moving = self._experiment_motion_active
        self._experiment_stop.set()
        self._manual_measurement_continue.set()
        if not self._experiment_running:
            self.experiment_status.set("IDLE")
            self._log("No point experiment is running", event="point_stop_noop")
            return
        try:
            if was_waiting_for_manual or not was_moving:
                # No motor command is active during a manual pause, telemetry
                # sample, or dwell. Avoid
                # sending STOP/IDLE into the serial stream where it could be
                # mistaken for the response to the next command.
                self._log(
                    "Point experiment stop accepted between motor movements",
                    event="point_stop_requested",
                    category="motion",
                )
            else:
                self.controller.stop()
                self._log("STOP requested for point experiment", event="point_stop_requested", category="motion")
        except ControllerError as exc:
            self._error(exc, event="point_stop_failed")

    def _nearest_ik_from_current(self):
        try:
            target = tuple(float(self.target_xyz[axis].get().strip()) for axis in ("X", "Y", "Z"))
        except ValueError as exc:
            raise ValueError("Enter numeric X, Y, and Z values in millimetres") from exc
        current_motors = self.controller.read_motor_angles()
        current_joints = motor_to_fk_angles(current_motors)
        physical_result = inverse_kinematics_physical(*target, reference=current_joints)
        result = physical_result.ik
        validate_ik_solution(result)
        deltas = tuple(
            normalize_angle(goal - current)
            for goal, current in zip(result.joint_angles.as_tuple(), current_joints.as_tuple(), strict=True)
        )
        return physical_result, current_motors, current_joints, result, deltas

    def preview_xyz(self) -> None:
        try:
            physical_result, _motors, _joints, result, deltas = self._nearest_ik_from_current()
            target = physical_result.physical_target
            internal = physical_result.internal_target
            text = (
                f"Physical XYZ=({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) mm; "
                f"internal IK XYZ=({internal.x:.1f}, {internal.y:.1f}, {internal.z:.1f}) mm; "
                f"nearest q=({result.joint_angles.theta1:.1f}, {result.joint_angles.theta2:.1f}, "
                f"{result.joint_angles.theta3:.1f}, {result.joint_angles.theta4:.1f}) deg; "
                f"move Δq=({deltas[0]:+.1f}, {deltas[1]:+.1f}, {deltas[2]:+.1f}, {deltas[3]:+.1f}) deg; "
                f"FK error={result.position_error:.3f} mm"
            )
            self.ik_preview.set(text)
            self._log(text)
        except (ControllerError, KinematicsError, ValueError) as exc:
            self._error(exc)

    def move_to_xyz(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON before MOVE TO XYZ")
            physical_result, _motors, _joints, result, deltas = self._nearest_ik_from_current()
            target = physical_result.physical_target
            internal = physical_result.internal_target
            preview = (
                f"Physical experiment XYZ: ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) mm\n"
                f"Transformed internal XYZ: ({internal.x:.1f}, {internal.y:.1f}, {internal.z:.1f}) mm\n"
                f"Joint movement: ({deltas[0]:+.1f}, {deltas[1]:+.1f}, {deltas[2]:+.1f}, {deltas[3]:+.1f}) deg\n"
                f"Predicted error: {result.position_error:.3f} mm\n\n"
                "Clear the workspace and support the arm. Move now?"
            )
            if not messagebox.askyesno("Confirm nearest IK movement", preview):
                self._log("XYZ movement cancelled")
                return
            self.controller.move_motor_angles(result.motor_angles)
            actual = self.controller.read_motor_angles()
            actual_internal = forward_kinematics(actual)
            actual_physical = forward_kinematics_physical(actual)
            self._log(
                f"XYZ move complete; physical FK=({actual_physical.x:.3f}, {actual_physical.y:.3f}, "
                f"{actual_physical.z:.3f}) mm; internal FK=({actual_internal.x:.3f}, "
                f"{actual_internal.y:.3f}, {actual_internal.z:.3f}) mm"
            )
        except (ControllerError, KinematicsError, ValueError) as exc:
            self._error(exc)

    def _status(self, parent=None) -> ttk.LabelFrame:
        frame = self._frame("Status", parent)
        ttk.Label(frame, textvariable=self.status).grid(row=0, column=0, sticky="w")
        self.log = tk.Text(frame, height=10, width=90, wrap="word")
        self.log.grid(row=1, column=0, sticky="nsew")
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        return frame

    def _label_value(self, frame: ttk.LabelFrame, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(frame, text=f"{label}:").grid(row=row, column=0, sticky="w")
        ttk.Label(frame, textvariable=variable).grid(row=row, column=1, sticky="w")

    def _refresh_ports(self) -> None:
        ports = available_ports()
        # COM numbers are assigned by Windows, not by the OpenCR sketch. Keep
        # the lab's known OpenCR port visible/defaulted even while the board is
        # temporarily disconnected; detected alternatives remain selectable.
        displayed_ports = ["COM7", *(port for port in ports if port.upper() != "COM7")]
        self.port_box.configure(values=displayed_ports)
        if not self.port.get():
            self.port.set("COM7")

    def _log(
        self,
        message: str,
        *,
        event: str = "gui_message",
        category: str = "gui",
        context=None,
        persist: bool = True,
    ) -> None:
        self.status.set(message)
        self.log.insert("end", message + "\n")
        self.log.see("end")
        if persist:
            try:
                self.logger.log_history(event, category=category, context={"message": message, "details": context})
            except Exception as log_exc:
                self.log.insert("end", f"LOGGING ERROR: {log_exc}\n")
                self.log.see("end")

    def _error(self, exc: Exception, *, event: str = "gui_error", context=None) -> None:
        self._log(f"ERROR: {exc}", event=event, category="error", context=context, persist=False)
        try:
            self.logger.log_error(event, exc, category="error", context=context)
        except Exception as log_exc:
            self.log.insert("end", f"ERROR LOGGING FAILED: {log_exc}\n")
            self.log.see("end")

    def _record_telemetry(
        self,
        state,
        *,
        event: str = "telemetry_sample",
        target=None,
        position_error_mm: float | None = None,
        extra=None,
    ) -> None:
        self._telemetry_sample_index += 1
        raw = state.raw_positions or (None, None, None, None)
        target = target if target is not None else self._telemetry_target
        values = {
            "elapsed_s": time.monotonic() - self._session_started,
            "sample_index": self._telemetry_sample_index,
            "id11_raw": raw[0], "id12_raw": raw[1], "id13_raw": raw[2], "id14_raw": raw[3],
            "id11_deg": state.motors.id11, "id12_deg": state.motors.id12,
            "id13_deg": state.motors.id13, "id14_deg": state.motors.id14,
            "q1_deg": state.joints.theta1, "q2_deg": state.joints.theta2,
            "q3_deg": state.joints.theta3, "q4_deg": state.joints.theta4,
            "physical_x_mm": state.physical_xyz.x, "physical_y_mm": state.physical_xyz.y,
            "physical_z_mm": state.physical_xyz.z,
            "internal_x_mm": state.internal_xyz.x, "internal_y_mm": state.internal_xyz.y,
            "internal_z_mm": state.internal_xyz.z,
            "position_error_mm": position_error_mm,
            "robot_timestamp_utc": state.timestamp_utc,
            "extra": extra,
        }
        if target is not None:
            values.update({"target_x_mm": target.x, "target_y_mm": target.y, "target_z_mm": target.z})
        self.logger.log_telemetry(values, event=event, category="telemetry")

    def _display_robot_state(self, state, announce: bool = False) -> None:
        raw_values = state.raw_positions or (None, None, None, None)
        for motor_id, value, raw in zip(
            (11, 12, 13, 14), state.motors.as_tuple(), raw_values, strict=True
        ):
            suffix = f" | RAW {raw}" if raw is not None else " | RAW unavailable"
            self.manual_angles[motor_id].set(f"{value:.3f} deg{suffix}")
        for joint, value in zip(("q1", "q2", "q3", "q4"), state.joints.as_tuple(), strict=True):
            self.manual_joints[joint].set(f"{value:.3f} deg")
        for axis, value in zip(
            ("X", "Y", "Z"),
            (state.physical_xyz.x, state.physical_xyz.y, state.physical_xyz.z),
            strict=True,
        ):
            self.manual_xyz[axis].set(f"{value:.3f} mm")
        if announce:
            self._log(
                "Live state: motors={} deg; q={} deg; physical XYZ=({:.3f},{:.3f},{:.3f}) mm; "
                "internal XYZ=({:.3f},{:.3f},{:.3f}) mm".format(
                    tuple(round(value, 3) for value in state.motors.as_tuple()),
                    tuple(round(value, 3) for value in state.joints.as_tuple()),
                    state.physical_xyz.x,
                    state.physical_xyz.y,
                    state.physical_xyz.z,
                    state.internal_xyz.x,
                    state.internal_xyz.y,
                    state.internal_xyz.z,
                )
            )

    def read_angles_now(self) -> None:
        if not self.controller.status.connected:
            self._error(ControllerError("Connect to OpenCR before reading angles"))
            return
        if self._telemetry_inflight:
            return
        self._telemetry_inflight = True
        threading.Thread(target=self._read_state_worker, args=(True,), daemon=True).start()

    def _read_state_worker(self, announce: bool = False) -> None:
        try:
            state = self.controller.read_robot_state()
            self.after(0, self._finish_state_read, state, announce)
        except ControllerError as exc:
            self.after(0, self._finish_state_error, exc)

    def _finish_state_read(self, state, announce: bool) -> None:
        self._telemetry_inflight = False
        self._telemetry_failures = 0
        self._display_robot_state(state, announce=announce)
        try:
            error_mm = None
            if self._telemetry_target is not None:
                error_mm = math.dist(self._xyz_tuple(state.physical_xyz), self._xyz_tuple(self._telemetry_target))
            self._record_telemetry(state, target=self._telemetry_target, position_error_mm=error_mm)
        except Exception as exc:
            self._error(exc, event="telemetry_log_failed")

    def _finish_state_error(self, exc: Exception) -> None:
        self._telemetry_inflight = False
        self._telemetry_failures += 1
        if self._telemetry_failures >= 3:
            self.stop_live_read()
            self._error(
                ControllerError(
                    f"Live reading stopped after {self._telemetry_failures} consecutive failures: {exc}"
                ),
                event="live_read_stopped_after_failures",
                context={"consecutive_failures": self._telemetry_failures},
            )
        else:
            self.logger.log_error(
                "live_read_sample_failed",
                exc,
                category="telemetry",
                context={"consecutive_failures": self._telemetry_failures},
            )

    def start_live_read(self) -> None:
        try:
            if not self.controller.status.connected:
                raise ControllerError("Connect to OpenCR before starting live read")
            interval = int(self.live_interval_ms.get())
            if not 100 <= interval <= 5000:
                raise ValueError("Live interval must be between 100 and 5000 ms")
            self._telemetry_enabled = True
            self._telemetry_failures = 0
            self.live_read_status.set("ON")
            self._schedule_live_read()
            self._log(f"Live encoder/TCP reading started at {interval} ms")
        except (ControllerError, ValueError) as exc:
            self._error(exc)

    def _schedule_live_read(self) -> None:
        if not self._telemetry_enabled:
            return
        if self.controller.status.connected and not self._telemetry_inflight:
            self._telemetry_inflight = True
            threading.Thread(target=self._read_state_worker, daemon=True).start()
        interval = max(100, int(self.live_interval_ms.get()))
        self._telemetry_after_id = self.after(interval, self._schedule_live_read)

    def stop_live_read(self) -> None:
        was_enabled = self._telemetry_enabled
        self._telemetry_enabled = False
        self.live_read_status.set("OFF")
        if self._telemetry_after_id is not None:
            self.after_cancel(self._telemetry_after_id)
            self._telemetry_after_id = None
        if was_enabled:
            self._log("Live encoder/TCP reading stopped")

    def connect(self) -> None:
        try:
            self.controller.connect(self.port.get())
            self.torque.set("ON" if self.controller.status.torque_on else "OFF")
            self._log("Connected")
        except ControllerError as exc:
            self._error(exc)

    def disconnect(self) -> None:
        self.stop_live_read()
        self.controller.disconnect()
        self.torque.set("UNKNOWN")
        self._log("Disconnected")

    def _close(self) -> None:
        self._experiment_stop.set()
        self._manual_measurement_continue.set()
        self._play_stop.set()
        try:
            self.stop_live_read()
            if self.controller.status.connected:
                self.controller.disconnect()
            self.logger.log_history("gui_session_closed", category="session")
        except Exception as exc:
            try:
                self.logger.log_error("gui_close_failed", exc, category="session")
            except Exception:
                pass
        finally:
            self.logger.close()
            self.master.destroy()

    def torque_on(self) -> None:
        try:
            self.controller.torque_on()
            self.torque.set("ON")
            self._log("Torque ON")
        except ControllerError as exc:
            self._error(exc)

    def torque_off(self) -> None:
        try:
            self.controller.torque_off()
            self.torque.set("OFF")
            self._log("Torque OFF")
        except ControllerError as exc:
            self._error(exc)

    def read_manual_pose(self) -> None:
        try:
            self.controller.torque_off()
            self.torque.set("OFF")
            motors = self.controller.read_motor_angles()
            xyz = forward_kinematics_physical(motors)
            for motor_id, value in zip((11, 12, 13, 14), motors.as_tuple(), strict=True):
                self.manual_angles[motor_id].set(f"{value:.3f} deg")
            for axis, value in zip(("X", "Y", "Z"), (xyz.x, xyz.y, xyz.z), strict=True):
                self.manual_xyz[axis].set(f"{value:.3f} mm")
            joints = motor_to_fk_angles(motors)
            for joint, value in zip(("q1", "q2", "q3", "q4"), joints.as_tuple(), strict=True):
                self.manual_joints[joint].set(f"{value:.3f} deg")
            self.manual_points.append(TrajectoryPoint(float(len(self.manual_points)), motors, xyz))
            path = save_manual_poses(self.manual_points, self.output_dir)
            self._log(
                "Manual pose: "
                f"motors=({motors.id11:.3f},{motors.id12:.3f},{motors.id13:.3f},{motors.id14:.3f}) deg, "
                f"joints=({joints.theta1:.3f},{joints.theta2:.3f},{joints.theta3:.3f},{joints.theta4:.3f}) deg, "
                f"physical XYZ=({xyz.x:.3f},{xyz.y:.3f},{xyz.z:.3f}) mm; "
                f"saved capture {len(self.manual_points)} to {path.name}"
            )
        except (ControllerError, OSError, ValueError) as exc:
            self._error(exc)

    def start_teaching(self) -> None:
        try:
            self.controller.start_teaching()
            self.recording.set("ON")
            self._log("Teaching started: move the arm by hand")
        except ControllerError as exc:
            self._error(exc)

    def rest(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON first; it will hold the current pose before REST")
            if not messagebox.askyesno(
                "Confirm REST movement",
                "Support the arm and clear the workspace. Move slowly to the straight raised REST pose?",
            ):
                return
            self.controller.move_rest()
            self._log(
                "Moved to REST: q=(0,0,0,0); official frame=(286,0,204.5), "
                f"finger-center TCP=({config.REST_XYZ_MM[0]:.1f},0,{config.REST_XYZ_MM[2]:.1f}) mm"
            )
        except ControllerError as exc:
            self._error(exc)

    def work(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON first; it will hold the current pose before WORK")
            if not messagebox.askyesno(
                "Confirm WORK movement",
                "Clear the table and support the arm. Move slowly to WORK finger-center TCP near "
                f"X={config.WORK_XYZ_MM[0]:.1f}, Y=0, Z={config.WORK_XYZ_MM[2]:.1f} mm?",
            ):
                return
            self.controller.move_work()
            self._log(
                "Moved to WORK: q=(0,0,0,82.881), finger-center FK="
                f"({config.WORK_XYZ_MM[0]:.3f},0,{config.WORK_XYZ_MM[2]:.3f}) mm"
            )
        except ControllerError as exc:
            self._error(exc)

    def stop_teaching(self) -> None:
        self._log("Receiving recorded trajectory...")
        threading.Thread(target=self._receive_teaching, daemon=True).start()

    def _receive_teaching(self) -> None:
        try:
            points = self.controller.stop_teaching()
            full_path, xyz_path = save_workbooks(points, self.output_dir)
            self.after(0, self._finish_receive_teaching, points, full_path, xyz_path)
        except (ControllerError, OSError, ValueError) as exc:
            self.after(0, self._error, exc)

    def _finish_receive_teaching(self, points, full_path: Path, xyz_path: Path) -> None:
        self.points = points
        self.recording.set("OFF")
        self.count.set(str(len(points)))
        self.elapsed.set(f"{points[-1].time_s:.3f} s" if points else "0.000 s")
        self.torque.set("OFF")
        if full_path.name != "trajectory_full.xlsx":
            self._log(
                "Excel has the normal trajectory files open. Saved a timestamped pair instead: "
                f"{full_path.name}, {xyz_path.name}; torque remains OFF"
            )
        else:
            self._log(f"Saved {len(points)} points: {full_path.name}, {xyz_path.name}; torque remains OFF")

    def save_trajectory(self) -> None:
        try:
            full_path, xyz_path = save_workbooks(self.points, self.output_dir)
            self._log(f"Saved {full_path} and {xyz_path}")
        except (OSError, ValueError) as exc:
            self._error(exc)

    def _move_to_teach_endpoint(self, first: bool) -> None:
        try:
            if self.points:
                point = endpoint_points(self.points)[0 if first else 1]
            else:
                point = xyz_endpoint_points(self.xyz_points)[0 if first else 1]
            if not self.controller.status.torque_on:
                self.controller.torque_on()
            current = self.controller.read_motor_angles()
            self.controller.move_xyz_point(point, reference=motor_to_fk_angles(current))
            self._log("Moved to teach start point" if first else "Moved to teach end point")
        except (ControllerError, ValueError) as exc:
            self._error(exc)

    def go_to_teach_start(self) -> None:
        self._move_to_teach_endpoint(True)

    def go_to_teach_end(self) -> None:
        self._move_to_teach_endpoint(False)

    def load_xyz(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("XYZ trajectory", "trajectory_xyz.xlsx"), ("Excel", "*.xlsx")])
        if not path:
            return
        try:
            self.xyz_points = load_xyz_workbook(path)
            self.count.set(str(len(self.xyz_points)))
            self._log(f"Loaded XYZ-only trajectory: {len(self.xyz_points)} points")
        except (OSError, ValueError, KeyError) as exc:
            self._error(exc)

    def validate(self) -> None:
        try:
            report = validate_xyz_trajectory(self.xyz_points)
            self._log(f"Validation: total={report.total}, valid={report.valid}, invalid={report.invalid}, max={report.maximum_error_mm:.3f} mm, average={report.average_error_mm:.3f} mm")
            for error in report.errors[:5]:
                self._log(error)
        except ValueError as exc:
            self._error(exc)

    def play(self) -> None:
        try:
            scale = float(self.speed.get())
            if not 0.05 <= scale <= 2.0:
                raise ValueError("Speed must be between 0.05 and 2.0")
            report = validate_xyz_trajectory(self.xyz_points)
            if report.invalid:
                detail = report.errors[0] if report.errors else "unknown invalid point"
                raise ValueError(f"Playback blocked: {detail}")
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON and move to TEACH START before playback")
            if not messagebox.askyesno(
                "Confirm trajectory playback",
                "The robot will move. Is the workspace clear and the arm supported?",
            ):
                return
            self._play_stop.clear()
            threading.Thread(target=self._play_worker, args=(scale,), daemon=True).start()
            self.playback.set("ON")
            self._log(f"Cartesian playback started at speed scale {scale:.2f}")
        except (ControllerError, ValueError) as exc:
            self._error(exc)

    def _play_worker(self, scale: float) -> None:
        try:
            self.controller.play_xyz_trajectory(self.xyz_points, scale, self._play_stop)
            self.after(0, lambda: self._log("Cartesian playback finished"))
            self.after(0, lambda: self.playback.set("OFF"))
        except (ControllerError, ValueError) as exc:
            self.after(0, lambda: self._error(exc))
            self.after(0, lambda: self.playback.set("OFF"))

    def stop(self) -> None:
        try:
            self._play_stop.set()
            self.controller.stop()
            self.playback.set("OFF")
            self._log("Playback stopped")
        except ControllerError as exc:
            self._error(exc)


def run() -> None:
    root = tk.Tk()
    root.title("OpenMANIPULATOR-X Cartesian and Repeatability Lab")
    root.geometry("1100x850")
    root.minsize(760, 600)
    TrajectoryGUI(root)
    root.mainloop()
