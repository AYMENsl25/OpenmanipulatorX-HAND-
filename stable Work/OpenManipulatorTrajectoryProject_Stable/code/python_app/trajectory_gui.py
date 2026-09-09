"""GUI for teach-by-hand recording and Cartesian XYZ replay."""

from __future__ import annotations

import threading
from threading import Event
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import config
from kinematics import (
    KinematicsError,
    forward_kinematics,
    inverse_kinematics,
    motor_to_fk_angles,
    normalize_angle,
    validate_ik_solution,
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
        self.elapsed = tk.StringVar(value="0.000 s")
        self.count = tk.StringVar(value="0")
        self.speed = tk.StringVar(value="0.25")
        self.target_xyz = {
            "X": tk.StringVar(value="180.5"),
            "Y": tk.StringVar(value="0.0"),
            "Z": tk.StringVar(value="40.1"),
        }
        self.ik_preview = tk.StringVar(value="Enter XYZ, then preview before moving")
        self.manual_xyz = {axis: tk.StringVar(value="---") for axis in ("X", "Y", "Z")}
        self.manual_angles = {motor_id: tk.StringVar(value="---") for motor_id in (11, 12, 13, 14)}
        self.manual_joints = {joint: tk.StringVar(value="---") for joint in ("q1", "q2", "q3", "q4")}
        self.points = []
        self.manual_points: list[TrajectoryPoint] = []
        self.xyz_points: list[XYZTrajectoryPoint] = []
        self._play_stop = Event()
        self.output_dir = Path(__file__).resolve().parents[1]
        self._build()
        self._refresh_ports()

    def _build(self) -> None:
        self.grid(sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self._connection().grid(row=0, column=0, columnspan=2, sticky="ew", pady=4)
        self._controls().grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=4)
        self._replay().grid(row=1, column=1, sticky="nsew", padx=(5, 0), pady=4)
        self._status().grid(row=2, column=0, columnspan=2, sticky="nsew", pady=4)

    def _frame(self, title: str) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(self, text=title, padding=8)
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
        return frame

    def _controls(self) -> ttk.LabelFrame:
        frame = self._frame("Teach By Hand")
        frame.columnconfigure(3, weight=1)
        ttk.Button(frame, text="START TEACHING", command=self.start_teaching).grid(row=0, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="STOP TEACHING", command=self.stop_teaching).grid(row=1, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="SAVE TRAJECTORY", command=self.save_trajectory).grid(row=2, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="CAPTURE + SAVE MANUAL POSE", command=self.read_manual_pose).grid(row=3, column=0, columnspan=2, sticky="ew", pady=3)
        self._label_value(frame, 4, "Recording", self.recording)
        self._label_value(frame, 5, "Time", self.elapsed)
        self._label_value(frame, 6, "Points", self.count)
        ttk.Label(frame, text="Encoder", font=("TkDefaultFont", 9, "bold")).grid(row=7, column=0, columnspan=2, sticky="w")
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
            text="REST ID14=0, q4=0; WORK ID14=82.881 deg, q4=82.881 deg, tip TCP~(180.5,0,40.1) mm",
        ).grid(row=15, column=0, columnspan=4, sticky="w", pady=(8, 0))
        ttk.Button(frame, text="GO TO TEACH START", command=self.go_to_teach_start).grid(row=16, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(frame, text="GO TO TEACH END", command=self.go_to_teach_end).grid(row=17, column=0, columnspan=2, sticky="ew", pady=3)
        return frame

    def _replay(self) -> ttk.LabelFrame:
        frame = self._frame("Cartesian Replay")
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
        ttk.Label(frame, textvariable=self.ik_preview, wraplength=390).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(2, 8)
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

    def _nearest_ik_from_current(self):
        try:
            target = tuple(float(self.target_xyz[axis].get().strip()) for axis in ("X", "Y", "Z"))
        except ValueError as exc:
            raise ValueError("Enter numeric X, Y, and Z values in millimetres") from exc
        current_motors = self.controller.read_motor_angles()
        current_joints = motor_to_fk_angles(current_motors)
        result = inverse_kinematics(*target, reference=current_joints)
        validate_ik_solution(result)
        deltas = tuple(
            normalize_angle(goal - current)
            for goal, current in zip(result.joint_angles.as_tuple(), current_joints.as_tuple(), strict=True)
        )
        return target, current_motors, current_joints, result, deltas

    def preview_xyz(self) -> None:
        try:
            target, _motors, _joints, result, deltas = self._nearest_ik_from_current()
            text = (
                f"Target=({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) mm; "
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
            target, _motors, _joints, result, deltas = self._nearest_ik_from_current()
            preview = (
                f"Target XYZ: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) mm\n"
                f"Joint movement: ({deltas[0]:+.1f}, {deltas[1]:+.1f}, {deltas[2]:+.1f}, {deltas[3]:+.1f}) deg\n"
                f"Predicted error: {result.position_error:.3f} mm\n\n"
                "Clear the workspace and support the arm. Move now?"
            )
            if not messagebox.askyesno("Confirm nearest IK movement", preview):
                self._log("XYZ movement cancelled")
                return
            self.controller.move_motor_angles(result.motor_angles)
            actual = self.controller.read_motor_angles()
            actual_xyz = forward_kinematics(actual)
            self._log(
                f"XYZ move complete; measured FK=({actual_xyz.x:.3f}, {actual_xyz.y:.3f}, "
                f"{actual_xyz.z:.3f}) mm"
            )
        except (ControllerError, KinematicsError, ValueError) as exc:
            self._error(exc)

    def _status(self) -> ttk.LabelFrame:
        frame = self._frame("Status")
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
        self.port_box.configure(values=ports)
        if not self.port.get():
            self.port.set("COM7" if "COM7" in ports else (ports[0] if ports else "COM7"))

    def _log(self, message: str) -> None:
        self.status.set(message)
        self.log.insert("end", message + "\n")
        self.log.see("end")

    def _error(self, exc: Exception) -> None:
        self._log(f"ERROR: {exc}")

    def connect(self) -> None:
        try:
            self.controller.connect(self.port.get())
            self.torque.set("ON" if self.controller.status.torque_on else "OFF")
            self._log("Connected")
        except ControllerError as exc:
            self._error(exc)

    def disconnect(self) -> None:
        self.controller.disconnect()
        self.torque.set("UNKNOWN")
        self._log("Disconnected")

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
            xyz = forward_kinematics(motors)
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
                f"XYZ=({xyz.x:.3f},{xyz.y:.3f},{xyz.z:.3f}) mm; "
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
            self._log("Moved to REST: q=(0,0,0,0); official frame=(286,0,204.5), tip TCP=(325.7,0,204.5) mm")
        except ControllerError as exc:
            self._error(exc)

    def work(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Turn torque ON first; it will hold the current pose before WORK")
            if not messagebox.askyesno(
                "Confirm WORK movement",
                "Clear the table and support the arm. Move slowly to WORK tip TCP near X=180.5, Y=0, Z=40.1 mm?",
            ):
                return
            self.controller.move_work()
            self._log("Moved to WORK: q=(0,0,0,82.881), tip FK=(180.536,0,40.077) mm")
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
    root.title("OpenMANIPULATOR-X Cartesian Teach and Replay")
    root.geometry("900x780")
    TrajectoryGUI(root)
    root.mainloop()
