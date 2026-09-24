"""Tkinter GUI for button-triggered OpenMANIPULATOR-X control."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import config
from kinematics import (
    KinematicsError,
    MotorAngles,
    forward_kinematics,
    inverse_kinematics,
    motor_to_fk_angles,
    validate_ik_solution,
)
from serial_controller import ControllerError, OpenCRController, available_ports


class OpenManipulatorGUI(ttk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padding=12)
        self.controller = OpenCRController()
        self.port_var = tk.StringVar()
        self.torque_var = tk.StringVar(value="TORQUE: UNKNOWN")
        self.angle_vars = {motor_id: tk.StringVar(value="---") for motor_id in (11, 12, 13, 14)}
        self.joint_vars = {axis: tk.StringVar(value="---") for axis in ("theta1", "theta2", "theta3", "theta4")}
        self.xyz_vars = {axis: tk.StringVar(value="---") for axis in ("X", "Y", "Z")}
        self.target_vars = {axis: tk.StringVar() for axis in ("X", "Y", "Z")}
        self.status_var = tk.StringVar(value="Disconnected")
        self.port_combo: ttk.Combobox | None = None
        self._build()
        self._refresh_ports()

    def _build(self) -> None:
        self.grid(sticky="nsew")
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

        self._connection_section().grid(row=0, column=0, columnspan=2, sticky="ew", pady=4)
        self._torque_section().grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=4)
        self._manual_section().grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=4)
        self._xyz_read_section().grid(row=2, column=0, sticky="nsew", padx=(0, 6), pady=4)
        self._xyz_command_section().grid(row=2, column=1, sticky="nsew", padx=(6, 0), pady=4)
        self._log_section().grid(row=3, column=0, columnspan=2, sticky="nsew", pady=4)

    def _labeled_frame(self, title: str) -> ttk.LabelFrame:
        frame = ttk.LabelFrame(self, text=title, padding=10)
        frame.columnconfigure(1, weight=1)
        return frame

    def _connection_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("Connection")
        ttk.Label(frame, text="COM Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, values=[], width=16)
        self.port_combo.grid(row=0, column=1, sticky="ew")
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="CONNECT", command=self.on_connect).grid(row=1, column=0, pady=6)
        ttk.Button(frame, text="DISCONNECT", command=self.on_disconnect).grid(row=1, column=1, pady=6)
        return frame

    def _torque_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("Torque")
        ttk.Button(frame, text="TORQUE ON", command=self.on_torque_on).grid(row=0, column=0, padx=3, pady=3)
        ttk.Button(frame, text="TORQUE OFF", command=self.on_torque_off).grid(row=0, column=1, padx=3, pady=3)
        ttk.Label(frame, textvariable=self.torque_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=8)
        ttk.Button(frame, text="SCAN (CAMERA)", command=self.on_scan).grid(row=2, column=0, sticky="ew")
        ttk.Button(frame, text="WORK (DOWN)", command=self.on_work).grid(row=2, column=1, sticky="ew")
        return frame

    def _manual_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("Manual Position Reading")
        ttk.Button(frame, text="READ MOTOR ANGLES", command=self.on_read_angles).grid(row=0, column=0, columnspan=2, sticky="ew")
        for row, motor_id in enumerate((11, 12, 13, 14), start=1):
            ttk.Label(frame, text=f"ID{motor_id}:").grid(row=row, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.angle_vars[motor_id]).grid(row=row, column=1, sticky="w")
        ttk.Label(frame, text="ID14: SCAN q4 about 124.570 deg; WORK q4 about 82.881 deg").grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        ttk.Label(frame, text="FK theta4:").grid(row=6, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.joint_vars["theta4"]).grid(row=6, column=1, sticky="w")
        return frame

    def _xyz_read_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("XYZ Reading")
        ttk.Button(frame, text="GET XYZ", command=self.on_get_xyz).grid(row=0, column=0, sticky="ew")
        ttk.Button(frame, text="READ POSITION", command=self.on_read_position).grid(row=0, column=1, sticky="ew")
        for row, axis in enumerate(("X", "Y", "Z"), start=1):
            ttk.Label(frame, text=f"{axis}:").grid(row=row, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.xyz_vars[axis]).grid(row=row, column=1, sticky="w")
        return frame

    def _xyz_command_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("XYZ Command")
        for row, axis in enumerate(("X", "Y", "Z")):
            ttk.Label(frame, text=f"Target {axis}:").grid(row=row, column=0, sticky="w")
            ttk.Entry(frame, textvariable=self.target_vars[axis]).grid(row=row, column=1, sticky="ew")
        ttk.Button(frame, text="COPY CURRENT XYZ TO TARGET", command=self.on_copy_xyz_to_target).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(4, 2)
        )
        ttk.Button(frame, text="MOVE TO XYZ", command=self.on_move_xyz).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=4
        )
        return frame

    def _log_section(self) -> ttk.LabelFrame:
        frame = self._labeled_frame("Status / Log")
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        self.log = tk.Text(frame, height=9, wrap="word")
        self.log.grid(row=1, column=0, sticky="nsew")
        return frame

    def _refresh_ports(self) -> None:
        ports = available_ports()
        if self.port_combo is not None:
            self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _log(self, text: str) -> None:
        self.status_var.set(text)
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def _handle_error(self, exc: Exception) -> None:
        self._log(f"ERROR: {exc}")

    def _display_angles(self, motors: MotorAngles) -> None:
        for motor_id, value in zip((11, 12, 13, 14), motors.as_tuple(), strict=True):
            self.angle_vars[motor_id].set(f"{value:.3f} deg")
        joints = motor_to_fk_angles(motors)
        for axis, value in zip(self.joint_vars, joints.as_tuple(), strict=True):
            self.joint_vars[axis].set(f"{value:.3f} deg")

    def _display_xyz(self, motors: MotorAngles) -> None:
        xyz = forward_kinematics(motors)
        self.xyz_vars["X"].set(f"{xyz.x:.3f} mm")
        self.xyz_vars["Y"].set(f"{xyz.y:.3f} mm")
        self.xyz_vars["Z"].set(f"{xyz.z:.3f} mm")
        self._log(f"FK calculated: X={xyz.x:.3f}, Y={xyz.y:.3f}, Z={xyz.z:.3f} mm")

    def on_connect(self) -> None:
        try:
            self.controller.connect(self.port_var.get())
            state = "ON" if self.controller.status.torque_on else "OFF"
            self.torque_var.set(f"TORQUE: {state}")
            self._log("Connected")
        except ControllerError as exc:
            self._handle_error(exc)

    def on_disconnect(self) -> None:
        self.controller.disconnect()
        self.torque_var.set("TORQUE: UNKNOWN")
        self._log("Disconnected")

    def on_torque_on(self) -> None:
        try:
            self.controller.torque_on()
            self.torque_var.set("TORQUE: ON")
            self._log("Torque ON")
        except ControllerError as exc:
            self._handle_error(exc)

    def on_torque_off(self) -> None:
        try:
            self.controller.torque_off()
            self.torque_var.set("TORQUE: OFF")
            self._log("Torque OFF")
        except ControllerError as exc:
            self._handle_error(exc)

    def on_read_angles(self) -> MotorAngles | None:
        try:
            self._log("Reading motors...")
            motors = self.controller.read_motor_angles()
            self._display_angles(motors)
            self._log("Angles received")
            return motors
        except ControllerError as exc:
            self._handle_error(exc)
            return None

    def on_get_xyz(self) -> None:
        motors = self.on_read_angles()
        if motors is not None:
            try:
                self._display_xyz(motors)
            except KinematicsError as exc:
                self._handle_error(exc)

    def on_read_position(self) -> None:
        self.on_get_xyz()

    def on_copy_xyz_to_target(self) -> None:
        for axis in ("X", "Y", "Z"):
            val_str = self.xyz_vars[axis].get().replace(" mm", "").strip()
            if val_str and val_str != "---":
                try:
                    val = float(val_str)
                    self.target_vars[axis].set(f"{val:.1f}")
                except ValueError:
                    pass
        self._log("Current XYZ copied to Target inputs")

    def on_move_xyz(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Cannot move: Torque is OFF")
            try:
                target = {axis: float(self.target_vars[axis].get().strip()) for axis in ("X", "Y", "Z")}
            except ValueError:
                raise ControllerError("Invalid XYZ input: Please enter valid numbers for Target X, Y, and Z")
            current = self.controller.read_motor_angles()
            reference = motor_to_fk_angles(current)
            result = inverse_kinematics(target["X"], target["Y"], target["Z"], reference=reference)
            validate_ik_solution(result)
            self._log(
                "IK calculated: "
                f"theta=({result.joint_angles.theta1:.3f}, {result.joint_angles.theta2:.3f}, "
                f"{result.joint_angles.theta3:.3f}, {result.joint_angles.theta4:.3f}), "
                f"FK error={result.position_error:.3f} mm"
            )
            self._log("Moving...")
            self.controller.move_motor_angles(result.motor_angles)
            self._log("Done")
        except (ControllerError, KinematicsError) as exc:
            self._handle_error(exc)

    def on_scan(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Cannot move: Torque is OFF")
            self._log("Moving to SCAN...")
            self.controller.move_scan()
            self._log(
                "SCAN reached: finger-center TCP="
                f"({config.SCAN_XYZ_MM[0]:.1f},0,{config.SCAN_XYZ_MM[2]:.1f}) mm"
            )
        except ControllerError as exc:
            self._handle_error(exc)

    def on_rest(self) -> None:
        self.on_scan()

    def on_work(self) -> None:
        try:
            if not self.controller.status.torque_on:
                raise ControllerError("Cannot move: Torque is OFF")
            self._log("Moving to WORK...")
            self.controller.move_work()
            self._log(
                "WORK reached: finger-center FK="
                f"({config.WORK_XYZ_MM[0]:.3f},0,{config.WORK_XYZ_MM[2]:.3f}) mm"
            )
        except ControllerError as exc:
            self._handle_error(exc)


def run() -> None:
    root = tk.Tk()
    root.title("OpenMANIPULATOR-X XYZ Controller")
    root.geometry("760x620")
    OpenManipulatorGUI(root)
    root.mainloop()
