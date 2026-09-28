"""Read-only camera/scan tab for the existing controller GUI."""

from __future__ import annotations

import base64
import csv
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import config

from app_paths import WORKSPACE_ROOT
ROOT = WORKSPACE_ROOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scan_cube_xyz_preview import ScanCalibration, process_frame, rotate_center_calibration  # noqa: E402
from experiment_frame import forward_kinematics_physical, inverse_kinematics_physical  # noqa: E402
from kinematics import JointAngles, fk_to_motor_angles, validate_motor_angles  # noqa: E402
from rear_reference_frame import (axis_to_rear_xyz, rear_to_axis_x,
                                  rear_to_internal_transform)  # noqa: E402
from serial_controller import ControllerError  # noqa: E402


LOG_FIELDS = ("timestamp", "scan_pose", "joint_1", "joint_2", "joint_3", "joint_4",
              "fk_x", "fk_y", "fk_z", "object_id", "status", "pixel_u", "pixel_v",
              "bbox_w", "bbox_h", "contour_area", "provisional_x", "provisional_y", "provisional_z",
              "xy_method", "nearest_edge", "scan_cycle", "frame_index",
              "fk_frame", "plate_frame", "axis_x_preview")


class VisionPanel(ttk.Frame):
    def __init__(self, parent: tk.Widget, *, controller=None, log_callback=None,
                 move_center_callback=None, pick_selected_callback=None) -> None:
        super().__init__(parent, padding=6)
        self.camera = None
        self.controller = controller
        self.log_callback = log_callback
        self.move_center_callback = move_center_callback
        self.pick_selected_callback = pick_selected_callback
        self.after_id = None
        self.photo = None
        self.raw_frame = None
        self.last_view = None
        self.frame_lock = threading.Lock()
        self.frame_seq = 0
        self.display_scale = 1.0
        self.view_width = 640
        self.results: list[dict] = []
        self.robot_state = None
        self.detect_enabled = tk.BooleanVar(value=True)
        self.camera_index = tk.StringVar(value="1")
        self.pose_name = tk.StringVar(value="CENTER")
        self.reference_name = tk.StringVar(value="P1")
        self.reference_x = tk.StringVar()
        self.reference_y = tk.StringVar()
        self.scan_delta = tk.StringVar(value="10")
        self.scan_span = tk.StringVar()
        self.scan_settle = tk.StringVar(value="0.8")
        self.left_mapping = tk.StringVar(value="UNVERIFIED - CHOOSE")
        self.path_verified = tk.BooleanVar(value=False)
        self.motion_status = tk.StringVar(value="IDLE - no scan motion running")
        self.scan_running = False
        self.scan_stop_requested = False
        self.info = tk.StringVar(value="Read robot angles to show hand XYZ.")
        self.coordinate_help = tk.StringVar(
            value="Pixel X/Y are image locations. Cube X/Y needs 4 clicked references; cube Z is not calibrated yet.")
        self.preview = tk.StringVar(value="Preview only; no motor command")
        self.calibration_quality = tk.StringVar(value="P1-P4 clicks needed for plate map.")
        self.calibration = ScanCalibration.load()
        self.center_calibration = self.calibration
        self._build()
        self._update_reference_table()
        self._refresh_info()

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(self)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        ttk.Label(toolbar, text="Camera index").pack(side="left")
        ttk.Entry(toolbar, textvariable=self.camera_index, width=4).pack(side="left", padx=3)
        ttk.Button(toolbar, text="START CAMERA", command=self.start_camera).pack(side="left", padx=2)
        ttk.Button(toolbar, text="STOP CAMERA", command=self.stop_camera).pack(side="left", padx=2)
        ttk.Checkbutton(toolbar, text="DETECT", variable=self.detect_enabled).pack(side="left", padx=5)
        ttk.Button(toolbar, text="READ HAND XYZ", command=self.read_hand_xyz).pack(side="left", padx=2)
        ttk.Button(toolbar, text="SAVE CURRENT VIEW CSV", command=self.save_snapshot).pack(side="left", padx=8)

        viewport = ttk.Frame(self)
        viewport.grid(row=1, column=0, sticky="nsew")
        viewport.rowconfigure(0, weight=1)
        viewport.columnconfigure(0, weight=1)
        self.scroll_canvas = tk.Canvas(viewport, highlightthickness=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(viewport, orient="vertical", command=self.scroll_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.scroll_canvas.configure(yscrollcommand=scrollbar.set)
        self.content = ttk.Frame(self.scroll_canvas)
        self.content.columnconfigure(0, weight=1)
        self.content_window = self.scroll_canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.content.bind("<Configure>", lambda _e: self.scroll_canvas.configure(
            scrollregion=self.scroll_canvas.bbox("all")))
        self.scroll_canvas.bind("<Configure>", self._resize_content)
        self.scroll_canvas.bind("<Enter>", lambda _e: self.scroll_canvas.bind_all(
            "<MouseWheel>", self._on_mousewheel))
        self.scroll_canvas.bind("<Leave>", lambda _e: self.scroll_canvas.unbind_all("<MouseWheel>"))

        self.image = tk.Canvas(self.content, width=640, height=480, highlightthickness=1,
                               highlightbackground="#6b7280")
        self.image.grid(row=0, column=0, sticky="nw", pady=(0, 6))
        self.image.create_text(320, 240, text="Camera stopped", tags="frame")
        self.image.bind("<Button-1>", self._click_reference)

        positions = ttk.LabelFrame(self.content, text="Robot and detected cube coordinates", padding=5)
        positions.grid(row=1, column=0, sticky="ew", pady=(0, 5))
        positions.columnconfigure(0, weight=1)
        ttk.Label(positions, textvariable=self.info, justify="left").grid(
            row=0, column=0, sticky="ew", pady=(0, 2))
        ttk.Label(positions, textvariable=self.coordinate_help, wraplength=900,
                  justify="left").grid(row=1, column=0, sticky="ew", pady=(0, 4))
        columns = ("ID", "STATUS", "PICK", "Pixel X", "Pixel Y", "Plate X rear", "Plate X axis", "Plate Y est",
                   "Cube Z mm", "XY source", "W px", "H px", "EDGE")
        self.table = ttk.Treeview(positions, columns=columns, show="headings", height=6)
        for name in columns:
            self.table.heading(name, text=name)
            self.table.column(name, width=108 if name in ("XY source", "PICK") else
                              72 if name not in ("STATUS", "Plate X rear", "Plate X axis", "Plate Y est", "Cube Z mm")
                              else 88, anchor="center", stretch=False)
        self.table.grid(row=2, column=0, sticky="ew")
        table_scroll = ttk.Scrollbar(positions, orient="horizontal", command=self.table.xview)
        table_scroll.grid(row=3, column=0, sticky="ew")
        self.table.configure(xscrollcommand=table_scroll.set)
        if self.pick_selected_callback is not None:
            self.table.bind("<ButtonRelease-1>", self._pick_from_result_row)
            ttk.Button(positions, text="PICK IT UP (SELECTED CUBE)…",
                       command=self.pick_selected_callback).grid(
                row=4, column=0, sticky="w", pady=(5, 0))

        calibration = ttk.LabelFrame(self.content, text="Calibrate cube X/Y (fixed camera pose)", padding=5)
        calibration.grid(row=2, column=0, sticky="ew", pady=(0, 5))
        calibration.columnconfigure(3, weight=1)
        ttk.Label(calibration, text="Calibration profile:").grid(row=0, column=0, sticky="w")
        poses = ttk.Combobox(calibration, textvariable=self.pose_name,
                             values=("CENTER", "LEFT", "RIGHT"), width=10, state="readonly")
        poses.grid(row=0, column=1, sticky="w", padx=4)
        poses.bind("<<ComboboxSelected>>", self._change_pose)
        ttk.Label(
            calibration,
            text="Select CENTER/LEFT/RIGHT for the camera's current fixed pose (this selection does not move the arm). "
                 "Select a point below, then click that same mark in the image; keep the arm still. "
                 f"The {self.calibration.physical_bounds_padding_mm:g} mm border extension is "
                 "detection-only, not a robot motion limit.",
            wraplength=760, justify="left",
        ).grid(row=1, column=0, columnspan=4, sticky="ew", pady=(3, 5))
        ttk.Label(calibration, text="Known red mark:").grid(row=2, column=0, sticky="w")
        self.reference_box = ttk.Combobox(calibration, textvariable=self.reference_name,
                                          values=list(self.calibration.references_xy_mm), width=21,
                                          state="readonly")
        self.reference_box.grid(row=2, column=1, sticky="w", padx=4)
        ttk.Label(calibration, text="Click the matching mark in the camera image.").grid(
            row=2, column=2, columnspan=2, sticky="w")
        ttk.Label(calibration, text="Extra measured point X rear / Y (mm):").grid(row=3, column=0, sticky="w")
        ttk.Entry(calibration, textvariable=self.reference_x, width=7).grid(row=3, column=1, sticky="w", padx=(4, 0))
        ttk.Entry(calibration, textvariable=self.reference_y, width=7).grid(row=3, column=1, sticky="w", padx=(65, 0))
        ttk.Button(calibration, text="ADD EXTRA POINT", command=self.add_reference).grid(
            row=3, column=2, sticky="w", padx=4)
        ttk.Label(calibration, textvariable=self.calibration_quality, wraplength=760,
                  justify="left", foreground="#7c2d12").grid(
            row=4, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        columns = ("Point", "X rear", "X axis", "Y mm", "Clicked pixel", "Corner check")
        self.reference_table = ttk.Treeview(calibration, columns=columns, show="headings", height=9)
        for name, width in (("Point", 65), ("X rear", 80), ("X axis", 80), ("Y mm", 80),
                            ("Clicked pixel", 125), ("Corner check", 145)):
            self.reference_table.heading(name, text=name)
            self.reference_table.column(name, width=width, anchor="center", stretch=False)
        self.reference_table.grid(row=5, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.reference_table.bind("<<TreeviewSelect>>", self._select_reference_row)

        scan = ttk.LabelFrame(self.content, text="Move and scan", padding=5)
        scan.grid(row=3, column=0, sticky="ew")
        for col in range(6):
            scan.columnconfigure(col, weight=1)
        ttk.Label(scan, text="LEFT convention:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(scan, textvariable=self.left_mapping,
                     values=("UNVERIFIED - CHOOSE", "LEFT = J1 +", "LEFT = J1 -"),
                     width=20, state="readonly").grid(row=0, column=1, columnspan=2, sticky="w", padx=4)
        ttk.Label(scan, text="Step °:").grid(row=0, column=3, sticky="e")
        ttk.Entry(scan, textvariable=self.scan_delta, width=6).grid(row=0, column=4, sticky="w", padx=3)
        ttk.Button(scan, text="PREVIEW STEP", command=self.preview_scan).grid(row=0, column=5, sticky="w")
        ttk.Button(scan, text="MOVE CAMERA LEFT", command=lambda: self.move_camera("LEFT")).grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(scan, text="MOVE CAMERA RIGHT", command=lambda: self.move_camera("RIGHT")).grid(
            row=1, column=2, columnspan=2, sticky="ew", padx=3, pady=3)
        if self.move_center_callback is not None:
            ttk.Button(scan, text="MOVE TO CENTER SCAN", command=self.move_to_center_scan).grid(
                row=1, column=4, columnspan=2, sticky="ew", pady=3)
        ttk.Label(scan, text="Total half-span °:").grid(row=2, column=0, sticky="w")
        ttk.Entry(scan, textvariable=self.scan_span, width=6).grid(row=2, column=1, sticky="w")
        ttk.Label(scan, text="Settle s:").grid(row=2, column=2, sticky="e")
        ttk.Entry(scan, textvariable=self.scan_settle, width=6).grid(row=2, column=3, sticky="w", padx=3)
        ttk.Checkbutton(scan, text="I verified clear path and both endpoints",
                        variable=self.path_verified).grid(row=2, column=4, columnspan=2, sticky="w")
        ttk.Button(scan, text="SCAN LEFT AREA", command=lambda: self.start_total_scan("LEFT")).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(scan, text="SCAN RIGHT AREA", command=lambda: self.start_total_scan("RIGHT")).grid(
            row=3, column=2, columnspan=2, sticky="ew", padx=3, pady=3)
        ttk.Button(scan, text="SCAN BOTH AREAS", command=lambda: self.start_total_scan("BOTH")).grid(
            row=3, column=4, columnspan=2, sticky="ew", pady=3)
        ttk.Button(scan, text="STOP AFTER CURRENT STEP", command=self.stop_total_scan).grid(
            row=4, column=0, columnspan=6, sticky="ew", pady=3)
        ttk.Label(scan, textvariable=self.motion_status, foreground="#7c2d12").grid(
            row=5, column=0, columnspan=6, sticky="ew")
        ttk.Label(scan, textvariable=self.preview, wraplength=820, justify="left").grid(
            row=6, column=0, columnspan=6, sticky="ew", pady=(3, 0))

    def set_robot_state(self, state) -> None:
        self.robot_state = state
        self._refresh_info()

    def read_hand_xyz(self) -> None:
        if self.controller is None or not self.controller.status.connected:
            messagebox.showerror("Robot position", "Connect to OpenCR first, then read the hand position.")
            return
        try:
            self.set_robot_state(self.controller.read_robot_state())
        except ControllerError as exc:
            messagebox.showerror("Robot position", f"Could not read robot angles: {exc}")

    def _resize_content(self, event) -> None:
        self.scroll_canvas.itemconfigure(self.content_window, width=event.width)
        available = max(320, event.width - 4)
        target_width = min(640, available)
        if target_width != self.view_width:
            self.view_width = target_width
            self.image.configure(width=target_width, height=max(1, round(target_width * 3 / 4)))
            self._render_frame()

    def _on_mousewheel(self, event):
        self.scroll_canvas.yview_scroll(int(-event.delta / 120), "units")

    def _render_frame(self, view=None) -> None:
        if view is None:
            view = self.last_view
        if view is None:
            return
        source_h, source_w = view.shape[:2]
        self.display_scale = min(1.0, self.view_width / source_w)
        shown = view if self.display_scale == 1.0 else cv2.resize(
            view, (self.view_width, max(1, round(source_h * self.display_scale))),
            interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(shown, cv2.COLOR_BGR2RGB)
        ok_png, encoded = cv2.imencode(".png", rgb)
        if ok_png:
            self.photo = tk.PhotoImage(data=base64.b64encode(encoded.tobytes()).decode("ascii"))
            self.image.delete("frame")
            self.image.create_image(0, 0, anchor="nw", image=self.photo, tags="frame")

    def _refresh_info(self) -> None:
        state = self.robot_state
        effective = self._effective_calibration(self.calibration, state)
        corners = ("P1", "P2", "P3", "P4")
        clicked_corners = sum(name in self.calibration.image_points_uv for name in corners)
        checks = self.calibration.corner_check_errors_mm()
        if checks:
            details = ", ".join(f"{name} {error:.1f} mm" for name, error in checks.items())
            warning = (" Check measured X/Y, point clicks and camera distortion before using cube XY."
                       if max(checks.values()) > 5 else "")
            self.calibration_quality.set(
                f"{clicked_corners}/4 corner clicks. Independent check against corners: {details}.{warning}")
        else:
            self.calibration_quality.set(
                f"{clicked_corners}/4 corner clicks. Click P5-P9 to check/refine the map; "
                "four corners alone cannot measure calibration error.")
        if effective.matrix() is None:
            geometry = "PIXELS_ONLY"
        elif effective.mapping_mode == "ROTATED_CENTER":
            geometry = f"CENTER_YAW_TRANSFER (plate plane, {effective.yaw_delta_deg:+.1f}°)"
        else:
            geometry = "FIXED_HOMOGRAPHY (plate plane)"
        if effective.matrix() is None:
            self.coordinate_help.set(
                f"Pixel X/Y are image coordinates (u/v). Cube X/Y need 4 confirmed CENTER plate-mark "
                f"clicks ({len(self.center_calibration.image_points_uv)}/4) at the CENTER scan pose. "
                "Cube Z is unknown; hand XYZ above is the robot hand position.")
        else:
            self.coordinate_help.set(
                "Pixel X/Y are image coordinates. Plate X/Y are rear-referenced plane estimates; the detector "
                "centre may not lie on that plane. Robot cube XYZ needs measured intrinsics, hand-eye "
                "geometry, and a cube-top pixel. No grasp target is generated from these estimates.")
        if state is None:
            self.info.set(f"Joint angles / hand FK: not read\n"
                          f"Camera pose source: unavailable | Localization: {geometry}\n"
                          f"Detected: {len(self.results)} | Robot cube XYZ: unavailable")
            return
        xyz = state.physical_xyz
        rear_xyz = axis_to_rear_xyz(xyz)
        joints = ", ".join(f"{q:.2f}" for q in state.joints.as_tuple())
        camera_source = ("measured J1 + CENTER plane model" if effective.mapping_mode == "ROTATED_CENTER"
                         else "fixed-pose image mapping" if effective.matrix() is not None
                         else "not calibrated")
        self.info.set(f"Joint angles (deg): {joints}\n"
                      f"Hand FK from ID11 rear (mm): {rear_xyz.x:.1f}, {rear_xyz.y:.1f}, {rear_xyz.z:.1f}\n"
                      f"Hand FK from ID11 axis (mm): {xyz.x:.1f}, {xyz.y:.1f}, {xyz.z:.1f}\n"
                      f"Camera pose source: {camera_source} | Localization: {geometry}\n"
                      f"Detected: {len(self.results)} | Robot cube XYZ: unavailable")

    def _effective_calibration(self, calibration: ScanCalibration, state):
        if state is not None and calibration.matrix() is not None:
            center_j1 = (self.center_calibration.calibrated_joints_deg[0]
                         if self.center_calibration.calibrated_joints_deg is not None
                         else float(config.SCAN_JOINT_DEGREES[0]))
            if (calibration.matches_joints(state.joints, tolerance_deg=0.1)
                    and self._pose_direction_matches(calibration.pose, state.joints.theta1,
                                                     center_j1, self.left_mapping.get())):
                return calibration
        if state is not None:
            rotated = rotate_center_calibration(
                self.center_calibration, state.joints, rear_to_internal_transform())
            if rotated is not None:
                return replace(rotated, pose=calibration.pose)
        return replace(calibration, image_points_uv={})

    @staticmethod
    def _pose_direction_matches(pose: str, joint1: float, center_joint1: float,
                                left_mapping: str) -> bool:
        if pose == "CENTER":
            return True
        if pose not in ("LEFT", "RIGHT") or left_mapping not in ("LEFT = J1 +", "LEFT = J1 -"):
            return False
        left_sign = 1.0 if left_mapping == "LEFT = J1 +" else -1.0
        expected_sign = left_sign if pose == "LEFT" else -left_sign
        return (joint1 - center_joint1) * expected_sign > 0.1

    def _change_pose(self, _event=None) -> None:
        self.calibration = ScanCalibration.load(pose=self.pose_name.get())
        if self.calibration.pose == "CENTER":
            self.center_calibration = self.calibration
        self.reference_box.configure(values=list(self.calibration.references_xy_mm))
        self.reference_name.set(next(iter(self.calibration.references_xy_mm)))
        self._update_reference_table()
        self.results = []
        self._update_table()
        self._refresh_info()
        self.preview.set("Pose label changed. Confirm the arm is at that fixed pose; no motion was sent.")

    def add_reference(self) -> None:
        try:
            x, y = float(self.reference_x.get()), float(self.reference_y.get())
        except ValueError:
            messagebox.showerror("Reference", "Enter measured X and Y in millimetres")
            return
        known_x = [point[0] for point in self.calibration.references_xy_mm.values()]
        known_y = [point[1] for point in self.calibration.references_xy_mm.values()]
        if not (min(known_x) <= x <= max(known_x)
                and min(known_y) <= y <= max(known_y)):
            messagebox.showerror("Reference", "Measured reference is outside the supplied plate bounds")
            return
        number = 1
        while f"M{number}" in self.calibration.references_xy_mm:
            number += 1
        name = f"M{number}"
        self.calibration.references_xy_mm[name] = (x, y)
        self.reference_box.configure(values=list(self.calibration.references_xy_mm))
        self.reference_name.set(name)
        self._update_reference_table()
        self.preview.set(f"{name}=({x:.1f},{y:.1f}) mm added. Click its measured image position.")

    def _update_reference_table(self) -> None:
        for row in self.reference_table.get_children():
            self.reference_table.delete(row)
        checks = self.calibration.corner_check_errors_mm()
        for name, (x, y) in self.calibration.references_xy_mm.items():
            uv = self.calibration.image_points_uv.get(name)
            pixel = "not clicked" if uv is None else f"({uv[0]:.0f}, {uv[1]:.0f})"
            check = "---" if name not in checks else f"{checks[name]:.1f} mm"
            self.reference_table.insert("", "end", iid=name,
                                        values=(name, f"{x:.1f}", f"{rear_to_axis_x(x):.1f}",
                                                f"{y:+.1f}", pixel, check))

    def _select_reference_row(self, _event=None) -> None:
        selected = self.reference_table.selection()
        if selected:
            self.reference_name.set(selected[0])

    def start_camera(self) -> None:
        if self.camera is not None:
            return
        try:
            index = int(self.camera_index.get())
        except ValueError:
            messagebox.showerror("Camera", "Enter an integer camera index")
            return
        camera = cv2.VideoCapture(index)
        if not camera.isOpened():
            camera.release()
            messagebox.showerror("Camera", f"Cannot open camera index {index}")
            return
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.calibration.image_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.calibration.image_height)
        self.camera = camera
        self._next_frame()

    def stop_camera(self) -> None:
        if self.scan_running:
            self.scan_stop_requested = True
            self.motion_status.set("CAMERA STOPPED - scan will stop before next step")
        if self.after_id is not None:
            self.after_cancel(self.after_id)
            self.after_id = None
        if self.camera is not None:
            self.camera.release()
            self.camera = None

    def _next_frame(self) -> None:
        if self.camera is None:
            return
        ok, frame = self.camera.read()
        if not ok:
            self.stop_camera()
            messagebox.showerror("Camera", "Frame read failed")
            return
        with self.frame_lock:
            self.raw_frame = frame.copy()
            self.frame_seq += 1
        try:
            if self.detect_enabled.get():
                active_calibration = self._effective_calibration(self.calibration, self.robot_state)
                view, self.results, _ = process_frame(frame, active_calibration)
            else:
                view, self.results = frame, []
            self._update_table()
            self._refresh_info()
            self.last_view = view.copy()
            self._render_frame(view)
        except (ValueError, cv2.error) as exc:
            self.stop_camera()
            messagebox.showerror("Vision", str(exc))
            return
        self.after_id = self.after(80, self._next_frame)

    def _click_reference(self, event) -> None:
        if self.scan_running:
            messagebox.showerror("XY calibration", "Finish the scan before recording reference points.")
            return
        if self.raw_frame is None:
            return
        if self.controller is None or not self.controller.status.connected:
            messagebox.showerror(
                "XY calibration",
                "Connect to OpenCR first. Calibration must be bound to current encoder angles.",
            )
            return
        u = int(event.x / self.display_scale)
        v = int(event.y / self.display_scale)
        if not (0 <= u < self.calibration.image_width and 0 <= v < self.calibration.image_height):
            return
        try:
            state = self.controller.read_robot_state()
        except ControllerError as exc:
            messagebox.showerror("XY calibration", f"Could not read current encoders: {exc}")
            return
        self.set_robot_state(state)
        if self.calibration.pose == "CENTER":
            actual = state.joints.as_tuple()
            expected = config.SCAN_JOINT_DEGREES
            if any(abs(a - b) > 2.0 for a, b in zip(actual, expected, strict=True)):
                messagebox.showerror(
                    "CENTER calibration",
                    "Move to the configured CENTER SCAN pose before recording its plate marks.",
                )
                return
        elif not self._pose_direction_matches(
                self.calibration.pose, state.joints.theta1,
                self.center_calibration.calibrated_joints_deg[0]
                if self.center_calibration.calibrated_joints_deg is not None
                else float(config.SCAN_JOINT_DEGREES[0]),
                self.left_mapping.get()):
            messagebox.showerror(
                "XY calibration",
                "The selected LEFT/RIGHT label does not match the measured J1 direction. "
                "Choose and verify the LEFT convention before recording this profile.",
            )
            return
        if (self.calibration.calibrated_joints_deg is not None
                and not self.calibration.matches_joints(state.joints)):
            messagebox.showerror(
                "XY calibration",
                "The arm moved since the first reference click. Select the correct profile "
                "or return to the original pose before adding this point.",
            )
            return
        name = self.reference_name.get()
        self.calibration.image_points_uv[name] = (float(u), float(v))
        self.calibration.save(joints_deg=self.robot_state.joints.as_tuple())
        if self.calibration.pose == "CENTER":
            self.center_calibration = self.calibration
        self._update_reference_table()
        self._refresh_info()
        count = len(self.calibration.image_points_uv)
        active = self._effective_calibration(self.calibration, self.robot_state)
        condition = "PROVISIONAL XY" if active.matrix() is not None else "XY UNCALIBRATED"
        self.preview.set(f"{name} pixel=({u},{v}) saved for {self.pose_name.get()} "
                         f"({count}/4). {condition}. Check point identity and fixed pose.")

    def _update_table(self) -> None:
        selected = self.table.selection()
        selected_uv = None
        if len(selected) == 1:
            values = self.table.item(selected[0], "values")
            if len(values) >= 5:
                selected_uv = (float(values[3]), float(values[4]))
        for row in self.table.get_children():
            self.table.delete(row)
        nearest_row = None
        nearest_distance = 20.0 ** 2
        for item in self.results:
            x = item["provisional_x_mm"]
            y = item["provisional_y_mm"]
            row_id = self.table.insert("", "end", values=(
                item["object_id"], item["status"],
                "PICK IT UP" if item["status"] == "VALID" else "—",
                item["center_u"], item["center_v"],
                "---" if x is None else x,
                "---" if x is None else round(rear_to_axis_x(x), 1),
                "---" if y is None else y,
                "---", item.get("xy_method", "NONE"),
                item["width_px"], item["height_px"], item["nearest_edge"] or "---"))
            if selected_uv is not None:
                distance = ((item["center_u"] - selected_uv[0]) ** 2 +
                            (item["center_v"] - selected_uv[1]) ** 2)
                if distance <= nearest_distance:
                    nearest_row, nearest_distance = row_id, distance
        if nearest_row is not None:
            self.table.selection_set(nearest_row)

    def _pick_from_result_row(self, event) -> None:
        if self.table.identify_column(event.x) != "#3":
            return
        row = self.table.identify_row(event.y)
        if not row or self.table.item(row, "values")[1] != "VALID":
            return
        self.table.selection_set(row)
        self.after_idle(self.pick_selected_callback)

    def capture_selected_cube(self) -> dict:
        """Freeze a fresh, pose-checked XY observation for a pick trial."""
        if self.scan_running or self.camera is None or not self.detect_enabled.get():
            raise ValueError("Stop scanning, start the camera, and enable DETECT first")
        selected = self.table.selection()
        if not selected and len(self.table.get_children()) == 1:
            self.table.selection_set(self.table.get_children()[0])
            selected = self.table.selection()
        if len(selected) != 1:
            raise ValueError("Select exactly one cube row in the camera table")
        old_row = self.table.item(selected[0], "values")
        old_u, old_v = float(old_row[3]), float(old_row[4])
        with self.frame_lock:
            if self.raw_frame is None:
                raise ValueError("No fresh camera frame is available")
            frame = self.raw_frame.copy()
        state = self.controller.read_robot_state()
        active = self._effective_calibration(self.calibration, state)
        if active.matrix() is None:
            raise ValueError("Camera X/Y calibration does not match the current arm pose")
        _view, current, _report = process_frame(frame, active)
        matches = [item for item in current
                   if (item["center_u"] - old_u) ** 2 + (item["center_v"] - old_v) ** 2 <= 20 ** 2]
        if len(matches) != 1:
            raise ValueError("Selected cube moved or changed identity; select it again")
        item = matches[0]
        if (item["status"] != "VALID" or item["provisional_x_mm"] is None or
                item["provisional_y_mm"] is None or item.get("xy_method") == "NONE"):
            raise ValueError("Cube is clipped or X/Y is uncalibrated; rescan before picking")
        return {
            "object_id": item["object_id"], "x_rear_mm": float(item["provisional_x_mm"]),
            "y_mm": float(item["provisional_y_mm"]), "pixel_u": item["center_u"],
            "pixel_v": item["center_v"], "xy_method": item["xy_method"],
            "scan_pose": self.pose_name.get(), "joints_deg": state.joints.as_tuple(),
            "captured_at": datetime.now(timezone.utc).isoformat(),
        }

    def _j1_sign(self, direction: str) -> float:
        mapping = self.left_mapping.get()
        if mapping == "UNVERIFIED - CHOOSE":
            raise ValueError("Choose and physically verify whether camera LEFT is J1 + or J1 -")
        left_sign = 1.0 if mapping == "LEFT = J1 +" else -1.0
        return left_sign if direction == "LEFT" else -left_sign

    def _require_motion_ready(self) -> None:
        if self.controller is None:
            raise ControllerError("Robot controller is unavailable")
        if not self.controller.status.connected:
            raise ControllerError("Connect to OpenCR first")
        if not self.controller.status.torque_on:
            raise ControllerError("Turn torque ON first")
        if not self.path_verified.get():
            raise ControllerError("Confirm that the scan path and both endpoints are physically clear")
        if self.scan_running:
            raise ControllerError("A scan movement is already running")

    @staticmethod
    def _target_from_j1(state, target_j1: float):
        q = state.joints
        predicted = JointAngles(target_j1, q.theta2, q.theta3, q.theta4)
        motors = fk_to_motor_angles(predicted)
        validate_motor_angles(motors)
        target = forward_kinematics_physical(motors)
        return predicted, motors, target

    @staticmethod
    def _assert_scan_posture(state) -> None:
        actual = state.joints.as_tuple()[1:]
        expected = config.SCAN_JOINT_DEGREES[1:]
        if any(abs(a - b) > 5.0 for a, b in zip(actual, expected, strict=True)):
            raise ControllerError(
                "J2-J4 are not near the configured camera SCAN posture. "
                "Move to CENTER SCAN and read encoders before scanning.")

    def move_camera(self, direction: str) -> None:
        try:
            self._require_motion_ready()
            step = float(self.scan_delta.get())
            if not 0.1 <= step <= 15.0:
                raise ValueError("Step must be between 0.1 and 15 degrees")
            if not self.scan_span.get().strip():
                raise ValueError("Enter the physically verified scan half-span before moving")
            span = float(self.scan_span.get())
            if not 1.0 <= span <= 45.0:
                raise ValueError("Enter a physically verified half-span of 1 to 45 degrees")
            sign = self._j1_sign(direction)
            state = self.controller.read_robot_state()
            self._assert_scan_posture(state)
            goal_j1 = state.joints.theta1 + sign * step
            if abs(goal_j1 - config.SCAN_JOINT_DEGREES[0]) > span + 1e-6:
                raise ControllerError("Requested step would exceed the verified scan half-span")
            predicted, motors, target = self._target_from_j1(
                state, goal_j1)
            if not messagebox.askyesno(
                f"Confirm camera {direction}",
                f"This WILL MOVE the robot by one discrete step.\n\n"
                f"Current J1: {state.joints.theta1:.2f} deg\n"
                f"Target J1: {predicted.theta1:.2f} deg\n"
                f"Predicted hand XYZ from ID11 axis: ({target.x:.1f}, {target.y:.1f}, {target.z:.1f}) mm\n\n"
                "Keep the emergency stop available and confirm the entire path is clear.",
            ):
                return
            self.scan_running = True
            self.scan_stop_requested = False
            self.motion_status.set(f"MOVING {direction} - discrete J1 step")
            threading.Thread(
                target=self._single_move_worker,
                args=(motors, direction),
                daemon=True,
            ).start()
        except (ControllerError, ValueError) as exc:
            messagebox.showerror("Camera scan movement", str(exc))

    def move_to_center_scan(self) -> None:
        if self.scan_running:
            messagebox.showerror("Camera scan movement", "A scan movement is already running")
            return
        self.move_center_callback()
        try:
            if self.controller is not None and self.controller.status.connected:
                self.set_robot_state(self.controller.read_robot_state())
        except ControllerError as exc:
            self.motion_status.set(f"CENTER moved, but state read failed: {exc}")

    def _single_move_worker(self, motors, direction: str) -> None:
        try:
            self.controller.set_motion_speed(config.CAMERA_PAYLOAD_MAXIMUM_SPEED_SCALE)
            self.controller.move_motor_angles(motors)
            state = self.controller.read_robot_state()
            self.after(0, self._motion_finished, state,
                       f"{direction} step complete; readback J1={state.joints.theta1:.2f} deg",
                       direction)
        except Exception as exc:
            self.after(0, self._motion_failed, exc)

    def _motion_finished(self, state, message: str, pose: str | None = None) -> None:
        self.scan_running = False
        if pose in ("CENTER", "LEFT", "RIGHT"):
            self.pose_name.set(pose)
            self.calibration = ScanCalibration.load(pose=pose)
        self.set_robot_state(state)
        self.motion_status.set(message)
        if self.log_callback is not None:
            self.log_callback(message)

    def _motion_failed(self, exc: Exception) -> None:
        self.scan_running = False
        self.motion_status.set(f"FAILED: {exc}")
        messagebox.showerror("Camera scan movement", str(exc))

    @staticmethod
    def _scan_sequence(center_q1: float, left_sign: float, span: float,
                       step: float, direction: str) -> list[tuple[str, float]]:
        if direction not in ("LEFT", "RIGHT", "BOTH"):
            raise ValueError("Scan direction must be LEFT, RIGHT, or BOTH")
        offsets = []
        distance = step
        while distance < span - 1e-9:
            offsets.append(distance)
            distance += step
        offsets.append(span)
        sequence = [("CENTER", center_q1)]
        for side in (("LEFT", "RIGHT") if direction == "BOTH" else (direction,)):
            sign = left_sign if side == "LEFT" else -left_sign
            for offset in offsets:
                sequence.append((side, center_q1 + sign * offset))
            for offset in reversed(offsets[:-1]):
                sequence.append((side, center_q1 + sign * offset))
            sequence.append(("CENTER", center_q1))
        return sequence

    def start_total_scan(self, direction: str = "BOTH") -> None:
        try:
            self._require_motion_ready()
            if self.camera is None:
                raise ControllerError("Start the camera before total scan")
            if not self.scan_span.get().strip():
                raise ValueError("Enter the physically verified scan half-span before moving")
            span = float(self.scan_span.get())
            step = float(self.scan_delta.get())
            settle = float(self.scan_settle.get())
            if not 1.0 <= span <= 45.0:
                raise ValueError("Total half-span must be between 1 and 45 degrees")
            if not 0.1 <= step <= 15.0:
                raise ValueError("Step must be between 0.1 and 15 degrees")
            if not 0.2 <= settle <= 5.0:
                raise ValueError("Settle time must be between 0.2 and 5 seconds")
            left_sign = self._j1_sign("LEFT")
            state = self.controller.read_robot_state()
            self._assert_scan_posture(state)
            center_q1 = float(config.SCAN_JOINT_DEGREES[0])
            if abs(state.joints.theta1 - center_q1) > 2.0:
                raise ControllerError(
                    f"Start total scan at CENTER: current J1={state.joints.theta1:.2f}, "
                    f"expected near {center_q1:.2f} deg")
            sequence = self._scan_sequence(center_q1, left_sign, span, step, direction)
            planned = []
            for pose, q1 in sequence:
                predicted, motors, xyz = self._target_from_j1(state, q1)
                planned.append((pose, predicted, motors, xyz))
            profiles = {}
            for pose in ("CENTER", "LEFT", "RIGHT"):
                stored = ScanCalibration.load(pose=pose)
                profiles[pose] = stored.matrix() is not None and any(
                    stored.matches_joints(item[1]) for item in planned if item[0] == pose)
            center_yaw_available = all(rotate_center_calibration(
                self.center_calibration, item[1], rear_to_internal_transform()) is not None
                for item in planned)
            if not messagebox.askyesno(
                "Confirm TOTAL camera scan",
                f"This WILL MOVE through {len(planned)} discrete positions in the {direction} area.\n"
                f"J1 step: {step:.1f}°; half-span: {span:.1f}°.\n"
                f"Settle: {settle:.1f}s; capture: 5 fresh frames per view.\n"
                f"Fixed-pose XY calibrations: {profiles}.\n"
                f"CENTER yaw XY model covers every step: {center_yaw_available}.\n"
                "Views without either model save pixels + joints + FK only.\n\n"
                "Keep the emergency stop available. Start?",
            ):
                return
            self.scan_running = True
            self.scan_stop_requested = False
            cycle = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.motion_status.set(f"TOTAL SCAN {cycle} starting")
            threading.Thread(
                target=self._total_scan_worker,
                args=(planned, settle, cycle),
                daemon=True,
            ).start()
        except (ControllerError, ValueError) as exc:
            messagebox.showerror("Total scan", str(exc))

    def stop_total_scan(self) -> None:
        self.scan_stop_requested = True
        self.motion_status.set("STOP REQUESTED - will stop before the next movement")

    def _total_scan_worker(self, planned, settle: float, cycle: str) -> None:
        log_path = ROOT / "data" / "vision_scan_logs" / f"total_scan_{cycle}.csv"
        image_dir = ROOT / "data" / "vision_scan_logs" / f"total_scan_{cycle}_images"
        image_dir.mkdir(parents=True, exist_ok=True)
        try:
            self.controller.set_motion_speed(config.CAMERA_PAYLOAD_MAXIMUM_SPEED_SCALE)
            for index, (pose, _predicted, motors, _xyz) in enumerate(planned):
                if self.scan_stop_requested:
                    break
                self.after(0, self.motion_status.set,
                           f"TOTAL SCAN: moving to {pose} ({index + 1}/{len(planned)})")
                self.controller.move_motor_angles(motors)
                state = self.controller.read_robot_state()
                self.after(0, self.set_robot_state, state)
                deadline = time.monotonic() + settle
                while time.monotonic() < deadline:
                    if self.scan_stop_requested:
                        break
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                if self.scan_stop_requested:
                    break
                calibration = ScanCalibration.load(pose=pose)
                active_calibration = self._effective_calibration(calibration, state)
                frame_results = []
                last_view = None
                last_frame = None
                with self.frame_lock:
                    last_seq = self.frame_seq
                for frame_index in range(1, 6):
                    if self.scan_stop_requested or self.camera is None:
                        self.scan_stop_requested = True
                        break
                    deadline = time.monotonic() + 2.0
                    frame = None
                    while time.monotonic() < deadline:
                        if self.scan_stop_requested or self.camera is None:
                            self.scan_stop_requested = True
                            break
                        with self.frame_lock:
                            if self.raw_frame is not None and self.frame_seq > last_seq:
                                frame = self.raw_frame.copy()
                                last_seq = self.frame_seq
                        if frame is not None:
                            break
                        time.sleep(0.02)
                    if self.scan_stop_requested:
                        break
                    if frame is None:
                        raise ControllerError("No new camera frame arrived within 2 seconds")
                    view, results, _details = process_frame(frame, active_calibration)
                    frame_results.append(results)
                    last_view, last_frame = view, frame
                    time.sleep(0.08)
                if self.scan_stop_requested:
                    break
                stable = self._majority_results(frame_results)
                self._append_csv(log_path, stable, state, pose, cycle, "majority_5")
                cv2.imwrite(str(image_dir / f"{index + 1}_{pose}_raw.png"), last_frame)
                cv2.imwrite(str(image_dir / f"{index + 1}_{pose}_annotated.png"), last_view)
                self.after(0, self._show_scan_capture, pose, calibration, stable, last_view, state)
            message = (f"TOTAL SCAN {cycle} stopped safely before next step"
                       if self.scan_stop_requested else
                       f"TOTAL SCAN {cycle} complete; results: {log_path}")
            self.after(0, self._total_scan_finished, message)
        except Exception as exc:
            self.after(0, self._motion_failed, exc)

    @staticmethod
    def _majority_results(frame_results: list[list[dict]]) -> list[dict]:
        clusters: list[list[dict]] = []
        for results in frame_results:
            used_clusters: set[int] = set()
            for item in results:
                nearest = None
                distance = float("inf")
                for index, cluster in enumerate(clusters):
                    if index in used_clusters:
                        continue
                    last = cluster[-1]
                    candidate_distance = ((item["center_u"] - last["center_u"]) ** 2
                                          + (item["center_v"] - last["center_v"]) ** 2) ** 0.5
                    if candidate_distance < distance:
                        nearest, distance = index, candidate_distance
                if nearest is not None and distance <= 25.0:
                    clusters[nearest].append(item)
                    used_clusters.add(nearest)
                else:
                    clusters.append([item])
                    used_clusters.add(len(clusters) - 1)
        stable = []
        required = len(frame_results) // 2 + 1
        for observations in clusters:
            if len(observations) >= required:
                edge_count = sum(item["status"] == "EDGE_RESCAN" for item in observations)
                status = "EDGE_RESCAN" if edge_count >= required else "VALID"
                preferred = [item for item in observations if item["status"] == status]
                median = sorted(preferred, key=lambda item: (item["center_u"], item["center_v"]))[
                    len(preferred) // 2].copy()
                median["status"] = status
                if status == "EDGE_RESCAN":
                    median["provisional_x_mm"] = None
                    median["provisional_y_mm"] = None
                stable.append(median)
        stable.sort(key=lambda item: (item["center_u"], item["center_v"]))
        for index, item in enumerate(stable, 1):
            item["object_id"] = f"cube_{index}"
        return stable

    def _show_scan_capture(self, pose: str, calibration, results, view, state) -> None:
        self.pose_name.set(pose)
        self.calibration = calibration
        self._update_reference_table()
        self.results = results
        self.set_robot_state(state)
        self._update_table()
        self.last_view = view.copy()
        self._render_frame(view)

    def _total_scan_finished(self, message: str) -> None:
        self.scan_running = False
        self.motion_status.set(message)
        if self.log_callback is not None:
            self.log_callback(message)

    def preview_scan(self) -> None:
        state = self.robot_state
        if state is None:
            self.preview.set("Read robot angles in the controller tab first; no assumed starting pose.")
            return
        try:
            step = float(self.scan_delta.get())
            if not 0.1 <= step <= 15.0:
                raise ValueError("Step must be between 0.1 and 15 degrees")
            before = state.physical_xyz
            if self.left_mapping.get() == "UNVERIFIED - CHOOSE":
                signs = [("J1 +", 1.0), ("J1 -", -1.0)]
            else:
                signs = [("LEFT", self._j1_sign("LEFT")), ("RIGHT", self._j1_sign("RIGHT"))]
            previews = []
            for label, sign in signs:
                predicted, _motors, target = self._target_from_j1(
                    state, state.joints.theta1 + sign * step)
                previews.append(
                    f"{label}: J1={predicted.theta1:.1f}°, "
                    f"axis XYZ=({target.x:.1f},{target.y:.1f},{target.z:.1f}), "
                    f"d=({target.x-before.x:+.1f},{target.y-before.y:+.1f},{target.z-before.z:+.1f}) mm")
            self.preview.set(
                f"NO MOTION | snapshot {state.timestamp_utc} | "
                + " | ".join(previews)
                + ". This is hand FK, not moving-camera cube XYZ.")
        except Exception as exc:
            self.preview.set(f"Preview unavailable: {exc}. NO MOTION.")

    def _append_csv(self, path: Path, results: list[dict], state, pose: str,
                    cycle: str, frame_index) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        new = not path.exists()
        joints = state.joints.as_tuple() if state else ("",) * 4
        xyz = (state.physical_xyz.x, state.physical_xyz.y, state.physical_xyz.z) if state else ("",) * 3
        with path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=LOG_FIELDS)
            if new:
                writer.writeheader()
            for item in results:
                writer.writerow(dict(zip(LOG_FIELDS, (
                    datetime.now(timezone.utc).isoformat(), pose, *joints, *xyz,
                    item["object_id"], item["status"], item["center_u"], item["center_v"],
                    item["width_px"], item["height_px"], item["contour_area"],
                    item["provisional_x_mm"], item["provisional_y_mm"], "",
                    item.get("xy_method", "NONE"), item["nearest_edge"],
                    cycle, frame_index,
                    config.PHYSICAL_FRAME["origin"], config.OPERATOR_REAR_FRAME["origin"],
                    ("" if item["provisional_x_mm"] is None else
                     round(rear_to_axis_x(item["provisional_x_mm"]), 1))), strict=True)))

    def save_snapshot(self) -> None:
        if self.raw_frame is None:
            messagebox.showerror("Snapshot", "Start camera first")
            return
        out = ROOT / "data" / "vision_scan_logs"
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"vision_snapshots_v3_{datetime.now().strftime('%Y%m%d')}.csv"
        state = self.robot_state
        self._append_csv(path, self.results, state, self.pose_name.get(), "manual", 1)
        self.preview.set(f"Saved {len(self.results)} detections to {path}.")
