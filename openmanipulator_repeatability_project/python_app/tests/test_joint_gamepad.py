import sys
import tkinter as tk
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gamepad_panel
from cartesian_jog import check_jog_arc, plan_xyz_jog_step
from experiment_frame import forward_kinematics_physical
from gamepad_panel import (GamepadPanel, advance_joint_target,
                           advance_xyz_target, normalized_xyz_input,
                           joint_jog_target, smooth_joint_speed, stick_command)
from kinematics import KinematicsError, MotorAngles, XYZ, normalize_angle


SCAN = MotorAngles(168.486328125, 159.829, 330.0735, 124.570333333)


class Device:
    def __init__(self):
        self.axes = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0]
        self.buttons = set()
    def get_init(self): return True
    def get_numaxes(self): return 6
    def get_axis(self, index): return self.axes[index]
    def get_numbuttons(self): return 11
    def get_button(self, index): return index in self.buttons
    def get_numhats(self): return 1
    def get_name(self): return "Controller (Wireless Gamepad F710)"
    def quit(self): pass


class JointGamepadTests(unittest.TestCase):
    def test_work_pose_xyz_axes_plan_without_moving_held_coordinates(self):
        import config
        from rear_reference_frame import axis_to_rear_xyz
        work = MotorAngles(*config.WORK_MOTOR_DEGREES)
        start = axis_to_rear_xyz(forward_kinematics_physical(work))
        for vector in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            target, motors, _ = plan_xyz_jog_step(
                work, work, start, (0.0, 0.0, 0.0), vector, 1.0, 0.05)
            self.assertNotEqual((target.x, target.y, target.z),
                                (start.x, start.y, start.z))
            for index, value in enumerate(vector):
                if value == 0.0:
                    self.assertEqual((target.x, target.y, target.z)[index],
                                     (start.x, start.y, start.z)[index])
            check_jog_arc(work, motors)

    def test_joint_rate_ramps_and_reverses_without_old_momentum(self):
        self.assertEqual(smooth_joint_speed(0.0, 1.0, 1.25), 3.0)
        self.assertEqual(smooth_joint_speed(3.0, 1.0, 1.25), 6.0)
        self.assertEqual(smooth_joint_speed(6.0, -1.0, 1.25), -3.0)

    def test_xyz_target_holds_untouched_axes_when_feedback_sags(self):
        commanded = XYZ(200.0, 15.0, 80.0)
        sagging = XYZ(199.0, 15.0, 77.5)
        target, velocity = advance_xyz_target(
            commanded, sagging, (0.0, 1.0, 0.0), (0.0, 0.0, 0.0), 1.0, 0.05)
        self.assertEqual(target.x, 200.0)
        self.assertGreater(target.y, 15.0)
        self.assertEqual(target.z, 80.0)
        target2, _ = advance_xyz_target(
            target, sagging, (0.0, 0.0, 1.0), velocity, 1.0, 0.05)
        self.assertEqual(target2.x, 200.0)
        self.assertEqual(target2.y, target.y)
        self.assertGreater(target2.z, 80.0)
        vector = normalized_xyz_input(1.0, 1.0, 1.0)
        self.assertAlmostEqual(sum(value * value for value in vector), 1.0)

    def test_integrated_target_advances_and_reverses_id11(self):
        target = SCAN
        for _ in range(10):
            target = advance_joint_target(SCAN, target, 0, 1.0, 1.0, 0.05)
        self.assertGreater(normalize_angle(target.id11 - SCAN.id11), 2.5)
        self.assertEqual(target.as_tuple()[1:], SCAN.as_tuple()[1:])
        reversed_target = target
        for _ in range(10):
            reversed_target = advance_joint_target(SCAN, reversed_target, 0, -1.0, 1.0, 0.05)
        self.assertLess(normalize_angle(reversed_target.id11 - target.id11), -2.5)

    def test_nonselected_joints_hold_initial_goals_despite_encoder_drift(self):
        measured = MotorAngles(SCAN.id11, SCAN.id12 + 0.8,
                               SCAN.id13, SCAN.id14)
        target = advance_joint_target(measured, SCAN, 0, 1.0, 0.25, 0.05)
        self.assertEqual(target.id12, SCAN.id12)
        self.assertGreater(target.id11, SCAN.id11)

    def test_scan_can_jog_one_motor_without_cartesian_ik(self):
        target = joint_jog_target(SCAN, 0, 1.0, 1.0)
        before, after = SCAN.as_tuple(), target.as_tuple()
        self.assertGreater(after[0], before[0])
        self.assertEqual(after[1:], before[1:])
        check_jog_arc(SCAN, target)
        self.assertEqual(stick_command(0.0), 0.0)
        self.assertGreater(stick_command(-1.0), 0.0)

    def test_direct_joint_limit_rejects_target(self):
        near_limit = MotorAngles(168.486328125, 159.829, 330.0735, 129.9)
        with self.assertRaises(KinematicsError):
            joint_jog_target(near_limit, 3, 1.0, 1.0)

    def test_f710_b_x_selection_and_right_stick(self):
        root = tk.Tk()
        root.withdraw()
        device = Device()
        controller = SimpleNamespace(status=SimpleNamespace(connected=True, torque_on=True),
            read_robot_state=lambda: SimpleNamespace(
                motors=SCAN, physical_xyz=forward_kinematics_physical(SCAN)),
            _command=lambda command: "CAPABILITIES,STATE,GAMEPAD_STREAM_V1")
        fake_pygame = SimpleNamespace(event=SimpleNamespace(get=lambda: [], pump=lambda: None),
                                      JOYDEVICEREMOVED=99)
        try:
            with patch.object(gamepad_panel, "pygame", fake_pygame):
                panel = GamepadPanel(root, controller=controller, busy_callback=lambda: False)
                panel.device = device
                panel._poll()
                device.buttons = {1}
                panel._poll()
                self.assertEqual(panel.selected_index, 1)
                self.assertIn("ID12", panel.selected_text.get())
                device.buttons.clear()
                panel._poll()
                device.buttons = {2}
                panel._poll()
                self.assertEqual(panel.selected_index, 0)
                device.buttons.clear()
                panel._poll()
                with patch.object(panel, "winfo_viewable", return_value=True), \
                     patch.object(panel, "winfo_toplevel", return_value=SimpleNamespace(
                         focus_displayof=lambda: root)), \
                     patch.object(panel, "command_gripper") as toggle:
                    device.buttons = {0}  # A / Cross: ID15 toggle.
                    panel._poll()
                    toggle.assert_called_once_with(None)
                    self.assertEqual(panel.selected_index, 0)
                device.buttons.clear()
                panel._poll()
                panel.arm()
                self.assertTrue(panel.armed)  # SCAN is allowed in direct joint mode.
                with patch.object(panel, "winfo_viewable", return_value=True), \
                     patch.object(panel, "winfo_toplevel", return_value=SimpleNamespace(
                         focus_displayof=lambda: root)), \
                     patch.object(gamepad_panel.threading, "Thread") as thread:
                    device.buttons = {1}  # B / Circle: next ID.
                    panel._poll()
                    self.assertEqual(panel.selected_index, 1)
                    self.assertEqual(panel.running, False)
                    device.buttons.clear()
                    panel._poll()
                    device.buttons = {2}  # X / Square: previous ID.
                    panel._poll()
                    self.assertEqual(panel.selected_index, 0)
                    self.assertIn("ID11", panel.selected_text.get())
                    device.buttons.clear()
                    device.axes[3] = -0.9
                    panel._poll()
                    thread.return_value.start.assert_called_once()
                    self.assertGreater(panel._sample[1], 0.0)
                    panel.running = False
                    device.axes[3] = 0.0
                    device.buttons = {7}
                    panel._poll()
                    self.assertFalse(panel.armed)
                panel.close()
        finally:
            root.destroy()

    def test_y_requests_work_and_xyz_right_and_left_sticks(self):
        root = tk.Tk()
        root.withdraw()
        device = Device()
        moved = []
        work = MotorAngles(*__import__("config").WORK_MOTOR_DEGREES)
        controller = SimpleNamespace(status=SimpleNamespace(connected=True, torque_on=True),
            read_robot_state=lambda: SimpleNamespace(
                motors=work, joints=__import__("kinematics").motor_to_fk_angles(work),
                physical_xyz=forward_kinematics_physical(work)),
            _command=lambda command: "CAPABILITIES,STATE,GAMEPAD_STREAM_V1")
        fake_pygame = SimpleNamespace(event=SimpleNamespace(get=lambda: [], pump=lambda: None),
                                      JOYDEVICEREMOVED=99)
        try:
            with patch.object(gamepad_panel, "pygame", fake_pygame):
                panel = GamepadPanel(root, controller=controller,
                    busy_callback=lambda: False,
                    move_work_callback=lambda: moved.append("WORK"))
                panel.device = device
                panel._poll()
                with patch.object(panel, "winfo_viewable", return_value=True), \
                     patch.object(panel, "winfo_toplevel", return_value=SimpleNamespace(
                         focus_displayof=lambda: root)):
                    device.buttons = {3}
                    panel._poll()
                    self.assertEqual(moved, ["WORK"])
                    device.buttons.clear()
                    panel._poll()
                    panel.mode.set("XYZ")
                    panel._mode_changed()
                    panel.arm()
                    self.assertTrue(panel.armed)
                    with patch.object(gamepad_panel.threading, "Thread") as worker:
                        device.axes[2] = -1.0  # Right horizontal to physical +Y.
                        device.axes[1] = -1.0  # Left vertical to physical +Z.
                        panel._poll()
                        worker.return_value.start.assert_called_once()
                        self.assertEqual(panel._sample[2][0], 0.0)
                        self.assertGreater(panel._sample[2][1], 0.0)
                        self.assertGreater(panel._sample[2][2], 0.0)
                panel.close()
        finally:
            root.destroy()

    def test_detect_controls_without_robot_motion(self):
        root = tk.Tk()
        root.withdraw()
        device = Device()
        fake_pygame = SimpleNamespace(event=SimpleNamespace(get=lambda: [], pump=lambda: None),
                                      JOYDEVICEREMOVED=99)
        try:
            with patch.object(gamepad_panel, "pygame", fake_pygame):
                panel = GamepadPanel(root, controller=SimpleNamespace(), busy_callback=lambda: False)
                panel.device = device
                panel.capture("y_axis")
                device.axes[0] = 0.9
                panel._poll()
                self.assertEqual(panel.y_axis.get(), "0")
                panel.capture("next")
                device.buttons = {4}
                panel._poll()
                self.assertEqual(panel.next_button.get(), "4")
                panel.close()
        finally:
            root.destroy()

    def test_editing_mapping_does_not_disconnect_controller(self):
        root = tk.Tk()
        root.withdraw()
        device = Device()
        fake_pygame = SimpleNamespace(event=SimpleNamespace(get=lambda: [], pump=lambda: None),
                                      JOYDEVICEREMOVED=99)
        try:
            with patch.object(gamepad_panel, "pygame", fake_pygame):
                panel = GamepadPanel(root, controller=SimpleNamespace(), busy_callback=lambda: False)
                panel.device = device
                panel.gripper_button.set("")
                panel._poll()
                self.assertIs(panel.device, device)
                self.assertIn("Correct controller mapping", panel.status.get())
                panel.close()
        finally:
            root.destroy()

    def test_gripper_buttons_use_exclusive_motion_when_arm_stopped(self):
        root = tk.Tk()
        root.withdraw()
        calls = []
        raw_values = iter((1800, 2650))
        controller = SimpleNamespace(
            status=SimpleNamespace(connected=True, torque_on=True),
            begin_exclusive_motion=lambda: calls.append("begin"),
            end_exclusive_motion=lambda: calls.append("end"),
            configure_gripper=lambda opened, closed: calls.append((opened, closed)),
            read_gripper_raw=lambda: next(raw_values),
            move_gripper=lambda target, cutoff: (
                calls.append((target, cutoff)) or
                SimpleNamespace(outcome="DONE", current_raw=10)))
        try:
            panel = GamepadPanel(root, controller=controller, busy_callback=lambda: False)
            with patch.object(gamepad_panel.threading, "Thread") as worker:
                panel.command_gripper(gamepad_panel.GRIPPER_CLOSE_RAW)
                worker.return_value.start.assert_called_once()
                self.assertTrue(panel.gripper_busy)
            panel._gripper_work(gamepad_panel.GRIPPER_CLOSE_RAW)
            self.assertEqual(calls, ["begin", (1800, 2650), (2650, 200), "end"])
            self.assertFalse(panel.gripper_busy)
            panel.running = True
            panel.command_gripper(gamepad_panel.GRIPPER_OPEN_RAW)
            self.assertEqual(len(calls), 4)
            panel.close()
        finally:
            root.destroy()

    def test_x_toggle_chooses_open_and_close_from_measured_raw(self):
        root = tk.Tk()
        root.withdraw()
        calls = []
        raw_values = iter((1800, 2650, 2650, 1800))
        controller = SimpleNamespace(
            status=SimpleNamespace(connected=True, torque_on=True),
            begin_exclusive_motion=lambda: calls.append("begin"),
            end_exclusive_motion=lambda: calls.append("end"),
            configure_gripper=lambda opened, closed: None,
            read_gripper_raw=lambda: next(raw_values),
            move_gripper=lambda target, cutoff: (
                calls.append((target, cutoff)) or
                SimpleNamespace(outcome="DONE", current_raw=10)))
        try:
            panel = GamepadPanel(root, controller=controller, busy_callback=lambda: False)
            panel._gripper_work(None)
            panel._gripper_work(None)
            self.assertEqual([item for item in calls if isinstance(item, tuple)],
                             [(2650, 200), (1800, 200)])
            panel.close()
        finally:
            root.destroy()

    def test_stream_commands_only_selected_id_then_stops(self):
        root = tk.Tk()
        root.withdraw()
        calls = []
        controller = SimpleNamespace(
            begin_exclusive_motion=lambda: calls.append("begin"),
            end_exclusive_motion=lambda: calls.append("end"),
            start_jog_stream=lambda: SCAN,
        )
        try:
            panel = GamepadPanel(root, controller=controller, busy_callback=lambda: False)
            panel.selected_index = 2
            panel.armed = True
            panel._sample = (gamepad_panel.time.monotonic(), 1.0, (1.0, 0.0, 0.0))
            def update(target):
                calls.append(target)
                panel._sample = (gamepad_panel.time.monotonic(), 0.0, (0.0, 0.0, 0.0))
                return target
            controller.update_jog_stream = update
            controller.stop_jog_stream = lambda: calls.append("stop")
            panel._work()
            targets = [item for item in calls if isinstance(item, MotorAngles)]
            self.assertEqual(len(targets), 1)
            self.assertEqual(targets[0].as_tuple()[:2], SCAN.as_tuple()[:2])
            self.assertNotEqual(targets[0].as_tuple()[2], SCAN.as_tuple()[2])
            self.assertEqual(targets[0].as_tuple()[3], SCAN.as_tuple()[3])
            self.assertEqual(calls[-2:], ["stop", "end"])
            panel.close()
        finally:
            root.destroy()
