import sys
import unittest
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from keyboard_panel import KeyboardPanel, keyboard_vector
from kinematics import JointAngles


class KeyboardControlTests(unittest.TestCase):
    def test_direction_aliases_opposites_and_diagonal_speed(self):
        self.assertEqual(keyboard_vector({"w", "up"}), (1, 0, 0))
        self.assertEqual(keyboard_vector({"w", "s"}), (0, 0, 0))
        self.assertEqual(keyboard_vector({"a"}), (0, 1, 0))
        self.assertEqual(keyboard_vector({"d"}), (0, -1, 0))
        self.assertEqual(keyboard_vector({"r"}), (0, 0, 1))
        self.assertEqual(keyboard_vector({"f"}), (0, 0, -1))
        self.assertAlmostEqual(sum(v*v for v in keyboard_vector({"w", "a", "r"})), 1)

    def test_no_gamepad_required_release_and_focus_loss_stop(self):
        root = tk.Tk()
        root.withdraw()
        controller = SimpleNamespace(status=SimpleNamespace(connected=True, torque_on=True),
            read_robot_state=lambda: SimpleNamespace(joints=JointAngles(0, 0, 0, 82.88)),
            _command=lambda command: "CAPABILITIES,GAMEPAD_STREAM_V1")
        try:
            panel = KeyboardPanel(root, controller=controller, busy_callback=lambda: False)
            panel.arm()
            self.assertTrue(panel.armed)
            panel._key_press(SimpleNamespace(keysym="w"))
            self.assertEqual(panel._keys, {"w"})
            with patch.object(panel.jog_pad, "winfo_viewable", return_value=True), \
                 patch.object(panel, "focus_get", return_value=panel.jog_pad), \
                 patch("keyboard_panel.threading.Thread") as worker:
                panel._poll()
                self.assertEqual(panel._sample[1:3], ((1.0, 0.0, 0.0), True))
                worker.return_value.start.assert_called_once()
                panel.running = False
            panel._key_release(SimpleNamespace(keysym="w"))
            self.assertTrue(panel._cancel.is_set())
            self.assertFalse(panel._keys)
            panel._key_press(SimpleNamespace(keysym="r"))
            panel._focus_lost()
            self.assertFalse(panel.armed)
            self.assertFalse(panel._keys)
            panel._key_press(SimpleNamespace(keysym="w"))
            self.assertFalse(panel._keys)
            panel.close()
        finally:
            root.destroy()

    def test_stale_firmware_blocks_keyboard_before_motion(self):
        root = tk.Tk()
        root.withdraw()
        controller = SimpleNamespace(status=SimpleNamespace(connected=True, torque_on=True),
            read_robot_state=lambda: SimpleNamespace(joints=JointAngles(0, 0, 0, 82.88)),
            _command=lambda command: "CAPABILITIES,GAMEPAD_JOG_V1")
        try:
            panel = KeyboardPanel(root, controller=controller, busy_callback=lambda: False)
            panel.arm()
            self.assertFalse(panel.armed)
            self.assertIn("Upload", panel.status.get())
            panel.close()
        finally:
            root.destroy()
