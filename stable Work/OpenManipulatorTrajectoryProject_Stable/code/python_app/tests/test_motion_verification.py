import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

from kinematics import MotorAngles
from serial_controller import ControllerError, OpenCRController


class MotionVerificationTests(unittest.TestCase):
    def test_move_rejects_done_when_a_motor_is_still_far_from_its_target(self):
        """A false DONE must not be presented to the GUI as a successful move."""
        controller = OpenCRController()
        controller.status.torque_on = True
        target = MotorAngles(168.486, 179.824, 355.869, 82.881)
        actual = MotorAngles(168.486, 179.824, 355.869, 62.852)
        controller._command = lambda _message: "MOVING"
        controller._read_line = lambda _timeout: "DONE"
        controller.read_motor_angles = lambda: actual

        with self.assertRaisesRegex(ControllerError, r"ID14.*20\.029"):
            controller.move_motor_angles(target)


if __name__ == "__main__":
    unittest.main()
