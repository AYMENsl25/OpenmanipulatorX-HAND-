import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1]))

import config
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

    def test_angle_read_retries_transient_bus_error(self):
        controller = OpenCRController()
        expected = MotorAngles(10.0, 20.0, 30.0, 40.0)
        attempts = iter((ControllerError("ERROR,BUS_READ"), expected))

        def read_once():
            result = next(attempts)
            if isinstance(result, Exception):
                raise result
            return result

        controller._read_motor_angles_once = read_once
        self.assertEqual(controller.read_motor_angles(), expected)

    def test_robot_state_uses_one_coherent_encoder_sample(self):
        controller = OpenCRController()
        motors = MotorAngles(168.486328125, 179.82421875, 355.869140625, 82.880859375)
        controller._command = lambda _message: (
            "STATE,1917,2046,4049,943,"
            "168.486328125,179.82421875,355.869140625,82.880859375,"
            "0.0,0.0,0.0,82.880859375"
        )
        state = controller.read_robot_state()
        self.assertEqual(state.motors, motors)
        self.assertEqual(state.raw_positions, (1917, 2046, 4049, 943))
        self.assertAlmostEqual(state.physical_xyz.x, config.WORK_XYZ_MM[0])
        self.assertAlmostEqual(state.physical_xyz.z, config.WORK_XYZ_MM[2])

    def test_firmware_joint_limits_match_python_before_motion(self):
        controller = OpenCRController()
        controller._command = lambda message: (
            "JOINT_LIMITS,-110,110,-25,85,-60,90,-45,130,RAW_TOLERANCE,0.045"
            if message == "JOINT_LIMITS"
            else ""
        )
        controller.verify_firmware_joint_limits()

    def test_firmware_joint_limit_mismatch_is_rejected(self):
        controller = OpenCRController()
        controller._command = lambda _message: (
            "JOINT_LIMITS,-90,100,-25,85,-60,90,-45,130,RAW_TOLERANCE,0.045"
        )
        with self.assertRaisesRegex(ControllerError, r"q1: OpenCR=-90\.0\.\.100\.0"):
            controller.verify_firmware_joint_limits()

    def test_outdated_firmware_is_reported_clearly(self):
        controller = OpenCRController()

        def unknown(_message):
            raise ControllerError("ERROR,UNKNOWN_COMMAND")

        controller._command = unknown
        with self.assertRaisesRegex(ControllerError, "firmware is outdated"):
            controller.read_firmware_joint_limits()

    def test_older_limit_protocol_without_raw_tolerance_is_rejected(self):
        controller = OpenCRController()
        controller._command = lambda _message: (
            "JOINT_LIMITS,-110,110,-25,85,-60,90,-45,130"
        )
        with self.assertRaisesRegex(ControllerError, "protocol is outdated"):
            controller.read_firmware_joint_limits()


if __name__ == "__main__":
    unittest.main()
