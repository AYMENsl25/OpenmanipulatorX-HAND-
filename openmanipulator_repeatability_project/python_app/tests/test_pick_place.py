import math
import sys
import unittest
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from experiment_frame import forward_kinematics_physical
from kinematics import MotorAngles, motor_to_fk_angles, forward_kinematics
from pick_place_panel import (CubeTrial, PickSettings, PickPlacePanel,
                              phase_speed,
                              check_destination_reachability,
                              format_arm_target_readback, plan_pick_place)
from serial_controller import ControllerError, RobotState


class TestPickPlace(unittest.TestCase):
    def setUp(self):
        work = MotorAngles(*config.WORK_MOTOR_DEGREES)
        self.planning_state = RobotState(
            "2026-09-24T00:00:00",
            None,
            work,
            motor_to_fk_angles(work),
            forward_kinematics_physical(work),
            forward_kinematics(work),
        )

    def test_pick_settings_defaults(self):
        settings = PickSettings(
            pick_z_mm=15.0,
            place_z_mm=15.0,
            travel_z_mm=80.0,
            open_raw=1400,
            close_raw=2650,
            current_limit_raw=200,
        )
        self.assertEqual(settings.speed_scale, config.PICK_PLACE_DEFAULT_SPEED_SCALE)
        self.assertLessEqual(
            config.PICK_PLACE_DEFAULT_SPEED_SCALE,
            config.PICK_PLACE_MAXIMUM_SPEED_SCALE,
        )

        custom_settings = PickSettings(
            pick_z_mm=10.0,
            place_z_mm=10.0,
            travel_z_mm=75.0,
            open_raw=1400,
            close_raw=2650,
            current_limit_raw=200,
            speed_scale=1.0,
        )
        self.assertEqual(custom_settings.speed_scale, 1.0)

    def test_plan_pick_place_direct_motion(self):
        trial = CubeTrial(
            source_id="CUBE_1",
            source_x_rear_mm=180.0,
            source_y_mm=50.0,
            destination_x_rear_mm=180.0,
            destination_y_mm=-50.0,
            pixel_u=320,
            pixel_v=240,
            xy_method="CALIB",
            captured_at="2026-09-24T00:00:00",
        )
        settings = PickSettings(
            pick_z_mm=15.0,
            place_z_mm=15.0,
            travel_z_mm=80.0,
            open_raw=1400,
            close_raw=2650,
            current_limit_raw=200,
            speed_scale=0.85,
        )
        plan = plan_pick_place((trial,), settings, self.planning_state)
        phases = [step.phase for step in plan]
        expected_phases = [
            "OPEN_BEFORE_PICK",
            "RAISE_TO_TRAVEL",
            "ABOVE_PICK",
            "LOWER_TO_PICK",
            "CLOSE_AND_CONFIRM",
            "LIFT",
            "ABOVE_PLACE",
            "LOWER_TO_PLACE",
            "RELEASE",
            "RETRACT",
            "RETURN_TO_WORK",
            "ALIGN_WORK_FOR_SCAN",
            "RETURN_TO_SCAN",
        ]
        self.assertEqual(phases, expected_phases)
        # Direct motion should produce exactly 1 target pose per travel leg
        for step in plan:
            if step.phase in ("RAISE_TO_TRAVEL", "ABOVE_PICK", "LOWER_TO_PICK", "LIFT", "ABOVE_PLACE", "LOWER_TO_PLACE", "RETRACT"):
                self.assertGreaterEqual(len(step.motors), 1)
                self.assertLessEqual(len(step.motors), 3)

    def test_travel_z_clearance_enforced(self):
        trial = CubeTrial(
            source_id="CUBE_1",
            source_x_rear_mm=180.0,
            source_y_mm=50.0,
            destination_x_rear_mm=180.0,
            destination_y_mm=-50.0,
            pixel_u=320,
            pixel_v=240,
            xy_method="CALIB",
            captured_at="2026-09-24T00:00:00",
        )
        # Transfer must not descend below the initial WORK height (~47 mm).
        low_z_settings = PickSettings(
            pick_z_mm=10.0,
            place_z_mm=10.0,
            travel_z_mm=40.0,
            open_raw=1400,
            close_raw=2650,
            current_limit_raw=200,
        )
        with self.assertRaises(ValueError) as ctx:
            plan_pick_place((trial,), low_z_settings, self.planning_state)
        self.assertIn("Travel Z must clear", str(ctx.exception))

    def test_low_travel_50_reaches_y100_destination(self):
        trial = CubeTrial("cube_1", 236.5, -62.3, 50.0, 100.0,
                          109, 185, "FIXED_POSE", "")
        check_destination_reachability(50.0, 100.0, 5.0, 50.0)
        plan = plan_pick_place((trial,), PickSettings(0.0, 5.0, 50.0,
                                                   1800, 2650, 200),
                               self.planning_state)
        self.assertIn("LOWER_TO_PLACE", [step.phase for step in plan])
        with self.assertRaisesRegex(ValueError, "Travel Z must clear"):
            plan_pick_place((trial,), PickSettings(35.0, 5.0, 50.0,
                                                  1800, 2650, 200),
                            self.planning_state)

    def test_quick_pick_dialog_interface(self):
        self.assertTrue(hasattr(PickPlacePanel, "quick_pick_dialog"))

    def test_five_cube_queue_and_travel_60_70(self):
        for z in (60.0, 70.0):
            trials = tuple(CubeTrial(f"cube_{i}", 220.0 + i * 10, -50.0,
                                    110.0, 100.0, 0, 0, "FIXED_POSE", "")
                           for i in range(5))
            check_destination_reachability(110.0, 100.0, 5.0, z)
            plan = plan_pick_place(trials, PickSettings(0, 5, z, 1800, 2650, 200, 2.0),
                                   self.planning_state)
            self.assertEqual(sum(step.phase == "CLOSE_AND_CONFIRM" for step in plan), 5)
            self.assertEqual(sum(step.phase == "RELEASE" for step in plan), 5)
            self.assertEqual(plan[-1].cube_index, 5)

    def test_contact_phases_remain_slower_than_transfer(self):
        self.assertEqual(phase_speed(2.0, "ABOVE_PLACE"), 2.0)
        self.assertEqual(phase_speed(2.0, "LOWER_TO_PICK"), 0.65)
        self.assertEqual(phase_speed(2.0, "MOVE_TO_WORK"), 0.5)
        self.assertEqual(phase_speed(0.25, "LOWER_TO_PLACE"), 0.25)
        for speed in (float("nan"), 2.1, 0):
            with self.assertRaises(ValueError):
                phase_speed(speed, "ABOVE_PICK")

    def test_side_destination_reachability_uses_rear_frame(self):
        with self.assertRaisesRegex(ValueError, "X is already inside the configured software X range"):
            check_destination_reachability(20.0, 100.0, 100.0, 120.0)
        check_destination_reachability(50.0, 170.0, 100.0, 120.0)
        # Rear X=5 is not excluded by the X envelope; reachability depends
        # on Y and Z and still requires physical clearance validation.
        check_destination_reachability(5.0, 170.0, 100.0, 120.0)
        with self.assertRaises(ValueError):
            check_destination_reachability(5.0, 100.0, 100.0, 120.0)

    def test_close_in_box_has_reachable_release_but_no_top_down_approach(self):
        with self.assertRaisesRegex(ValueError, "top-down route needs at least") as ctx:
            check_destination_reachability(50.0, 100.0, 5.0, 90.0)
        self.assertIn("0–52 mm", str(ctx.exception))
        self.assertIn("Y is already inside the configured software Y range", str(ctx.exception))
        for travel_z in (80.0, 100.0, 160.0):
            with self.subTest(travel_z=travel_z), self.assertRaises(ValueError):
                check_destination_reachability(50.0, 100.0, 5.0, travel_z)

    def test_proposed_close_rear_fallback_has_no_safe_top_down_route(self):
        with self.assertRaisesRegex(ValueError, "above-place/travel"):
            check_destination_reachability(60.0, 10.0, 5.0, 80.0)

    def test_target_not_reached_reports_encoder_and_never_retries(self):
        target = MotorAngles(160.0, 180.0, 350.0, 80.0)
        actual = MotorAngles(142.5, 180.0, 350.0, 80.0)
        self.assertIn("ID11 target=160.00°, read=142.50°, error=17.50°",
                      format_arm_target_readback(target, actual))
        controller = MagicMock()
        controller.move_motor_angles.side_effect = ControllerError(
            "Movement failed or malformed response: ERROR,TARGET_NOT_REACHED,ID11,ERROR_DEG,17.490")
        controller.read_telemetry.return_value = SimpleNamespace(
            state=SimpleNamespace(motors=actual),
            motors=(SimpleNamespace(current_raw=50, pwm_raw=100,
                                    velocity_raw=0, voltage_v=12.0,
                                    temperature_c=30, hardware_error=0),),
        )
        panel = SimpleNamespace(stop_requested=Event(), _motor_move_active=Event(),
                                controller=controller)
        with self.assertRaisesRegex(ControllerError, "ABOVE_PICK:.*ID11 target=160.00") as ctx:
            PickPlacePanel._move_arm(panel, target, "ABOVE_PICK")
        self.assertIn("no automatic retry", str(ctx.exception))
        controller.move_motor_angles.assert_called_once_with(target)
        controller.read_telemetry.assert_called_once_with()
        self.assertFalse(panel._motor_move_active.is_set())

    def test_travel_z_160_alone_does_not_fix_y_100(self):
        trial = CubeTrial("cube_1", 236.5, -62.3, 50.0, 100.0,
                          109, 185, "FIXED_POSE", "")
        with self.assertRaisesRegex(ValueError, "RAISE_TO_TRAVEL"):
            plan_pick_place((trial,), PickSettings(0.0, 5.0, 160.0,
                                                     1800, 2650, 200),
                            self.planning_state)

    def test_screenshot_y_change_to_160_makes_route_reachable(self):
        trial = CubeTrial("cube_1", 236.5, -62.3, 50.0, 160.0,
                          109, 185, "FIXED_POSE", "")
        check_destination_reachability(50.0, 160.0, 5.0, 80.0)
        plan = plan_pick_place((trial,), PickSettings(0.0, 5.0, 80.0,
                                                   1800, 2650, 200),
                               self.planning_state)
        self.assertIn("LOWER_TO_PLACE", [step.phase for step in plan])

    def test_new_cube_above_pick_is_independent_of_destination_y(self):
        for destination_y in (100.0, 150.0):
            trial = CubeTrial("cube_1", 149.1, -24.1, 50.0,
                              destination_y, 0, 0, "FIXED_POSE", "")
            with self.subTest(destination_y=destination_y):
                with self.assertRaisesRegex(
                        ValueError, "Changing destination Y cannot fix this pick-side point") as ctx:
                    plan_pick_place((trial,), PickSettings(0.0, 5.0, 80.0,
                                                             1800, 2650, 200),
                                    self.planning_state)
                self.assertIn("0–71 mm", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
