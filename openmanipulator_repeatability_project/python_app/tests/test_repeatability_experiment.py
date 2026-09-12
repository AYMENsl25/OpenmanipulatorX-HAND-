from __future__ import annotations

from pathlib import Path
import sys
import unittest

from openpyxl import load_workbook

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinematics import JointAngles, MotorAngles, XYZ
from repeatability_experiment import (
    ManualMeasurement,
    RepeatabilityRunConfig,
    RepeatabilityWorkbook,
    TouchRecord,
)
from serial_controller import MotorTelemetry, OpenCRController, RobotState, TelemetrySnapshot


def telemetry_snapshot(offset: int = 0) -> TelemetrySnapshot:
    motors = MotorAngles(168.486, 179.824, 355.869, 82.881)
    joints = JointAngles(0.0, 0.0, 0.0, 82.881)
    physical = XYZ(230.0 + offset * 0.1, 70.0, offset * 0.05)
    state = RobotState(
        timestamp_utc="2026-09-10T00:00:00.000Z",
        raw_positions=(1917 + offset, 2046, 4049, 943),
        motors=motors,
        joints=joints,
        physical_xyz=physical,
        internal_xyz=XYZ(physical.x, -physical.y, physical.z),
    )
    diagnostics = tuple(
        MotorTelemetry(
            motor_id=motor_id,
            position_raw=raw,
            velocity_raw=offset,
            current_raw=10 + offset,
            pwm_raw=20 + offset,
            voltage_raw=120,
            temperature_c=30 + offset,
            hardware_error=0,
            moving=0,
            moving_status=0,
        )
        for motor_id, raw in zip((11, 12, 13, 14), state.raw_positions, strict=True)
    )
    return TelemetrySnapshot(state.timestamp_utc, state, diagnostics)


class RepeatabilityExperimentTests(unittest.TestCase):
    def test_telemetry_protocol_parser(self) -> None:
        groups = []
        for motor_id, raw in zip((11, 12, 13, 14), (1917, 2046, 4049, 943), strict=True):
            groups.extend((motor_id, raw, -2, 10, 20, 120, 31, 0, 0, 0))
        response = "TELEMETRY_V1," + ",".join(str(value) for value in groups)
        controller = OpenCRController()
        controller._command = lambda command: response  # type: ignore[method-assign]

        snapshot = controller.read_telemetry()

        self.assertEqual(tuple(m.motor_id for m in snapshot.motors), (11, 12, 13, 14))
        self.assertEqual(snapshot.state.raw_positions, (1917, 2046, 4049, 943))
        self.assertAlmostEqual(snapshot.motors[0].velocity_rpm, -0.458)
        self.assertAlmostEqual(snapshot.motors[0].current_ma, 26.9)
        self.assertAlmostEqual(snapshot.motors[0].voltage_v, 12.0)

    def test_workbook_contains_summary_samples_config_and_events(self) -> None:
        output_path = Path(__file__).resolve().parent / "repeatability_test_output.xlsx"
        try:
            run = RepeatabilityRunConfig(
                session_id="test-repeatability",
                selected_points=("P09",),
                repetitions=2,
                sample_count=2,
                sample_interval_ms=50,
                pause_for_manual_measurement=True,
            )
            writer = RepeatabilityWorkbook(
                Path(__file__).resolve().parents[2], run, output_path=output_path
            )
            record = TouchRecord(
                cycle=1,
                point_name="P09",
                touch_index=1,
                commanded_physical=XYZ(230.0, 70.0, 0.0),
                commanded_internal=XYZ(230.0, -70.0, 0.0),
                planned_joints=JointAngles(0.0, 0.0, 0.0, 0.0),
                planned_motors=MotorAngles(168.486, 179.824, 355.869, 82.881),
                samples=[telemetry_snapshot(0), telemetry_snapshot(1)],
            )
            writer.add_touch(record)
            writer.update_manual(
                record,
                ManualMeasurement(measured_xyz=XYZ(248.0, 70.0, 0.0), note="laser"),
            )

            self.assertTrue(writer.path.exists())
            workbook = load_workbook(writer.path, data_only=False, read_only=True)
            try:
                self.assertEqual(
                    workbook.sheetnames,
                    ["Touch Summary", "Telemetry Samples", "Run Config", "Run Events"],
                )
                summary = workbook["Touch Summary"]
                headers = [cell.value for cell in summary[1]]
                row = [cell.value for cell in summary[2]]
                self.assertEqual(row[headers.index("point")], "P09")
                self.assertAlmostEqual(row[headers.index("manual_error_x_mm")], 18.0)
                self.assertEqual(workbook["Telemetry Samples"].max_row, 3)
                self.assertGreater(workbook["Run Events"].max_row, 1)
            finally:
                workbook.close()
        finally:
            output_path.unlink(missing_ok=True)
            output_path.with_name("repeatability_test_output.saving.xlsx").unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
