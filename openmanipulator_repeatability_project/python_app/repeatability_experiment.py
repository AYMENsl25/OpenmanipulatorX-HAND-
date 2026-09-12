"""Repeatability experiment records and recoverable Excel export.

The experiment compares the commanded physical point with FK reconstructed from
fresh encoder readings.  Optional manual measurements are kept separately so
encoder/FK error is never confused with external metrology error.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
import os
from pathlib import Path
from statistics import fmean, pstdev
from typing import Iterable

from openpyxl import Workbook
from openpyxl.styles import Alignment, Font, PatternFill

import config
from kinematics import JointAngles, MotorAngles, XYZ
from serial_controller import TelemetrySnapshot


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def _mean(values: Iterable[float]) -> float:
    materialized = tuple(float(value) for value in values)
    return fmean(materialized)


def _std(values: Iterable[float]) -> float:
    materialized = tuple(float(value) for value in values)
    return pstdev(materialized) if len(materialized) > 1 else 0.0


def _vector_error(actual: XYZ, target: XYZ) -> tuple[float, float, float, float]:
    dx = actual.x - target.x
    dy = actual.y - target.y
    dz = actual.z - target.z
    return dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz)


@dataclass(frozen=True)
class RepeatabilityRunConfig:
    session_id: str
    selected_points: tuple[str, ...]
    repetitions: int
    sample_count: int
    sample_interval_ms: int
    pause_for_manual_measurement: bool
    started_utc: str = field(default_factory=utc_timestamp)


@dataclass(frozen=True)
class ManualMeasurement:
    measured_xyz: XYZ | None = None
    reported_distance_error_mm: float | None = None
    note: str = ""
    timestamp_utc: str = field(default_factory=utc_timestamp)


@dataclass
class TouchRecord:
    cycle: int
    point_name: str
    touch_index: int
    commanded_physical: XYZ
    commanded_internal: XYZ
    planned_joints: JointAngles
    planned_motors: MotorAngles
    samples: list[TelemetrySnapshot]
    recorded_utc: str = field(default_factory=utc_timestamp)
    manual: ManualMeasurement | None = None
    status: str = "AUTO_SAMPLED"

    def mean_physical_fk(self) -> XYZ:
        return XYZ(
            _mean(sample.state.physical_xyz.x for sample in self.samples),
            _mean(sample.state.physical_xyz.y for sample in self.samples),
            _mean(sample.state.physical_xyz.z for sample in self.samples),
        )

    def mean_internal_fk(self) -> XYZ:
        return XYZ(
            _mean(sample.state.internal_xyz.x for sample in self.samples),
            _mean(sample.state.internal_xyz.y for sample in self.samples),
            _mean(sample.state.internal_xyz.z for sample in self.samples),
        )

    def fk_std(self) -> XYZ:
        return XYZ(
            _std(sample.state.physical_xyz.x for sample in self.samples),
            _std(sample.state.physical_xyz.y for sample in self.samples),
            _std(sample.state.physical_xyz.z for sample in self.samples),
        )

    def fk_error(self) -> tuple[float, float, float, float]:
        return _vector_error(self.mean_physical_fk(), self.commanded_physical)

    def manual_error(self) -> tuple[float, float, float, float] | None:
        if self.manual is None or self.manual.measured_xyz is None:
            return None
        return _vector_error(self.manual.measured_xyz, self.commanded_physical)


class RepeatabilityWorkbook:
    """Accumulate one run and rewrite a recoverable workbook after each touch."""

    def __init__(
        self,
        project_root: str | Path,
        run: RepeatabilityRunConfig,
        *,
        output_path: str | Path | None = None,
    ) -> None:
        self.run = run
        self.records: list[TouchRecord] = []
        self.events: list[tuple[str, str, str]] = []
        if output_path is None:
            self.run_dir = Path(project_root).resolve() / "experiment_results" / run.session_id
            self.run_dir.mkdir(parents=True, exist_ok=True)
            self.path = self.run_dir / "repeatability_results.xlsx"
        else:
            self.path = Path(output_path).resolve()
            self.run_dir = self.path.parent
            if not self.run_dir.is_dir():
                raise ValueError("output_path parent directory must already exist")

    def add_event(self, event: str, details: str = "") -> None:
        self.events.append((utc_timestamp(), event, details))

    def add_touch(self, record: TouchRecord) -> None:
        if not record.samples:
            raise ValueError("A touch record requires at least one telemetry sample")
        self.records.append(record)
        self.add_event(
            "touch_sampled",
            f"cycle={record.cycle}; point={record.point_name}; samples={len(record.samples)}",
        )
        self.save()

    def update_manual(self, record: TouchRecord, manual: ManualMeasurement | None) -> None:
        if record not in self.records:
            raise ValueError("Touch record does not belong to this run")
        record.manual = manual
        record.status = "MANUAL_RECORDED" if manual is not None else "MANUAL_SKIPPED"
        self.add_event(record.status.lower(), f"cycle={record.cycle}; point={record.point_name}")
        self.save()

    @staticmethod
    def _style_sheet(sheet) -> None:
        header_fill = PatternFill("solid", fgColor="1F4E78")
        for cell in sheet[1]:
            cell.font = Font(color="FFFFFF", bold=True)
            cell.fill = header_fill
            cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        sheet.freeze_panes = "A2"
        sheet.auto_filter.ref = sheet.dimensions
        for column in sheet.columns:
            values = [str(cell.value) if cell.value is not None else "" for cell in column[:100]]
            width = min(24, max(9, max((len(value) for value in values), default=0) + 2))
            sheet.column_dimensions[column[0].column_letter].width = width

    @staticmethod
    def _append_xyz(row: list, value: XYZ | None) -> None:
        row.extend((value.x, value.y, value.z) if value is not None else (None, None, None))

    def _summary_headers(self) -> list[str]:
        headers = [
            "session_id", "cycle", "point", "touch_index", "recorded_utc", "status",
            "command_x_mm", "command_y_mm", "command_z_mm",
            "internal_command_x_mm", "internal_command_y_mm", "internal_command_z_mm",
            "planned_q1_deg", "planned_q2_deg", "planned_q3_deg", "planned_q4_deg",
            "planned_id11_deg", "planned_id12_deg", "planned_id13_deg", "planned_id14_deg",
            "fk_mean_x_mm", "fk_mean_y_mm", "fk_mean_z_mm",
            "fk_error_x_mm", "fk_error_y_mm", "fk_error_z_mm", "fk_error_norm_mm",
            "fk_std_x_mm", "fk_std_y_mm", "fk_std_z_mm", "fk_noise_norm_mm",
            "sample_count", "manual_x_mm", "manual_y_mm", "manual_z_mm",
            "manual_error_x_mm", "manual_error_y_mm", "manual_error_z_mm",
            "manual_error_norm_mm", "manual_reported_distance_error_mm",
            "manual_note", "manual_timestamp_utc",
        ]
        for motor_id in (11, 12, 13, 14):
            prefix = f"id{motor_id}"
            headers.extend(
                (
                    f"{prefix}_raw_mean", f"{prefix}_raw_std",
                    f"{prefix}_motor_deg_mean", f"{prefix}_motor_deg_std",
                    f"{prefix}_q_deg_mean", f"{prefix}_q_deg_std",
                    f"{prefix}_velocity_rpm_mean", f"{prefix}_velocity_rpm_abs_max",
                    f"{prefix}_current_ma_mean", f"{prefix}_current_ma_abs_max",
                    f"{prefix}_pwm_percent_mean", f"{prefix}_pwm_percent_abs_max",
                    f"{prefix}_voltage_v_mean", f"{prefix}_voltage_v_min",
                    f"{prefix}_temperature_c_mean", f"{prefix}_temperature_c_max",
                    f"{prefix}_hardware_error_or", f"{prefix}_moving_max",
                    f"{prefix}_moving_status_or",
                )
            )
        return headers

    def _summary_row(self, record: TouchRecord) -> list:
        mean_fk = record.mean_physical_fk()
        std_fk = record.fk_std()
        fk_error = record.fk_error()
        manual_xyz = record.manual.measured_xyz if record.manual else None
        manual_error = record.manual_error()
        row = [
            self.run.session_id, record.cycle, record.point_name, record.touch_index,
            record.recorded_utc, record.status,
        ]
        self._append_xyz(row, record.commanded_physical)
        self._append_xyz(row, record.commanded_internal)
        row.extend(record.planned_joints.as_tuple())
        row.extend(record.planned_motors.as_tuple())
        self._append_xyz(row, mean_fk)
        row.extend(fk_error)
        self._append_xyz(row, std_fk)
        row.extend((math.sqrt(std_fk.x ** 2 + std_fk.y ** 2 + std_fk.z ** 2), len(record.samples)))
        self._append_xyz(row, manual_xyz)
        row.extend(manual_error if manual_error is not None else (None, None, None, None))
        row.extend(
            (
                record.manual.reported_distance_error_mm if record.manual else None,
                record.manual.note if record.manual else "",
                record.manual.timestamp_utc if record.manual else "",
            )
        )

        for motor_index in range(4):
            motor_samples = [sample.motors[motor_index] for sample in record.samples]
            states = [sample.state for sample in record.samples]
            raw = [sample.position_raw for sample in motor_samples]
            motor_deg = [state.motors.as_tuple()[motor_index] for state in states]
            joint_deg = [state.joints.as_tuple()[motor_index] for state in states]
            velocity = [sample.velocity_rpm for sample in motor_samples]
            current = [sample.current_ma for sample in motor_samples]
            pwm = [sample.pwm_percent for sample in motor_samples]
            voltage = [sample.voltage_v for sample in motor_samples]
            temperature = [sample.temperature_c for sample in motor_samples]
            row.extend(
                (
                    _mean(raw), _std(raw), _mean(motor_deg), _std(motor_deg),
                    _mean(joint_deg), _std(joint_deg),
                    _mean(velocity), max(abs(value) for value in velocity),
                    _mean(current), max(abs(value) for value in current),
                    _mean(pwm), max(abs(value) for value in pwm),
                    _mean(voltage), min(voltage), _mean(temperature), max(temperature),
                    self._bitwise_or(sample.hardware_error for sample in motor_samples),
                    max(sample.moving for sample in motor_samples),
                    self._bitwise_or(sample.moving_status for sample in motor_samples),
                )
            )
        return row

    @staticmethod
    def _bitwise_or(values: Iterable[int]) -> int:
        result = 0
        for value in values:
            result |= int(value)
        return result

    def _sample_headers(self) -> list[str]:
        headers = [
            "session_id", "cycle", "point", "touch_index", "sample_index", "timestamp_utc",
            "command_x_mm", "command_y_mm", "command_z_mm",
            "fk_x_mm", "fk_y_mm", "fk_z_mm",
            "fk_error_x_mm", "fk_error_y_mm", "fk_error_z_mm", "fk_error_norm_mm",
        ]
        for motor_id in (11, 12, 13, 14):
            prefix = f"id{motor_id}"
            headers.extend(
                (
                    f"{prefix}_position_raw", f"{prefix}_motor_deg", f"{prefix}_q_deg",
                    f"{prefix}_velocity_raw", f"{prefix}_velocity_rpm",
                    f"{prefix}_current_raw", f"{prefix}_current_ma",
                    f"{prefix}_pwm_raw", f"{prefix}_pwm_percent",
                    f"{prefix}_voltage_raw", f"{prefix}_voltage_v",
                    f"{prefix}_temperature_c", f"{prefix}_hardware_error",
                    f"{prefix}_moving", f"{prefix}_moving_status",
                )
            )
        return headers

    def _sample_row(self, record: TouchRecord, sample_index: int, sample: TelemetrySnapshot) -> list:
        state = sample.state
        error = _vector_error(state.physical_xyz, record.commanded_physical)
        row = [
            self.run.session_id, record.cycle, record.point_name, record.touch_index,
            sample_index, sample.timestamp_utc,
            record.commanded_physical.x, record.commanded_physical.y, record.commanded_physical.z,
            state.physical_xyz.x, state.physical_xyz.y, state.physical_xyz.z,
            *error,
        ]
        for index, motor in enumerate(sample.motors):
            row.extend(
                (
                    motor.position_raw, state.motors.as_tuple()[index], state.joints.as_tuple()[index],
                    motor.velocity_raw, motor.velocity_rpm,
                    motor.current_raw, motor.current_ma,
                    motor.pwm_raw, motor.pwm_percent,
                    motor.voltage_raw, motor.voltage_v,
                    motor.temperature_c, motor.hardware_error, motor.moving, motor.moving_status,
                )
            )
        return row

    def _build_workbook(self) -> Workbook:
        workbook = Workbook()
        summary = workbook.active
        summary.title = "Touch Summary"
        summary.append(self._summary_headers())
        for record in self.records:
            summary.append(self._summary_row(record))

        samples = workbook.create_sheet("Telemetry Samples")
        samples.append(self._sample_headers())
        for record in self.records:
            for sample_index, sample in enumerate(record.samples, start=1):
                samples.append(self._sample_row(record, sample_index, sample))

        run_config = workbook.create_sheet("Run Config")
        run_config.append(("field", "value"))
        run_config_rows = (
            ("session_id", self.run.session_id),
            ("started_utc", self.run.started_utc),
            ("selected_points", ", ".join(self.run.selected_points)),
            ("repetitions", self.run.repetitions),
            ("touch_sample_count", self.run.sample_count),
            ("sample_interval_ms", self.run.sample_interval_ms),
            ("pause_for_manual_measurement", self.run.pause_for_manual_measurement),
            ("kinematics_version", config.KINEMATICS_VERSION),
            ("tcp_definition", config.TCP_DESCRIPTION),
            ("coordinate_units", "millimetres"),
            ("physical_frame", "ID11 center; +X forward; +Y right; +Z up"),
            ("work_xyz_mm", str(config.WORK_XYZ_MM)),
            ("approach_clearance_mm", config.APPROACH_CLEARANCE_MM),
            ("retract_clearance_mm", config.RETRACT_CLEARANCE_MM),
        )
        for item in run_config_rows:
            run_config.append(item)

        events = workbook.create_sheet("Run Events")
        events.append(("timestamp_utc", "event", "details"))
        for event in self.events:
            events.append(event)

        for sheet in workbook.worksheets:
            self._style_sheet(sheet)
        return workbook

    def save(self) -> Path:
        workbook = self._build_workbook()
        temporary = self.path.with_name(self.path.stem + ".saving.xlsx")
        try:
            workbook.save(temporary)
            os.replace(temporary, self.path)
        except PermissionError:
            temporary.unlink(missing_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            recovery = self.run_dir / f"repeatability_results_{timestamp}.xlsx"
            workbook.save(recovery)
            self.path = recovery
        finally:
            workbook.close()
        return self.path


__all__ = [
    "ManualMeasurement",
    "RepeatabilityRunConfig",
    "RepeatabilityWorkbook",
    "TouchRecord",
]
