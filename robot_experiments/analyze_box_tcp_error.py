"""Compare measured jaw-midpoint positions with existing FK; never changes control calibration."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


DEFAULT_INPUT = Path(__file__).parent.parent / "calibration" / "box_point_observations_20260923.csv"
FRAME_CONFIG = (Path(__file__).parent.parent / "openmanipulator_repeatability_project" /
                "config" / "experiment_config.json")


def _number(row: dict[str, str], field: str) -> float | None:
    value = row[field].strip()
    return float(value) if value else None


def analyze(path: Path) -> dict:
    frame = json.loads(FRAME_CONFIG.read_text(encoding="utf-8"))
    rear_to_axis_mm = float(frame["operator_rear_frame"]["rear_to_axis_x_mm"])
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    observations = []
    missing = []
    for row in rows:
        actual_x = _number(row, "observed_jaw_x_mm")
        fk_x = _number(row, "fk_x_mm")
        if actual_x is None or fk_x is None:
            missing.append(row["point_id"])
            continue
        origin = row["measurement_origin"]
        if origin not in ("ID11_motor_center", "ID11_rear_face"):
            raise ValueError(f"{row['point_id']}: unknown measurement origin {origin!r}")
        predicted_x = fk_x + (rear_to_axis_mm if origin == "ID11_rear_face" else 0.0)
        q1, q2, q3, q4 = (_number(row, f"q{i}_deg") for i in range(1, 5))
        if any(value is None for value in (q1, q2, q3, q4)):
            raise ValueError(f"{row['point_id']}: joint angles are required with an X observation")
        yaw = math.radians(q1)
        tool_pitch = -math.radians(q2 + q3 + q4)
        # This is the last-link axis from the existing FK, not a new FK model.
        axis_x = math.cos(yaw) * math.cos(tool_pitch)
        axis_z = math.sin(tool_pitch)
        actual_y = _number(row, "observed_jaw_y_mm")
        actual_z = _number(row, "observed_jaw_z_mm")
        fk_y = _number(row, "fk_y_mm")
        fk_z = _number(row, "fk_z_mm")
        observations.append({
            "point_id": row["point_id"], "measurement_origin": origin,
            "fk_x_axis_mm": fk_x, "predicted_x_at_measurement_origin_mm": predicted_x,
            "error_x_mm": actual_x - predicted_x,
            "error_y_mm": None if actual_y is None or fk_y is None else actual_y - fk_y,
            "error_z_mm": None if actual_z is None or fk_z is None else actual_z - fk_z,
            "tool_axis_x": axis_x, "tool_axis_z": axis_z,
        })
    if not observations:
        return {"status": "NEED_PHYSICAL_JAW_X", "missing_points": missing, "rows_used": 0}
    errors = [item["error_x_mm"] for item in observations]
    constant_x = statistics.mean(errors)
    axis_denominator = sum(item["tool_axis_x"] ** 2 for item in observations)
    tool_length = (sum(item["tool_axis_x"] * item["error_x_mm"] for item in observations)
                   / axis_denominator) if axis_denominator > 1e-9 else None
    constant_rmse = math.sqrt(statistics.mean((error - constant_x) ** 2 for error in errors))
    tool_rmse = (None if tool_length is None else math.sqrt(statistics.mean(
        (item["error_x_mm"] - tool_length * item["tool_axis_x"]) ** 2
        for item in observations)))
    return {
        "status": "DIAGNOSTIC_ONLY", "rows_used": len(observations),
        "missing_points": missing, "observations": observations,
        "mean_x_error_mm": constant_x, "x_error_sd_mm": statistics.pstdev(errors),
        "constant_x_model_rmse_mm": constant_rmse,
        "tool_extension_fit_mm": tool_length,
        "tool_extension_x_model_rmse_mm": tool_rmse,
        "rear_to_axis_x_mm_nominal": rear_to_axis_mm,
        "note": "Fits compare residual physical error AFTER frame conversion; they are not corrections. Check measured Y/Z and the actual rear-to-axis distance before changing FK.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, help="Optional JSON report path")
    args = parser.parse_args()
    report = analyze(args.csv)
    message = json.dumps(report, indent=2)
    print(message)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(message + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
