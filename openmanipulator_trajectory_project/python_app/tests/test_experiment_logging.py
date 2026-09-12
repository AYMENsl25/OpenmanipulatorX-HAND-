import csv
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import sys
import threading
import unittest
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiment_logging import ExperimentLogger, TELEMETRY_COLUMNS


TEST_TEMP_ROOT = Path(__file__).resolve().parent / "_experiment_logging_tmp"


@dataclass(frozen=True)
class PoseDiagnostic:
    point_name: str
    xyz_mm: tuple[float, float, float]


class ExperimentLoggingTests(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = TEST_TEMP_ROOT / uuid4().hex
        self.temporary_directory.mkdir(parents=True)

    def tearDown(self):
        shutil.rmtree(self.temporary_directory, ignore_errors=True)
        try:
            TEST_TEMP_ROOT.rmdir()
        except OSError:
            pass

    def test_creates_session_scoped_append_only_files(self):
        root = self.temporary_directory
        with ExperimentLogger(project_root=root, session_id="ground-run-001") as logger:
            logger.log_history(
                "preview_passed",
                category="validation",
                context={"point": "P03"},
            )
            self.assertEqual(
                logger.session_dir, root.resolve() / "logs" / "ground-run-001"
            )

        with ExperimentLogger(project_root=root, session_id="ground-run-001") as logger:
            logger.log_history("motion_complete", category="motion")

        history_path = root / "logs" / "ground-run-001" / "history.jsonl"
        records = [json.loads(line) for line in history_path.read_text(encoding="utf-8").splitlines()]
        self.assertEqual([record["event"] for record in records], ["preview_passed", "motion_complete"])
        self.assertEqual([record["sequence"] for record in records], [0, 1])
        for record in records:
            self.assertEqual(record["session_id"], "ground-run-001")
            self.assertTrue(record["timestamp_utc"].endswith("Z"))
            self.assertTrue(record["category"])

    def test_error_log_serializes_dataclass_context_and_exception(self):
        logger = ExperimentLogger(
            project_root=self.temporary_directory, session_id="error-context"
        )
        try:
            raise RuntimeError("IK verification failed")
        except RuntimeError as error:
            logger.log_error(
                "point_rejected",
                error,
                category="ik",
                context={
                    "pose": PoseDiagnostic("P07", (-35.0, -145.0, 0.0)),
                    "path": Path("trajectory_xyz.xlsx"),
                },
            )
        logger.close()

        record = json.loads(logger.error_path.read_text(encoding="utf-8"))
        self.assertEqual(record["event"], "point_rejected")
        self.assertEqual(record["category"], "ik")
        self.assertEqual(record["exception_type"], "RuntimeError")
        self.assertEqual(record["message"], "IK verification failed")
        self.assertEqual(record["context"]["pose"]["point_name"], "P07")
        self.assertEqual(record["context"]["pose"]["xyz_mm"], [-35.0, -145.0, 0.0])
        self.assertIn("RuntimeError: IK verification failed", record["traceback"])

    def test_telemetry_has_robot_columns_full_payload_and_one_header(self):
        for sample_index in range(2):
            with ExperimentLogger(
                project_root=self.temporary_directory, session_id="telemetry-run"
            ) as logger:
                logger.log_telemetry(
                    {
                        "sample_index": sample_index,
                        "elapsed_s": sample_index * 0.05,
                        "id11_raw": 1917 + sample_index,
                        "id14_deg": 82.881,
                        "physical_z_mm": 40.1,
                        "custom_status": {"stable": True},
                    },
                    event="angle_read",
                    category="feedback",
                )

        telemetry_path = self.temporary_directory / "logs" / "telemetry-run" / "telemetry.csv"
        with telemetry_path.open("r", encoding="utf-8", newline="") as source:
            rows = list(csv.DictReader(source))
        self.assertEqual(len(rows), 2)
        self.assertEqual(tuple(rows[0].keys()), TELEMETRY_COLUMNS)
        self.assertEqual(rows[0]["event"], "angle_read")
        self.assertEqual(rows[1]["id11_raw"], "1918")
        self.assertTrue(json.loads(rows[0]["values_json"])["custom_status"]["stable"])
        header_count = telemetry_path.read_text(encoding="utf-8").count("timestamp_utc")
        self.assertEqual(header_count, 1)

    def test_concurrent_history_writes_are_complete_and_valid(self):
        logger = ExperimentLogger(
            project_root=self.temporary_directory, session_id="threaded-run"
        )
        worker_count = 6
        records_per_worker = 40

        def write_records(worker: int) -> None:
            for index in range(records_per_worker):
                logger.log_history(
                    "angle_sample_processed",
                    category="monitor",
                    context={"worker": worker, "index": index},
                )

        workers = [
            threading.Thread(target=write_records, args=(worker,))
            for worker in range(worker_count)
        ]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()
        logger.close()

        lines = logger.history_path.read_text(encoding="utf-8").splitlines()
        records = [json.loads(line) for line in lines]
        expected = worker_count * records_per_worker
        self.assertEqual(len(records), expected)
        self.assertEqual(len({record["sequence"] for record in records}), expected)

    def test_invalid_session_id_cannot_escape_logs_directory(self):
        with self.assertRaises(ValueError):
            ExperimentLogger(
                project_root=self.temporary_directory, session_id="../outside"
            )


if __name__ == "__main__":
    unittest.main()
