"""Thread-safe, append-only logging for OpenMANIPULATOR experiments.

This module has no serial or motion dependencies.  It only records experiment
events and telemetry beneath ``<project>/logs/<session_id>/`` so a hardware run
can be reconstructed and debugged later.
"""

from __future__ import annotations

import csv
from dataclasses import fields, is_dataclass
from datetime import date, datetime, time, timezone
from enum import Enum
import json
import math
from pathlib import Path
import re
import threading
import traceback
from typing import Any, Mapping, TextIO
from uuid import uuid4


_SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")

# Frequently inspected robot values get their own CSV columns.  The complete
# sample is also retained in values_json, so adding a new diagnostic does not
# require changing this logger or losing data from an older reader.
TELEMETRY_COLUMNS = (
    "timestamp_utc",
    "session_id",
    "sequence",
    "event",
    "category",
    "elapsed_s",
    "sample_index",
    "id11_raw",
    "id12_raw",
    "id13_raw",
    "id14_raw",
    "id15_raw",
    "id11_deg",
    "id12_deg",
    "id13_deg",
    "id14_deg",
    "id15_deg",
    "q1_deg",
    "q2_deg",
    "q3_deg",
    "q4_deg",
    "physical_x_mm",
    "physical_y_mm",
    "physical_z_mm",
    "internal_x_mm",
    "internal_y_mm",
    "internal_z_mm",
    "target_x_mm",
    "target_y_mm",
    "target_z_mm",
    "position_error_mm",
    "values_json",
)


def _utc_timestamp() -> str:
    """Return an ISO-8601 UTC timestamp suitable for sorting and JSON."""

    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace(
        "+00:00", "Z"
    )


def new_session_id() -> str:
    """Create a filesystem-safe, collision-resistant experiment session ID."""

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%f")[:-3] + "Z"
    return f"{stamp}-{uuid4().hex[:8]}"


def _safe_repr(value: Any) -> str:
    try:
        return repr(value)
    except Exception:
        return f"<{type(value).__name__}: representation failed>"


def _json_safe(value: Any, *, _seen: set[int] | None = None, _depth: int = 0) -> Any:
    """Recursively convert arbitrary diagnostics to strict JSON-safe values.

    Dataclass instances are expanded by field instead of using ``asdict`` so a
    recursive or unusual diagnostic object cannot take down the error logger.
    """

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, (datetime, date, time)):
        return value.isoformat()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, Enum):
        return _json_safe(value.value, _seen=_seen, _depth=_depth + 1)
    if isinstance(value, BaseException):
        return {
            "exception_type": type(value).__name__,
            "message": str(value),
        }
    if _depth >= 30:
        return f"<{type(value).__name__}: maximum serialization depth reached>"

    if _seen is None:
        _seen = set()
    object_id = id(value)
    if object_id in _seen:
        return f"<{type(value).__name__}: recursive reference>"

    _seen.add(object_id)
    try:
        if is_dataclass(value) and not isinstance(value, type):
            return {
                field.name: _json_safe(
                    getattr(value, field.name), _seen=_seen, _depth=_depth + 1
                )
                for field in fields(value)
            }
        if isinstance(value, Mapping):
            return {
                str(key): _json_safe(item, _seen=_seen, _depth=_depth + 1)
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [
                _json_safe(item, _seen=_seen, _depth=_depth + 1) for item in value
            ]
        if isinstance(value, (set, frozenset)):
            ordered = sorted(value, key=_safe_repr)
            return [
                _json_safe(item, _seen=_seen, _depth=_depth + 1) for item in ordered
            ]
        if hasattr(value, "__dict__"):
            attributes = {
                key: item
                for key, item in vars(value).items()
                if not str(key).startswith("_")
            }
            if attributes:
                return {
                    "object_type": type(value).__name__,
                    "attributes": _json_safe(
                        attributes, _seen=_seen, _depth=_depth + 1
                    ),
                }
        return _safe_repr(value)
    except Exception as serialization_error:
        return {
            "object_type": type(value).__name__,
            "representation": _safe_repr(value),
            "serialization_error": str(serialization_error),
        }
    finally:
        _seen.discard(object_id)


def _json_text(value: Any) -> str:
    return json.dumps(
        _json_safe(value),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )


class ExperimentLogger:
    """Write one experiment session's history, errors, and telemetry.

    The object is safe to call from the GUI, serial receiver, and monitoring
    threads at the same time.  Every record is flushed before the method
    returns.  Use it as a context manager or call :meth:`close` at shutdown.
    """

    def __init__(
        self,
        *,
        project_root: str | Path | None = None,
        session_id: str | None = None,
    ) -> None:
        root = (
            Path(project_root)
            if project_root is not None
            else Path(__file__).resolve().parents[1]
        ).resolve()
        self.session_id = session_id or new_session_id()
        if not _SESSION_ID_PATTERN.fullmatch(self.session_id):
            raise ValueError(
                "session_id must be 1-128 filesystem-safe characters "
                "(letters, numbers, dot, underscore, or hyphen)"
            )

        self.project_root = root
        self.session_dir = root / "logs" / self.session_id
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.history_path = self.session_dir / "history.jsonl"
        self.error_path = self.session_dir / "errors.jsonl"
        self.telemetry_path = self.session_dir / "telemetry.csv"

        self._lock = threading.RLock()
        self._closed = False
        self._sequence = self._largest_existing_sequence()
        self._history_file = self.history_path.open("a", encoding="utf-8", newline="")
        self._error_file = self.error_path.open("a", encoding="utf-8", newline="")
        telemetry_is_empty = (
            not self.telemetry_path.exists() or self.telemetry_path.stat().st_size == 0
        )
        self._telemetry_file = self.telemetry_path.open(
            "a", encoding="utf-8", newline=""
        )
        self._telemetry_writer = csv.DictWriter(
            self._telemetry_file, fieldnames=TELEMETRY_COLUMNS
        )
        if telemetry_is_empty:
            self._telemetry_writer.writeheader()
            self._telemetry_file.flush()

    def _largest_existing_sequence(self) -> int:
        largest = -1
        for path in (self.history_path, self.error_path):
            if not path.exists():
                continue
            try:
                with path.open("r", encoding="utf-8") as source:
                    for line in source:
                        try:
                            largest = max(largest, int(json.loads(line)["sequence"]))
                        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                            continue
            except OSError:
                continue
        if self.telemetry_path.exists():
            try:
                with self.telemetry_path.open("r", encoding="utf-8", newline="") as source:
                    for row in csv.DictReader(source):
                        try:
                            largest = max(largest, int(row["sequence"]))
                        except (KeyError, TypeError, ValueError):
                            continue
            except OSError:
                pass
        return largest

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("ExperimentLogger is closed")

    @staticmethod
    def _validate_label(label: str, field_name: str) -> str:
        if not isinstance(label, str) or not label.strip():
            raise ValueError(f"{field_name} must be a non-empty string")
        return label.strip()

    def _base_record(self, event: str, category: str) -> dict[str, Any]:
        return {
            "timestamp_utc": _utc_timestamp(),
            "session_id": self.session_id,
            "sequence": self._next_sequence(),
            "event": self._validate_label(event, "event"),
            "category": self._validate_label(category, "category"),
        }

    @staticmethod
    def _write_json_line(destination: TextIO, record: Mapping[str, Any]) -> None:
        destination.write(_json_text(record) + "\n")
        destination.flush()

    def log_history(
        self,
        event: str,
        *,
        category: str = "experiment",
        context: Any = None,
    ) -> dict[str, Any]:
        """Append a normal lifecycle, command, state, or validation event."""

        with self._lock:
            self._ensure_open()
            record = self._base_record(event, category)
            record["context"] = _json_safe(context)
            self._write_json_line(self._history_file, record)
            return record

    def log_error(
        self,
        event: str,
        exception: BaseException,
        *,
        category: str = "error",
        context: Any = None,
    ) -> dict[str, Any]:
        """Append an exception without assuming its context is JSON serializable."""

        if not isinstance(exception, BaseException):
            raise TypeError("exception must be a BaseException instance")
        with self._lock:
            self._ensure_open()
            record = self._base_record(event, category)
            record.update(
                {
                    "exception_type": type(exception).__name__,
                    "message": str(exception),
                    "context": _json_safe(context),
                    "traceback": "".join(
                        traceback.format_exception(
                            type(exception), exception, exception.__traceback__
                        )
                    ).rstrip(),
                }
            )
            self._write_json_line(self._error_file, record)
            return record

    def log_telemetry(
        self,
        values: Mapping[str, Any] | Any,
        *,
        event: str = "telemetry_sample",
        category: str = "telemetry",
    ) -> dict[str, Any]:
        """Append one real-time sample to CSV and return the written row.

        A mapping populates matching dedicated columns.  Dataclass samples and
        other objects are still preserved safely in ``values_json``.
        """

        with self._lock:
            self._ensure_open()
            base = self._base_record(event, category)
            safe_values = _json_safe(values)
            mapped_values = safe_values if isinstance(safe_values, Mapping) else {}
            row = {column: "" for column in TELEMETRY_COLUMNS}
            for key in ("timestamp_utc", "session_id", "sequence", "event", "category"):
                row[key] = base[key]
            for column in TELEMETRY_COLUMNS:
                if column in mapped_values and column != "values_json":
                    cell = mapped_values[column]
                    row[column] = (
                        cell
                        if cell is None or isinstance(cell, (str, bool, int, float))
                        else _json_text(cell)
                    )
            row["values_json"] = _json_text(safe_values)
            self._telemetry_writer.writerow(row)
            self._telemetry_file.flush()
            return row

    def flush(self) -> None:
        """Flush every log explicitly (writes already flush automatically)."""

        with self._lock:
            self._ensure_open()
            self._history_file.flush()
            self._error_file.flush()
            self._telemetry_file.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._history_file.flush()
            self._error_file.flush()
            self._telemetry_file.flush()
            self._history_file.close()
            self._error_file.close()
            self._telemetry_file.close()
            self._closed = True

    def __enter__(self) -> "ExperimentLogger":
        return self

    def __exit__(self, exc_type: Any, exc: Any, exc_tb: Any) -> bool:
        if isinstance(exc, BaseException):
            try:
                self.log_error(
                    "unhandled_session_exception",
                    exc,
                    category="session",
                )
            except Exception:
                # Never replace the original experiment exception with a logging
                # failure during context-manager cleanup.
                pass
        self.close()
        return False


__all__ = ["ExperimentLogger", "TELEMETRY_COLUMNS", "new_session_id"]
