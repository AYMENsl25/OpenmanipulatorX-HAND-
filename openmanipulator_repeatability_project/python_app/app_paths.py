"""Separate portable user data from bundled read-only application assets."""
from pathlib import Path
import sys

FROZEN = bool(getattr(sys, "frozen", False))
WORKSPACE_ROOT = Path(sys.executable).resolve().parent if FROZEN else Path(__file__).resolve().parents[2]
PROJECT_ROOT = WORKSPACE_ROOT / "openmanipulator_repeatability_project"
ASSET_ROOT = (Path(sys._MEIPASS) / "assets") if FROZEN else PROJECT_ROOT / "assets"
