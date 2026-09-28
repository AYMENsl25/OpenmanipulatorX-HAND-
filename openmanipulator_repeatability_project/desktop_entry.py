"""Desktop launcher and disconnected packaging smoke check."""
import sys
import traceback
from pathlib import Path


def main():
    try:
        from trajectory_gui import run
        if "--smoke-test" in sys.argv:
            import tkinter as tk
            import serial
            import cv2
            from app_paths import ASSET_ROOT, WORKSPACE_ROOT
            from trajectory_gui import TrajectoryGUI
            from scan_cube_xyz_preview import ScanCalibration

            def forbidden(*args, **kwargs):
                raise RuntimeError("Hardware access forbidden during packaging smoke check")

            serial.Serial = forbidden
            cv2.VideoCapture = forbidden
            root = tk.Tk()
            root.withdraw()
            app = TrajectoryGUI(root)
            root.update_idletasks()
            assert app._header_artwork is not None
            assert (ASSET_ROOT / "isu_xr_lab_header.png").is_file()
            ScanCalibration.load()
            root.destroy()
            (WORKSPACE_ROOT / "smoke-test-ok.txt").write_text(
                "PASS: GUI initialized, banner and calibration loaded; hardware access disabled.\n",
                encoding="utf-8",
            )
        else:
            run()
    except Exception:
        report = traceback.format_exc()
        output = Path(sys.executable).parent if getattr(sys, "frozen", False) else Path.cwd()
        (output / "startup-error.txt").write_text(report, encoding="utf-8")
        if "--smoke-test" not in sys.argv:
            from tkinter import messagebox
            messagebox.showerror("Application startup failed", f"See {output / 'startup-error.txt'}\n\n{report[-1500:]}")
        raise


if __name__ == "__main__":
    main()
