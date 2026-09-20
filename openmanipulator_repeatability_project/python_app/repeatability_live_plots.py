"""Dependency-free live charts for repeatability touch calculations."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

import config
from repeatability_experiment import TouchCalculation


COLORS = ("#1565c0", "#ef6c00", "#2e7d32", "#8e24aa")


class LinePlot(ttk.Frame):
    def __init__(self, master, title: str, y_label: str, series_names: tuple[str, ...], thresholds=()):
        super().__init__(master)
        self.title = title
        self.y_label = y_label
        self.series_names = series_names
        self.thresholds = thresholds
        self.rows: list[tuple[str, tuple[float, ...]]] = []
        self.canvas = tk.Canvas(self, background="white", highlightthickness=1, highlightbackground="#aaa")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

    def clear(self) -> None:
        self.rows.clear()
        self.redraw()

    def append(self, label: str, values: tuple[float, ...]) -> None:
        self.rows.append((label, values))
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 500), max(canvas.winfo_height(), 220)
        left, right, top, bottom = 64, 18, 34, 48
        canvas.create_text(left, 8, text=self.title, anchor="nw", font=("TkDefaultFont", 11, "bold"))
        plot_w, plot_h = width - left - right, height - top - bottom
        values = [v for _, row in self.rows for v in row if math.isfinite(v)]
        values.extend(t[1] for t in self.thresholds)
        extent = max([abs(v) for v in values] + [1.0]) * 1.12
        y_min = -extent if any(v < 0 for v in values) or self.thresholds else 0.0
        y_max = extent
        def sx(i: int) -> float:
            return left + (i * plot_w / max(len(self.rows) - 1, 1))
        def sy(v: float) -> float:
            return top + (y_max - v) * plot_h / (y_max - y_min)
        canvas.create_rectangle(left, top, width-right, height-bottom, outline="#888")
        for fraction in range(5):
            value = y_min + fraction * (y_max-y_min) / 4
            y = sy(value)
            canvas.create_line(left, y, width-right, y, fill="#e6e6e6")
            canvas.create_text(left-6, y, text=f"{value:.1f}", anchor="e")
        canvas.create_text(12, top + plot_h/2, text=self.y_label, angle=90)
        for name, value, color, dash in self.thresholds:
            for signed in ({value, -value} if y_min < 0 else {value}):
                canvas.create_line(left, sy(signed), width-right, sy(signed), fill=color, dash=dash, width=2)
            canvas.create_text(width-right-4, sy(value)-4, text=name, fill=color, anchor="se")
        for series_index, name in enumerate(self.series_names):
            points = [(sx(i), sy(row[series_index])) for i, (_, row) in enumerate(self.rows)]
            if len(points) > 1:
                canvas.create_line(*[coordinate for point in points for coordinate in point], fill=COLORS[series_index], width=2)
            for x, y in points:
                canvas.create_oval(x-2, y-2, x+2, y+2, fill=COLORS[series_index], outline="")
            canvas.create_text(left + series_index * 105, 25, text=name, fill=COLORS[series_index], anchor="w")
        if self.rows:
            step = max(1, len(self.rows) // 8)
            for i, (label, _) in enumerate(self.rows):
                if i % step == 0 or i == len(self.rows)-1:
                    canvas.create_text(sx(i), height-bottom+5, text=label, anchor="n", angle=30)


class RepeatabilityPlotPanel(ttk.Frame):
    """Reusable plot notebook for both the main GUI and the pop-out window."""

    def __init__(self, master: tk.Misc):
        super().__init__(master)
        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True)
        warning = config.MOTOR_TRACKING_WARNING_DEGREES
        stop = config.POST_MOVE_TOLERANCE_DEGREES
        self.motor = LinePlot(notebook, "Motor Error", "error (deg)", ("ID11", "ID12", "ID13", "ID14"), (("warning", warning, "#ef6c00", (5, 3)), ("10 deg stop", stop, "#c62828", (2, 2))))
        self.xyz = LinePlot(notebook, "XYZ Error", "error (mm)", ("X error", "Y error", "Z error", "XYZ norm"))
        self.noise = LinePlot(notebook, "Within-touch FK Sample Variation", "standard deviation (mm)", ("X std", "Y std", "Z std", "noise norm"))
        notebook.add(self.motor, text="Motor Error")
        notebook.add(self.xyz, text="XYZ Error")
        notebook.add(self.noise, text="FK Sample Noise")

    def clear(self) -> None:
        self.motor.clear(); self.xyz.clear(); self.noise.clear()

    def add_result(self, result: TouchCalculation) -> None:
        label = result.axis_label
        self.motor.append(label, result.motor_errors)
        self.xyz.append(label, result.fk_error)
        self.noise.append(
            label,
            (result.fk_std.x, result.fk_std.y, result.fk_std.z, result.fk_noise_norm),
        )


class RepeatabilityLivePlots:
    def __init__(self, master: tk.Misc):
        self.window = tk.Toplevel(master)
        self.window.title("Repeatability Live Error Plots")
        self.window.geometry("1100x760")
        self.window.minsize(720, 500)
        self.panel = RepeatabilityPlotPanel(self.window)
        self.panel.pack(fill="both", expand=True, padx=8, pady=8)
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)

    def clear(self) -> None:
        self.panel.clear()

    def add_result(self, result: TouchCalculation) -> None:
        self.panel.add_result(result)
        self.window.deiconify()

    def show(self) -> None:
        self.window.deiconify(); self.window.lift()

    def exists(self) -> bool:
        return bool(self.window.winfo_exists())
