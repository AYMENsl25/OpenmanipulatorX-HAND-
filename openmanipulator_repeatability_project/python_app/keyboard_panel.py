"""Keyboard input for the shared continuous Cartesian jog worker."""
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import config
from cartesian_jog import CartesianJogPanel, POLL_MS


KEY_DIRECTIONS = {
    "w": (1, 0, 0), "up": (1, 0, 0),
    "s": (-1, 0, 0), "down": (-1, 0, 0),
    "a": (0, 1, 0), "left": (0, 1, 0),
    "d": (0, -1, 0), "right": (0, -1, 0),
    "r": (0, 0, 1), "prior": (0, 0, 1),
    "f": (0, 0, -1), "next": (0, 0, -1),
}


def keyboard_vector(keys):
    # Aliases must not double the speed. Opposite directions cancel.
    directions = {KEY_DIRECTIONS[key] for key in keys if key in KEY_DIRECTIONS}
    vector = tuple(sum(direction[i] for direction in directions) for i in range(3))
    length = max(1.0, math.sqrt(sum(value * value for value in vector)))
    return tuple(value / length for value in vector)


class KeyboardPanel(CartesianJogPanel):
    def __init__(self, *args, **kwargs):
        self._keys = set()
        super().__init__(*args, **kwargs)
        self.status.set("Keyboard movement OFF — click ENABLE KEYBOARD to begin")
        self.inputs.set("No keys held")

    def _build(self):
        self.columnconfigure(0, weight=1)
        ttk.Label(self, text="Live keyboard movement", font=("TkDefaultFont", 14, "bold")).grid(
            row=0, column=0, sticky="w")
        ttk.Label(self, text="Enable once, then hold a direction key to move. Release to stop. "
                  "No controller required and no XYZ entry for each movement.",
                  wraplength=850).grid(row=1, column=0, sticky="w", pady=8)
        actions = ttk.Frame(self)
        actions.grid(row=2, column=0, sticky="w", pady=6)
        ttk.Button(actions, text="MOVE TO WORK", command=self.move_to_work).pack(side="left", padx=4)
        ttk.Button(actions, text="ENABLE KEYBOARD", command=self.arm).pack(side="left", padx=4)
        ttk.Button(actions, text="STOP / DISABLE", command=self.disarm).pack(side="left", padx=4)
        ttk.Label(actions, text="Speed").pack(side="left", padx=8)
        ttk.Spinbox(actions, from_=0.25, to=2.0, increment=0.25,
                    textvariable=self.speed, width=5).pack(side="left")
        self.jog_pad = tk.Canvas(self, height=185, background="#edf5fa",
                                 highlightthickness=2, highlightbackground="#7893a3",
                                 highlightcolor="#007fa5", takefocus=True)
        self.jog_pad.grid(row=3, column=0, sticky="ew", pady=10)
        self.jog_pad.create_text(20, 20, anchor="nw", font=("Segoe UI", 12), text=(
            "W / ↑     Forward (+X)          S / ↓     Backward (−X)\n\n"
            "A / ←     Left (+Y)                   D / →     Right (−Y)\n\n"
            "R / Page Up     Up (+Z)         F / Page Down     Down (−Z)\n\n"
            "Esc / Space     STOP     •     Moving focus away stops keyboard motion"))
        self.jog_pad.bind("<KeyPress>", self._key_press)
        self.jog_pad.bind("<KeyRelease>", self._key_release)
        self.jog_pad.bind("<FocusOut>", self._focus_lost)
        self.jog_pad.bind("<Button-1>", lambda event: self.jog_pad.focus_set())
        ttk.Label(self, textvariable=self.inputs).grid(row=4, column=0, sticky="w")
        ttk.Label(self, textvariable=self.status, wraplength=850).grid(row=5, column=0, sticky="w", pady=8)
        limits = config.SOFT_WORKSPACE
        ttk.Label(self, text=(
            f"Workspace: X rear {limits['x_min_mm'] + config.REAR_TO_AXIS_X_MM:g} to "
            f"{limits['x_max_mm'] + config.REAR_TO_AXIS_X_MM:g} mm; "
            f"Y {limits['y_min_mm']:g} to {limits['y_max_mm']:g} mm; Z ≥ {config.GROUND_Z_MM:g} mm. "
            "Reachability and joint limits also apply."), wraplength=850).grid(row=6, column=0, sticky="w")

    def _check_input(self):
        if self.running:
            raise ValueError("Wait for the previous jog to stop, then enable again")
        self._keys.clear()

    def arm(self):
        super().arm()
        if self.armed:
            self.jog_pad.focus_set()
            self.status.set("Keyboard enabled — hold W/S, A/D or R/F; release to stop")

    def disarm(self):
        self._keys.clear()
        super().disarm()

    def _focus_lost(self, event=None):
        if self.armed:
            self.disarm()
            self.status.set("Keyboard stopped after focus change. Click ENABLE KEYBOARD to resume.")

    def _key_press(self, event):
        key = event.keysym.lower()
        if key in ("escape", "space"):
            self.disarm()
            return "break"
        if self.armed and key in KEY_DIRECTIONS:
            self._keys.add(key)
            return "break"

    def _key_release(self, event):
        self._keys.discard(event.keysym.lower())
        if not any(keyboard_vector(self._keys)):
            self._cancel.set()
        return "break" if event.keysym.lower() in KEY_DIRECTIONS else None

    def _poll(self):
        if self.closed:
            return
        try:
            if self.armed:
                if not self.jog_pad.winfo_viewable() or self.focus_get() != self.jog_pad:
                    self._focus_lost()
                elif self.busy_callback() or not self.controller.status.connected or not self.controller.status.torque_on:
                    self.disarm()
                    self.status.set("Keyboard stopped: robot disconnected, torque off, or another operation active")
                else:
                    speed = float(self.speed.get())
                    if not math.isfinite(speed) or not 0.25 <= speed <= 2.0:
                        raise ValueError("Speed must be 0.25–2.0")
                    self._speed_value = speed
                    vector = keyboard_vector(self._keys)
                    moving = any(vector)
                    with self._lock:
                        self._sample = (time.monotonic(), vector, moving, False)
                    if not moving:
                        self._cancel.set()
                    elif not self.running:
                        self._cancel.clear()
                        self.running = True
                        threading.Thread(target=self._work, daemon=True).start()
            self.inputs.set("Held: " + ", ".join(sorted(self._keys)) if self._keys else "No keys held — holding position")
        except (ValueError, tk.TclError) as exc:
            self.disarm()
            self.status.set(str(exc))
        self.after(POLL_MS, self._poll)
