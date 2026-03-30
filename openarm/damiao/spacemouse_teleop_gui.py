"""Tkinter GUI for SpaceMouse during monitor.py teleop (optional --spacemouse-gui).

Mirrors spacemouse_test.py: gains, axis toggles (checkboxes), 6-DOF readout, buttons,
and 100-sample history plots. Runs in a daemon thread; shared state is updated from
the asyncio teleop loop.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from tkinter import font as tkfont

import tkinter as tk

from .spacemouse_ctrl import (
    GAIN_ROT_MAX,
    GAIN_ROT_MIN,
    GAIN_TRANS_MAX,
    GAIN_TRANS_MIN,
    SpacemouseSample,
)

HISTORY_LEN = 100
AXIS_LABELS = ("x", "y", "z", "roll", "pitch", "yaw")
PLOT_COLORS = ("#e74c3c", "#2ecc71", "#3498db", "#9b59b6", "#f39c12", "#1abc9c")


def _format_val(v: float, decimals: int = 3) -> str:
    return f"{v:+.{decimals}f}"


def _draw_history_canvas(
    canvas: tk.Canvas,
    history: deque[float],
    width: int,
    height: int,
    color: str,
) -> None:
    canvas.delete("all")
    margin = 4
    mid_y = height // 2
    canvas.create_line(margin, mid_y, width - margin, mid_y, fill="#444444", width=1)
    if len(history) < 2:
        return
    w_plot = width - 2 * margin
    pts: list[float] = []
    for i, v in enumerate(history):
        xi = margin + (i / max(len(history) - 1, 1)) * w_plot
        vc = max(-1.0, min(1.0, float(v)))
        yi = mid_y - vc * (mid_y - margin - 2)
        pts.extend([xi, yi])
    if len(pts) >= 4:
        canvas.create_line(*pts, fill=color, width=2, smooth=False)


@dataclass
class SpacemouseTeleopSharedState:
    """Thread-safe snapshot + axis list mutated by teleop loop and GUI checkboxes."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    axis_on: list[bool] = field(default_factory=lambda: [True] * 6)
    histories: list[deque[float]] = field(
        default_factory=lambda: [deque(maxlen=HISTORY_LEN) for _ in range(6)],
    )

    def monitor_publish(
        self,
        sample: SpacemouseSample,
        eff: list[float],
        left_mode: bool,
        connected: bool,
        error: str | None,
        gain_trans: float,
        gain_rot: float,
        append_history: bool,
    ) -> None:
        with self.lock:
            self._sample = SpacemouseSample(
                sample.x,
                sample.y,
                sample.z,
                sample.roll,
                sample.pitch,
                sample.yaw,
                tuple(sample.buttons) if sample.buttons else (0, 0),
            )
            self._eff = (list(eff) + [0.0] * 6)[:6]
            self.left_mode = left_mode
            self.connected = connected
            self.error = error
            self.gain_trans = gain_trans
            self.gain_rot = gain_rot
            if append_history:
                for i in range(6):
                    self.histories[i].append(float(self._eff[i]))

    def snapshot(self) -> tuple[SpacemouseSample, list[float], bool, bool, str | None, float, float, list[deque[float]]]:
        with self.lock:
            samp = SpacemouseSample(
                self._sample.x,
                self._sample.y,
                self._sample.z,
                self._sample.roll,
                self._sample.pitch,
                self._sample.yaw,
                self._sample.buttons,
            )
            eff = list(self._eff)
            hist = [deque(self.histories[i], maxlen=HISTORY_LEN) for i in range(6)]
            return (
                samp,
                eff,
                self.left_mode,
                self.connected,
                self.error,
                self.gain_trans,
                self.gain_rot,
                hist,
            )

    def __post_init__(self) -> None:
        self._sample = SpacemouseSample()
        self._eff = [0.0] * 6
        self.left_mode = False
        self.connected = False
        self.error = None
        self.gain_trans = 0.1
        self.gain_rot = 0.1


class SpacemouseTeleopGui:
    """Daemon-thread Tk window; call ``start()`` / ``stop()`` from teleop."""

    def __init__(self, shared: SpacemouseTeleopSharedState) -> None:
        self.shared = shared
        self._thread: threading.Thread | None = None
        self._root: tk.Tk | None = None
        self._stop = threading.Event()
        self._gui_batch = False
        self._axis_vars: list[tk.IntVar] = []
        self._plot_w = 420
        self._plot_h = 44
        self._plot_canvases: list[tk.Canvas] = []

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run_tk, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        root = self._root
        if root is not None:

            def _close() -> None:
                try:
                    root.quit()
                    root.destroy()
                except Exception:
                    pass

            try:
                root.after(0, _close)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._root = None

    def _run_tk(self) -> None:
        root = tk.Tk()
        self._root = root
        root.title("SpaceMouse — monitor teleop")
        root.geometry("780x720")

        main = tk.Frame(root, padx=12, pady=10)
        main.pack(fill=tk.BOTH, expand=True)

        title_font = tkfont.Font(size=13, weight="bold")
        tk.Label(main, text="SpaceMouse (monitor teleop)", font=title_font).pack(anchor=tk.W)

        self.status_var = tk.StringVar(value="Teleop running — press d in terminal for left-pair mode")
        tk.Label(main, textvariable=self.status_var, fg="gray").pack(anchor=tk.W)

        gain_frame = tk.LabelFrame(main, text="Gains (puck Btn0 ↑  Btn1 ↓)", padx=6, pady=4)
        gain_frame.pack(fill=tk.X, pady=(6, 4))
        self.gain_var = tk.StringVar(value="trans=0.10  rot=0.10")
        tk.Label(gain_frame, textvariable=self.gain_var, font=("Monospace", 10)).pack(anchor=tk.W)

        toggle_frame = tk.LabelFrame(main, text="Axis toggles (or keys x y z r p w in terminal)", padx=6, pady=4)
        toggle_frame.pack(fill=tk.X, pady=(0, 6))
        inner = tk.Frame(toggle_frame)
        inner.pack(anchor=tk.W)
        for i, (lab, col) in enumerate(zip(AXIS_LABELS, PLOT_COLORS)):
            f = tk.Frame(inner)
            f.pack(side=tk.LEFT, padx=6)
            tk.Label(f, text=lab, font=("Monospace", 9), fg=col).pack()
            var = tk.IntVar(value=1)
            self._axis_vars.append(var)

            def make_cb(idx: int) -> None:
                def _cb() -> None:
                    if self._gui_batch:
                        return
                    with self.shared.lock:
                        self.shared.axis_on[idx] = bool(self._axis_vars[idx].get())

                return _cb

            tk.Checkbutton(f, variable=var, command=make_cb(i)).pack()

        dof_frame = tk.LabelFrame(main, text="6-DOF effective (normalized × gain, toggles)", padx=6, pady=4)
        dof_frame.pack(fill=tk.X, pady=(0, 6))
        self.xyz_var = tk.StringVar(value="x: ---  y: ---  z: ---")
        self.rpy_var = tk.StringVar(value="roll: ---  pitch: ---  yaw: ---")
        tk.Label(dof_frame, textvariable=self.xyz_var, font=("Monospace", 10)).pack(anchor=tk.W)
        tk.Label(dof_frame, textvariable=self.rpy_var, font=("Monospace", 10)).pack(anchor=tk.W)

        btn_frame = tk.LabelFrame(main, text="Puck buttons", padx=6, pady=4)
        btn_frame.pack(fill=tk.X, pady=(0, 6))
        self.btn_var = tk.StringVar(value="Btn 0: ---  Btn 1: ---")
        tk.Label(btn_frame, textvariable=self.btn_var, font=("Monospace", 10)).pack(anchor=tk.W)

        hist_outer = tk.LabelFrame(main, text=f"Time history (last {HISTORY_LEN}, SpaceMouse mode on)", padx=6, pady=6)
        hist_outer.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        for i, lab in enumerate(AXIS_LABELS):
            row = tk.Frame(hist_outer)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{lab:5}", width=6, anchor=tk.W, font=("Monospace", 9)).pack(side=tk.LEFT)
            c = tk.Canvas(
                row,
                width=self._plot_w,
                height=self._plot_h,
                bg="#1a1a1a",
                highlightthickness=0,
            )
            c.pack(side=tk.LEFT, padx=4)
            self._plot_canvases.append(c)

        tk.Label(
            main,
            text="Terminal: d toggles left-pair SpaceMouse | q quits teleop",
            fg="gray",
            font=("", 9),
        ).pack(anchor=tk.W)

        self._poll()
        root.mainloop()

    def _poll(self) -> None:
        if self._stop.is_set() or self._root is None:
            return
        try:
            sample, eff, left_mode, connected, err, gt, gr, histories = self.shared.snapshot()
            with self.shared.lock:
                axis_copy = self.shared.axis_on[:]
            self._gui_batch = True
            try:
                for i in range(6):
                    want = 1 if axis_copy[i] else 0
                    if int(self._axis_vars[i].get()) != want:
                        self._axis_vars[i].set(want)
            finally:
                self._gui_batch = False

            if err:
                self.status_var.set(f"Error: {err}")
            elif connected:
                self.status_var.set(
                    "spacenavd OK | SpaceMouse LEFT pair: "
                    + ("ON" if left_mode else "OFF (press d in terminal)"),
                )
            else:
                self.status_var.set("SpaceMouse idle — press d in terminal to connect spnav")

            self.gain_var.set(
                f"trans={gt:.3f}  rot={gr:.3f}  "
                f"(limits trans [{GAIN_TRANS_MIN},{GAIN_TRANS_MAX}], rot [{GAIN_ROT_MIN},{GAIN_ROT_MAX}])",
            )

            self.xyz_var.set(
                f"x: {_format_val(eff[0])}  y: {_format_val(eff[1])}  z: {_format_val(eff[2])}",
            )
            self.rpy_var.set(
                f"roll: {_format_val(eff[3])}  pitch: {_format_val(eff[4])}  yaw: {_format_val(eff[5])}",
            )
            btns = sample.buttons
            b0 = btns[0] if len(btns) > 0 else 0
            b1 = btns[1] if len(btns) > 1 else 0
            self.btn_var.set(f"Btn 0: {'ON' if b0 else 'off'}  Btn 1: {'ON' if b1 else 'off'}")

            for i in range(6):
                _draw_history_canvas(
                    self._plot_canvases[i],
                    histories[i],
                    self._plot_w,
                    self._plot_h,
                    PLOT_COLORS[i],
                )
        except Exception:
            pass

        if self._root is not None:
            self._root.after(50, self._poll)
