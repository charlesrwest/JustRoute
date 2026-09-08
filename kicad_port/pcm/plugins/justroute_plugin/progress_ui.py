"""Tiny progress window for the JustRoute KiCad action.

The IPC plugin is an external process, so it owns its own (stdlib-only)
window: a status line, a scrolling log, elapsed time, and a Cancel button.
Cancel is safe by design — the engine checkpoints its best state between
passes, so stopping keeps everything routed so far.

Falls back to plain console logging when tkinter or a display is
unavailable (headless/CI); the caller never has to care.
"""

from __future__ import annotations

import os
import queue
import threading
import time


class ConsoleProgress:
    """No-window fallback: same interface, prints to stdout."""

    def __init__(self):
        self.cancelled = False
        self.on_cancel = None

    def log(self, msg: str) -> None:
        print(f"[JustRoute] {msg}", flush=True)

    def should_stop(self) -> bool:
        return self.cancelled

    def run(self, work) -> None:
        work()

    def close(self, final_msg: str = "") -> None:
        if final_msg:
            self.log(final_msg)


class TkProgress:
    """Progress window; routing runs in a worker thread while tk owns the UI."""

    def __init__(self, title: str = "JustRoute"):
        import tkinter as tk
        from tkinter import scrolledtext

        self._tk = tk
        self.cancelled = False
        self.on_cancel = None      # extra callback fired on Cancel (thread-safe)
        self.poller = None         # optional () -> (done, total) pass progress
        self._q: queue.Queue = queue.Queue()
        self._t0 = time.monotonic()
        self._done = False

        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("560x340")
        self.root.minsize(420, 240)

        self.status = tk.StringVar(value="starting…")
        bar = tk.Frame(self.root)
        bar.pack(fill="x", padx=8, pady=(8, 4))
        tk.Label(bar, textvariable=self.status, anchor="w").pack(
            side="left", fill="x", expand=True)
        self.elapsed = tk.StringVar(value="0s")
        tk.Label(bar, textvariable=self.elapsed, width=8, anchor="e").pack(
            side="right")

        # progress bar as a plain Canvas rectangle — theme-proof (ttk bars
        # collapse to a hairline on the native themes here). Before a total
        # is known it shows a bouncing chunk; after, a determinate fill.
        self._bar_h = 16
        self.pbar = tk.Canvas(self.root, height=self._bar_h, highlightthickness=1,
                              highlightbackground="#9a9a9a", bg="#d0d0d0")
        self.pbar.pack(fill="x", padx=8, pady=(0, 4))
        self._bar_fill = self.pbar.create_rectangle(0, 0, 0, self._bar_h,
                                                    fill="#2d7d46", width=0)
        self._bounce = 0.0

        self.text = scrolledtext.ScrolledText(
            self.root, height=12, state="disabled", font=("monospace", 9))
        self.text.pack(fill="both", expand=True, padx=8, pady=4)

        btns = tk.Frame(self.root)
        btns.pack(fill="x", padx=8, pady=(4, 8))
        self.btn = tk.Button(btns, text="Cancel (keep best so far)",
                             command=self._cancel)
        self.btn.pack(side="right")
        self.root.protocol("WM_DELETE_WINDOW", self._cancel)

        t = os.environ.get("JUSTROUTE_TEST_CANCEL_AFTER")  # test hook
        if t:
            self.root.after(int(float(t) * 1000), self._cancel)
        if os.environ.get("JUSTROUTE_TEST_AUTOCLOSE"):     # test hook
            def _autoclose():
                if self._done:
                    self.root.destroy()
                else:
                    self.root.after(2000, _autoclose)
            self.root.after(2000, _autoclose)

    def _draw_bar(self, done: int, total: int) -> None:
        w = max(1, self.pbar.winfo_width())
        if total > 0:
            frac = max(0.0, min(1.0, done / total))
            self.pbar.coords(self._bar_fill, 0, 0, int(w * frac), self._bar_h)
        else:
            # indeterminate: a chunk sliding back and forth
            self._bounce = (self._bounce + 0.06) % 2.0
            b = self._bounce if self._bounce <= 1.0 else 2.0 - self._bounce
            cw = w * 0.25
            x0 = (w - cw) * b
            self.pbar.coords(self._bar_fill, int(x0), 0, int(x0 + cw), self._bar_h)

    def _cancel(self) -> None:
        if self._done:
            self.root.destroy()
            return
        self.cancelled = True
        self.status.set("stopping — keeping everything routed so far…")
        self.btn.configure(state="disabled")
        if self.on_cancel is not None:
            try:
                self.on_cancel()
            except Exception:
                pass

    # --- interface used by the routing thread (thread-safe via queue) ---
    def log(self, msg: str) -> None:
        print(f"[JustRoute] {msg}", flush=True)
        self._q.put(msg)

    def should_stop(self) -> bool:
        return self.cancelled

    # --- main-thread pump ---
    def _pump(self) -> None:
        try:
            while True:
                msg = self._q.get_nowait()
                self.status.set(msg)
                self.text.configure(state="normal")
                self.text.insert("end", msg + "\n")
                self.text.see("end")
                self.text.configure(state="disabled")
        except queue.Empty:
            pass
        t = f"{time.monotonic() - self._t0:.0f}s"
        done, total = 0, 0
        if self.poller is not None:
            try:
                done, total = self.poller()
                if total > 0 and done <= total:
                    t = f"{done}/{total} · {t}"
            except Exception:
                self.poller = None
        try:
            self._draw_bar(done, total)
        except Exception:
            pass
        self.elapsed.set(t)
        if not self._done:
            self.root.after(120, self._pump)

    def run(self, work) -> None:
        """Run `work()` in a thread while the window pumps; returns when done."""
        exc: list = []

        def _worker():
            try:
                work()
            except Exception as e:      # surfaced after mainloop ends
                exc.append(e)
            finally:
                self._done = True
                try:
                    self.root.after(0, self.root.quit)
                except Exception:
                    pass

        threading.Thread(target=_worker, daemon=True).start()
        self._pump()
        self.root.mainloop()
        if exc:
            try:
                self.root.destroy()
            except Exception:
                pass
            raise exc[0]

    def close(self, final_msg: str = "") -> None:
        """Show the final result and switch Cancel -> Close (window stays up
        until the user dismisses it, so the summary is actually readable)."""
        self._done = True
        try:
            try:
                self._draw_bar(1, 1)   # full = finished
            except Exception:
                pass
            if final_msg:
                self.text.configure(state="normal")
                self.text.insert("end", final_msg + "\n")
                self.text.see("end")
                self.text.configure(state="disabled")
                self.status.set(final_msg.splitlines()[0])
            self.btn.configure(text="Close", state="normal",
                               command=self.root.destroy)
            self.root.mainloop()
        except Exception:
            pass


def make_progress(title: str = "JustRoute"):
    try:
        return TkProgress(title)
    except Exception:
        return ConsoleProgress()
