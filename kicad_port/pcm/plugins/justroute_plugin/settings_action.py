"""JustRoute settings action: edit justroute.json without a text editor.

A small stdlib-only (tkinter) form for the persistent 10%: routing budget,
effort, pour-fed net handling, and grid resolution. Saves `justroute.json`
next to the open board's project, which both the "Route board" action and
the `JustRoute` CLI read.

Test hooks (unattended validation): JUSTROUTE_TEST_SETTINGS_SAVE=1 saves
and closes ~2s after the window opens.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

DEFAULTS = {
    "budget_s": 0,            # 0 = auto (1s per net, clamped 60..600)
    "effort": "full",
    "route_poured_nets": False,
    "resolution": 0,          # 0 = auto (0.05mm, 0.025mm for fine pitch)
    "fillet_radius_mm": 0,    # 0 = square corners; >0 = rounded (arc fillets)
}


def _board_dir() -> Path | None:
    try:
        from kipy import KiCad  # type: ignore
        board = KiCad().get_board()
        doc = board.document
        proj = getattr(doc, "project", None)
        pdir = getattr(proj, "path", "") if proj is not None else ""
        if pdir:
            return Path(pdir)
        p = Path(board.name).resolve()
        return p.parent if p.parent.is_dir() else None
    except Exception:
        return None


def run_dialog(cfg_path: Path) -> int:
    import tkinter as tk

    cfg = dict(DEFAULTS)
    try:
        cfg.update(json.loads(cfg_path.read_text(encoding="utf-8")))
    except Exception:
        pass

    root = tk.Tk()
    root.title("JustRoute settings")
    root.minsize(420, 0)
    frm = tk.Frame(root, padx=12, pady=10)
    frm.pack(fill="both", expand=True)
    frm.columnconfigure(1, weight=1)

    def row(r, label, hint):
        tk.Label(frm, text=label, anchor="w").grid(
            row=r, column=0, sticky="w", pady=(6, 0))
        tk.Label(frm, text=hint, anchor="w", fg="#777",
                 font=("TkDefaultFont", 8)).grid(
            row=r + 1, column=0, columnspan=2, sticky="w")

    budget = tk.StringVar(value=str(cfg.get("budget_s") or 0))
    row(0, "Routing budget (seconds)", "0 = automatic: 1s per net, 60..600")
    tk.Entry(frm, textvariable=budget, width=8).grid(
        row=0, column=1, sticky="e", pady=(6, 0))

    effort = tk.StringVar(value=str(cfg.get("effort", "full")))
    row(2, "Effort", "full: keeps improving until the budget runs out; "
                     "fast: one quick pass")
    eb = tk.Frame(frm)
    eb.grid(row=2, column=1, sticky="e", pady=(6, 0))
    for v, lbl in (("full", "Full"), ("fast", "Fast")):
        tk.Radiobutton(eb, text=lbl, value=v, variable=effort).pack(side="left")

    poured = tk.BooleanVar(value=bool(cfg.get("route_poured_nets", False)))
    row(4, "Route pour-fed nets", "off: nets owned by a copper pour are left "
                                  "to their zones (recommended)")
    tk.Checkbutton(frm, variable=poured).grid(
        row=4, column=1, sticky="e", pady=(6, 0))

    res = tk.StringVar(value=str(cfg.get("resolution") or 0))
    row(6, "Grid resolution (mm)", "0 = automatic: 0.05, or 0.025 for "
                                   "fine-pitch boards")
    rb = tk.Frame(frm)
    rb.grid(row=6, column=1, sticky="e", pady=(6, 0))
    for v, lbl in (("0", "Auto"), ("0.05", "0.05"), ("0.025", "0.025")):
        tk.Radiobutton(rb, text=lbl, value=v, variable=res).pack(side="left")

    fillet = tk.StringVar(value=str(cfg.get("fillet_radius_mm") or 0))
    row(8, "Corner rounding (mm)", "0 = square corners; a radius rounds "
                                   "corners with arcs (validated DRC-clean)")
    fb = tk.Frame(frm)
    fb.grid(row=8, column=1, sticky="e", pady=(6, 0))
    for v, lbl in (("0", "Off"), ("0.3", "0.3"), ("0.5", "0.5"), ("1.0", "1.0")):
        tk.Radiobutton(fb, text=lbl, value=v, variable=fillet).pack(side="left")

    status = tk.StringVar(value=str(cfg_path))
    tk.Label(frm, textvariable=status, anchor="w", fg="#777",
             font=("TkDefaultFont", 8), wraplength=400, justify="left").grid(
        row=10, column=0, columnspan=2, sticky="we", pady=(10, 0))

    saved = []

    def save():
        out = {}
        try:
            b = float(budget.get() or 0)
            if b > 0:
                out["budget_s"] = int(b) if b == int(b) else b
        except ValueError:
            status.set("budget must be a number")
            return
        if effort.get() != "full":
            out["effort"] = effort.get()
        if poured.get():
            out["route_poured_nets"] = True
        try:
            r = float(res.get() or 0)
            if r > 0:
                out["resolution"] = r
        except ValueError:
            status.set("resolution must be a number")
            return
        try:
            fr = float(fillet.get() or 0)
            if fr > 0:
                out["fillet_radius_mm"] = fr
        except ValueError:
            status.set("corner rounding must be a number")
            return
        # only non-default keys are written: an empty file means "all auto"
        if out:
            cfg_path.write_text(json.dumps(out, indent=2) + "\n",
                                encoding="utf-8")
        elif cfg_path.exists():
            cfg_path.unlink()
        saved.append(out)
        root.destroy()

    btns = tk.Frame(frm)
    btns.grid(row=11, column=0, columnspan=2, sticky="e", pady=(10, 0))
    tk.Button(btns, text="Cancel", command=root.destroy).pack(
        side="right", padx=(6, 0))
    tk.Button(btns, text="Save", command=save, default="active").pack(
        side="right")
    root.bind("<Return>", lambda e: save())
    root.bind("<Escape>", lambda e: root.destroy())

    if os.environ.get("JUSTROUTE_TEST_SETTINGS_SAVE"):   # test hook
        root.after(2000, save)

    root.mainloop()
    if saved:
        print(f"[JustRoute] settings saved to {cfg_path}"
              if saved[0] or cfg_path.exists() else
              "[JustRoute] settings reset to automatic defaults")
    return 0


def main() -> int:
    d = _board_dir()
    if d is None:
        print("[JustRoute] could not locate the project directory — "
              "open a saved board in the PCB editor and retry.")
        return 1
    return run_dialog(d / "justroute.json")


if __name__ == "__main__":
    sys.exit(main())
