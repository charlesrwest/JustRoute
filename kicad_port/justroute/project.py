"""KiCad project-file (.kicad_pro) netclass support.

KiCad 6+ keeps netclass DEFINITIONS and ASSIGNMENTS in the sidecar
`<project>.kicad_pro` (JSON), not in the `.kicad_pcb` — so most modern
boards look single-class when only the pcb file is read. This module
parses the sidecar and produces per-net rules for the physics tiers and
the writer.

Supported assignment dialects:
- KiCad 6: each class carries a `"nets": [names...]` array;
- KiCad 7+: `net_settings.netclass_patterns` = [{netclass, pattern}]
  with KiCad wildcard patterns (fnmatch-style * and ?);
- KiCad 8/9: `net_settings.netclass_assignments` = {net: [classes...]}.

Precedence per net: explicit assignment > first matching pattern > v6
nets array > Default. Only the fields the router uses are read:
clearance, track_width, via_diameter, via_drill.
"""

from __future__ import annotations

import fnmatch
import json
from pathlib import Path


def find_project(pcb_path: Path) -> Path | None:
    """The sidecar project file for a board, if present."""
    cand = pcb_path.with_suffix(".kicad_pro")
    return cand if cand.exists() else None


def load_classes(pro_path: Path) -> tuple[dict, list, dict]:
    """Parse `.kicad_pro` -> (classes, patterns, assignments).

    classes: {name: {clearance, track_width, via_dia, via_drill}} (mm; only
    keys present in the file appear). patterns: ordered [(class, pattern)].
    assignments: {net_name: class_name}.
    """
    data = json.loads(pro_path.read_text(encoding="utf-8", errors="replace"))
    ns = data.get("net_settings", {}) or {}
    classes: dict = {}
    patterns: list = []
    assignments: dict = {}
    for cls in ns.get("classes", []) or []:
        name = cls.get("name")
        if not name:
            continue
        rules = {}
        for src, dst in (("clearance", "clearance"), ("track_width", "track_width"),
                         ("via_diameter", "via_dia"), ("via_drill", "via_drill")):
            v = cls.get(src)
            if isinstance(v, (int, float)) and v > 0:
                rules[dst] = float(v)
        classes[name] = rules
        for net in cls.get("nets", []) or []:      # v6 style
            assignments.setdefault(str(net), name)
    for pat in ns.get("netclass_patterns", []) or []:
        c, p = pat.get("netclass"), pat.get("pattern")
        if c and p is not None:
            patterns.append((str(c), str(p)))
    na = ns.get("netclass_assignments", {}) or {}
    for net, cls in na.items():
        if isinstance(cls, list):
            if cls:
                assignments[str(net)] = str(cls[0])
        elif cls:
            assignments[str(net)] = str(cls)
    return classes, patterns, assignments


def class_of(net_name: str, classes: dict, patterns: list, assignments: dict) -> str:
    if net_name in assignments and assignments[net_name] in classes:
        return assignments[net_name]
    for cls, pat in patterns:
        if cls in classes and fnmatch.fnmatchcase(net_name, pat):
            return cls
    return "Default"


def net_rules(net_names: list, pro_path: Path) -> tuple[dict, list]:
    """Per-board-net rules from the project file.

    Returns (default_rules, per_net) where per_net[i] is the rules dict for
    board net index i (== the Default rules when unmatched/unnamed).
    """
    classes, patterns, assignments = load_classes(pro_path)
    default = classes.get("Default", {})
    per_net = []
    for nm in net_names:
        if not nm:
            per_net.append(default)
            continue
        cls = class_of(nm, classes, patterns, assignments)
        rules = dict(default)
        rules.update(classes.get(cls, {}))
        per_net.append(rules)
    return default, per_net


def apply_project_rules(env, board, frame, info: dict, pro_path: Path,
                        resolution: float, log=lambda m: None) -> int:
    """Apply sidecar netclass rules: physics tiers + emission geometry.

    Mirrors the loader's own netclass handling: tier clearance is ceiled with
    the diagonal-safe snap margin; the global via radius is raised to the
    class max (via machinery is max-safe, sizes are emitted per net).
    Returns the number of nets that got non-Default rules.
    """
    import math

    default, per_net = net_rules(info.get("net_names", []), pro_path)
    if not per_net:
        return 0
    snap = 0.7072 * resolution
    base_clr = default.get("clearance", 0.0) or (frame.rule_clearance or 0.2)
    base_tw = default.get("track_width", 0.0) or (frame.default_track_width
                                                  or frame.rule_track_width or 0.2)
    if default.get("track_width"):
        frame.default_track_width = default["track_width"]
    if default.get("via_dia"):
        frame.default_via_dia = default["via_dia"]
    if default.get("via_drill"):
        frame.default_via_drill = default["via_drill"]

    n = len(per_net)
    widths = list(frame.net_widths or [0.0] * n)
    vdias = list(frame.net_via_dias or [0.0] * n)
    vdrills = list(frame.net_via_drills or [0.0] * n)
    while len(widths) < n:
        widths.append(0.0)
    while len(vdias) < n:
        vdias.append(0.0)
    while len(vdrills) < n:
        vdrills.append(0.0)

    max_via = 0.0
    changed = 0
    for i, rules in enumerate(per_net):
        tw = rules.get("track_width", base_tw)
        clr = rules.get("clearance", base_clr)
        if rules.get("via_dia"):
            vdias[i] = rules["via_dia"]
            vdrills[i] = rules.get("via_drill", 0.0)
            max_via = max(max_via, rules["via_dia"])
        if rules.get("track_width"):
            widths[i] = rules["track_width"]
        if tw != base_tw or clr != base_clr:
            changed += 1
        clr_cells = max(1.0, math.ceil((clr + snap) / resolution - 1e-9))
        board.set_net_class_physics(i, tw * 0.5 / resolution, clr_cells)

    frame.net_widths = widths
    frame.net_via_dias = vdias
    frame.net_via_drills = vdrills
    if max_via > 0.0:
        cur = board.via_radius_cells()
        board.set_via_radius_cells(max(cur, max_via * 0.5 / resolution))
    if changed:
        log(f"project netclasses: {changed} nets with non-Default rules "
            f"({pro_path.name})")
    return changed
