"""Auto-pour generation: give a from-scratch board the plane a human would.

A high-fanout net (GND on nearly every board) is not routed as tracks by any
sane designer — it gets a copper pour. A from-scratch board (no zones drawn
yet) therefore fails its biggest net through no fault of the router. Instead
of deferring-and-telling ("pour it yourself"), generate the pour: inject a
board-covering zone for the plane net into the board text BEFORE loading.

Everything downstream is existing machinery: the loader's zone-glue marks the
net pour-fed (skip_poured), the router routes the remaining nets normally,
the writer preserves the injected zone in the output, and kicad-cli DRC with
--refill-zones — the external judge, not our model — verifies the pour truly
connects the net (measured: MANUAL's 39-pad GND, 38 unconnected -> 0 from the
zone alone; through-hole barrels reach a back-layer pour without stitching).

v1 scope: ONE zone, the largest-fanout net above `min_pins`, on the back
copper layer, only when the file has no copper zones at all (a board with any
zone reflects the designer's own plane plan — never second-guess it).
"""

from __future__ import annotations

import re
from collections import Counter


def _pad_counts(text: str) -> Counter:
    """Pads per net number (net 0 = unconnected, excluded)."""
    c = Counter(int(m.group(1)) for m in re.finditer(
        r'\(pad\b[^{}]*?\(net\s+(\d+)[\s)]', text))
    c.pop(0, None)
    return c


def _net_names(text: str) -> dict:
    return {int(m.group(1)): m.group(2) for m in re.finditer(
        r'\(net\s+(\d+)\s+"?([^")]*)"?\)', text)}


def _has_copper_zone(text: str) -> bool:
    """Any non-keepout zone (a keepout is a rule area, not a pour)."""
    for m in re.finditer(r'\(zone\b', text):
        window = text[m.start():m.start() + 800]
        if "(keepout" not in window:
            return True
    return False


def _board_bbox(text: str):
    """(x0, y0, x1, y1) in file mm from Edge.Cuts graphics; pads fallback."""
    xs, ys = [], []
    for m in re.finditer(
            r'\((?:gr_line|gr_rect)\s+\(start\s+([-\d.]+)\s+([-\d.]+)\)\s*'
            r'\(end\s+([-\d.]+)\s+([-\d.]+)\)[^()]*(?:\([^)]*\))*?\s*'
            r'\(layer\s+"?Edge\.Cuts"?\)', text):
        xs += [float(m.group(1)), float(m.group(3))]
        ys += [float(m.group(2)), float(m.group(4))]
    if not xs:
        for m in re.finditer(r'\(footprint\b|\(module\b', text):
            at = re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)', text[m.start():m.start() + 400])
            if at:
                xs.append(float(at.group(1)))
                ys.append(float(at.group(2)))
        if not xs:
            return None
        # pad-position fallback: pad some margin around the parts
        return (min(xs) - 5, min(ys) - 5, max(xs) + 5, max(ys) + 5)
    return (min(xs), min(ys), max(xs), max(ys))


def _quoted_dialect(text: str) -> bool:
    """v6+ files quote layer names; v4/v5 don't."""
    return '(layer "' in text or '(layers "' in text


def zone_sexpr(net_num: int, net_name: str, bbox, layer: str, quoted: bool) -> str:
    x0, y0, x1, y1 = bbox
    q = '"' if quoted else ''
    nm = net_name if not quoted else net_name  # name text identical; quoting differs
    stamp = '(uuid "00000000-0000-0000-0000-00004a520001")' if quoted else "(tstamp 0)"
    return (
        f'  (zone (net {net_num}) (net_name {q}{nm}{q}) (layer {q}{layer}{q}) {stamp}\n'
        f'    (hatch edge 0.508)\n'
        f'    (connect_pads (clearance 0.508))\n'
        f'    (min_thickness 0.254)\n'
        f'    (fill yes (thermal_gap 0.508) (thermal_bridge_width 0.508))\n'
        f'    (polygon\n'
        f'      (pts\n'
        f'        (xy {x0:.3f} {y0:.3f}) (xy {x1:.3f} {y0:.3f}) '
        f'(xy {x1:.3f} {y1:.3f}) (xy {x0:.3f} {y1:.3f})\n'
        f'      )\n'
        f'    )\n'
        f'  )\n'
    )


def plane_candidate(text: str, min_pins: int = 30, target_name: str | None = None):
    """(net_num, name, pad_count) of the pour target, or None.

    Only when the file has NO copper zone (an existing zone = the designer's
    own plane plan; adding ours would fight it). With `target_name` the
    candidate is that specific net (failure-driven trigger: the router already
    proved it can't route it as tracks) instead of the largest-fanout one."""
    if _has_copper_zone(text):
        return None
    counts = _pad_counts(text)
    if not counts:
        return None
    names = _net_names(text)
    if target_name is not None:
        matches = [n for n, nm in names.items() if nm == target_name]
        if not matches or counts.get(matches[0], 0) < min_pins:
            return None
        net = matches[0]
        return (net, target_name, counts[net])
    net, pads = counts.most_common(1)[0]
    if pads < min_pins:
        return None
    name = names.get(net, f"net#{net}")
    return (net, name, pads)


def inject_pour(text: str, min_pins: int = 30, layer: str = "B.Cu",
                target_name: str | None = None):
    """Inject a generated pour for the plane candidate.

    Returns (new_text, {"net", "name", "pads", "layer"}) or (text, None) when
    no candidate (existing zones / no big net / no bbox)."""
    cand = plane_candidate(text, min_pins, target_name)
    if cand is None:
        return text, None
    bbox = _board_bbox(text)
    if bbox is None:
        return text, None
    net, name, pads = cand
    zone = zone_sexpr(net, name, bbox, layer, _quoted_dialect(text))
    body = text.rstrip()
    j = body.rfind(')')
    if j < 0:
        return text, None
    new_text = body[:j] + zone + ')\n'
    return new_text, {"net": net, "name": name, "pads": pads, "layer": layer}


# --------------------------------------------------------------------------
# Stitching vias: pads the pour can't reach (SMD pads on the far layer) show
# up in kicad-cli's unconnected_items WITH exact positions — the judge tells
# us where the gaps are, so no footprint-rotation math is ever needed. A via
# dropped at the pad position lands on the pad (same net: no clearance rule
# between a via and its own pad) and its barrel reaches the pour layer.
# --------------------------------------------------------------------------

def plane_unconnected_positions(drc_report: dict, plane_name: str):
    """(x, y) mm of pads kicad-cli says are still unconnected on the plane net."""
    out = []
    for u in drc_report.get("unconnected_items", []):
        # the airwire's description names both anchors; net name appears in it
        desc = " ".join(str(it.get("description", "")) for it in u.get("items", []))
        if f"[{plane_name}]" not in desc and f" {plane_name} " not in desc \
                and f'"{plane_name}"' not in desc and plane_name not in desc:
            continue
        for it in u.get("items", []):
            d = str(it.get("description", ""))
            pos = it.get("pos") or {}
            if d.startswith("Pad ") and "x" in pos:
                out.append((float(pos["x"]), float(pos["y"])))
    # dedup (same pad can anchor several airwires)
    seen = set()
    uniq = []
    for x, y in out:
        k = (round(x, 3), round(y, 3))
        if k not in seen:
            seen.add(k)
            uniq.append((x, y))
    return uniq


def via_sexpr(x: float, y: float, net_num: int, quoted: bool,
              size: float = 0.6, drill: float = 0.3) -> str:
    q = '"' if quoted else ''
    return (f'  (via (at {x:.4f} {y:.4f}) (size {size}) (drill {drill}) '
            f'(layers {q}F.Cu{q} {q}B.Cu{q}) (net {net_num}))\n')


def add_stitch_vias(text: str, positions, net_num: int,
                    size: float = 0.6, drill: float = 0.3):
    """Append a stitch via at each position. Returns new text."""
    if not positions:
        return text
    quoted = _quoted_dialect(text)
    body = text.rstrip()
    j = body.rfind(')')
    if j < 0:
        return text
    vias = "".join(via_sexpr(x, y, net_num, quoted, size, drill)
                   for x, y in positions)
    return body[:j] + vias + ')\n'
