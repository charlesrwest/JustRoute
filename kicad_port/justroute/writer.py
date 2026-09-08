"""Write routed Board geometry back into a .kicad_pcb file.

Exact inverse of the loader transform (routing_env/src/kicad_pcb.cpp):
    mm = origin + cell * resolution
where origin is KicadPcbInfo.border_min_{x,y} — the loader's minimum over
pad/outline extents AFTER the one-resolution margin fold-in, i.e. the same
value used to place pads on the grid. There is no y-flip: KiCad file y
already grows downward, and the loader maps it linearly.

Net codes on emitted copper are the FILE's net numbers, mapped through
original_net_ids[board_net_id]; board net ids are compacted build indices
and mean nothing to KiCad.

Emitted track width is 2*trace_half_width_cells*resolution — the design
half-width the model was configured with, not the rasterized (2hw+1)-cell
paint extent. The rasterized copper is a half-cell fatter per side than
what we emit, so every clearance our DRC verified holds a fortiori in
KiCad. Same for via size (2*via_radius_cells*resolution).
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class BoardFrame:
    """The file-frame facts the writer needs, from load_kicad_pcb_info()."""
    origin_x: float
    origin_y: float
    resolution: float
    layer_names: list           # Board layer index -> KiCad copper layer name
    original_net_ids: list      # Board net id -> KiCad file net number
    net_names: list = None      # v11 name-keyed files: net NAME per board net
    # The board's own design rules (mm, 0 = absent): emitting anything else
    # trips kicad-cli's track_width / via_diameter / drill_out_of_range checks.
    rule_track_width: float = 0.0
    rule_via_dia: float = 0.0
    rule_via_drill: float = 0.0
    # Per-net class geometry (0 = use default_*): a power-class net is emitted
    # at ITS width, not Default's. Always <= the modeled (max-class) swath, so
    # certified clearances hold a fortiori.
    default_track_width: float = 0.0
    default_via_dia: float = 0.0
    default_via_drill: float = 0.0
    net_widths: list = None
    net_via_dias: list = None
    net_via_drills: list = None

    @classmethod
    def from_info(cls, info: dict, resolution: float) -> "BoardFrame":
        return cls(origin_x=info["origin_x"], origin_y=info["origin_y"],
                   resolution=resolution,
                   layer_names=list(info["layer_names"]),
                   original_net_ids=list(info["original_net_ids"]),
                   net_names=list(info.get("net_names", [])) or None,
                   rule_track_width=info.get("rule_track_width", 0.0),
                   rule_via_dia=info.get("rule_via_dia", 0.0),
                   rule_via_drill=info.get("rule_via_drill", 0.0),
                   default_track_width=info.get("default_track_width", 0.0),
                   default_via_dia=info.get("default_via_dia", 0.0),
                   default_via_drill=info.get("default_via_drill", 0.0),
                   net_widths=list(info.get("net_widths", [])) or None,
                   net_via_dias=list(info.get("net_via_dias", [])) or None,
                   net_via_drills=list(info.get("net_via_drills", [])) or None)

    def mm(self, cx: int, cy: int) -> tuple:
        return (self.origin_x + cx * self.resolution,
                self.origin_y + cy * self.resolution)


@dataclass
class NetGeometry:
    net_id: int                                   # board net id
    runs: list = field(default_factory=list)      # (layer, x0, y0, x1, y1) in cells
    vias: list = field(default_factory=list)      # (x, y) in cells, deduped
    arcs: list = field(default_factory=list)      # (layer, x0, y0, mx, my, x1, y1) cells


def extract_geometry(board) -> list:
    """Collect straight cell runs and via sites from every net's committed paths.

    A path is a contiguous 8-direction cell sequence; a layer change between
    consecutive cells is a via (the router only transitions in place). Collinear
    steps merge into one run; zero-length runs are dropped (their copper is
    covered by the via barrel or the pad the cell sits on).

    Two hard-won rules from the Stage 16 exporter (rl/export_kicad.py):
    - Copper is emitted ONLY where the net still OWNS the cells in the board's
      owner array. Rip-up can leave stale cells in net.segments; drawing them
      fabricates copper that shorts a later commit — KiCad sees it, our DRC
      cannot (it checks the owner grid, not the emitted file).
    - A layer transition riding the net's own thru-hole pad barrel is NOT a
      via: the pad is already drilled, and stacking a via there trips KiCad's
      hole-to-hole check.
    """
    grid = board.grid()
    W, H = grid.width(), grid.height()
    owner = board.owner()
    layer_stride = H * W

    def owns(cell, tok):
        idx = cell.layer * layer_stride + cell.y * W + cell.x
        return idx < len(owner) and owner[idx] == tok

    out = []
    for net in board.nets():
        geo = NetGeometry(net_id=net.id)
        tok = net.id + 1
        via_seen = set()

        def flush(a, b):
            if a.x != b.x or a.y != b.y:
                geo.runs.append((a.layer, a.x, a.y, b.x, b.y))

        for path in net.segments:
            if len(path) < 2:
                continue
            run_start = None    # None = no open run (previous cell unowned)
            prev = None
            direction = None
            for cell in path:
                if not owns(cell, tok):
                    if run_start is not None:
                        flush(run_start, prev)
                    run_start = prev = None
                    direction = None
                    continue
                if prev is None:
                    run_start = prev = cell
                    direction = None
                    continue
                if cell.layer != prev.layer:
                    flush(run_start, prev)
                    if (board.thru_pad_token(prev.x, prev.y) != tok
                            and (prev.x, prev.y) not in via_seen):
                        via_seen.add((prev.x, prev.y))
                        geo.vias.append((prev.x, prev.y))
                    run_start = prev = cell
                    direction = None
                    continue
                d = (cell.x - prev.x, cell.y - prev.y)
                if d == (0, 0):
                    continue
                if direction is None:
                    direction = d
                elif d != direction:
                    flush(run_start, prev)
                    run_start = prev
                    direction = d
                prev = cell
            if run_start is not None:
                flush(run_start, prev)
        if geo.runs or geo.vias:
            out.append(geo)

    # If a smoothing pass has run (board.smooth_paths), replace the raw grid
    # staircase runs with the validated straight segments — same net ids, same
    # vias. The smoothed segments are DRC-clean by the same model the gates
    # certify (see Board::smooth_paths); the writer's terminal snapping still
    # applies to their endpoints (unchanged pad-touching cells).
    if getattr(board, "has_smoothed", lambda: False)():
        sp = board.smoothed_paths()
        nets = board.nets()
        by_id = {g.net_id: g for g in out}
        for k, segs in enumerate(sp):
            if not segs:
                continue
            g = by_id.get(nets[k].id)
            if g is None:
                continue
            g.runs = [(int(s[0]), s[1], s[2], s[3], s[4])
                      for s in segs if not s[7]]
            g.arcs = [(int(s[0]), s[1], s[2], s[5], s[6], s[3], s[4])
                      for s in segs if s[7]]   # (layer, x0,y0, mx,my, x1,y1)
    return out


def _fmt(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def _shape_sdf_cells(shape: dict, x: float, y: float) -> float:
    """Signed distance (cells) from point to a pad shape's copper boundary
    (< 0 inside). Mirrors PadShape::distance in the C++ core."""
    import math
    dx, dy = x - shape["cx"], y - shape["cy"]
    lx = dx * shape["cos_r"] + dy * shape["sin_r"]
    ly = -dx * shape["sin_r"] + dy * shape["cos_r"]
    hw, hh = shape["half_w"], shape["half_h"]
    if shape["oval"]:
        r = min(hw, hh)
        qx = max(0.0, abs(lx) - max(0.0, hw - r))
        qy = max(0.0, abs(ly) - max(0.0, hh - r))
        return math.hypot(qx, qy) - r
    qx, qy = abs(lx) - hw, abs(ly) - hh
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0)


def _terminal_snaps(board, frame: BoardFrame) -> dict:
    """(token, layer, pin_x, pin_y) -> (exact pad-center mm, shape dict).

    Pin cells are floor()-quantized from fractional pad centers, so a track
    ending at the pin cell can jut up to half a cell past its own pad copper —
    enough to shave a fine-pitch neighbor's clearance (seen: 0.186mm vs 0.2mm
    on 0.5mm-pitch USB-C pads). Terminals are snapped to the true center, the
    same convention interactive routers use. The shape rides along so emission
    can SKIP the snap when the grid endpoint already lands deep enough inside
    the pad copper — a needless stub toward the center can shave a tight
    neighbor instead (seen: 0.198mm vs 0.2 on a 0.49mm-pitch IC).
    """
    res = frame.resolution
    snaps = {}
    for shape in board.pad_shapes():
        cx, cy = shape["cx"], shape["cy"]
        px, py = int(cx), int(cy)   # the loader's floor()-quantized pin cell
        snaps[(shape["tok"], shape["layer"], px, py)] = (
            (frame.origin_x + cx * res, frame.origin_y + cy * res), shape)
    return snaps


def _shapes_by_layer(board) -> dict:
    """All pad shapes bucketed by layer (foreign-clearance checks at terminals)."""
    out = {}
    for shape in board.pad_shapes():
        out.setdefault(shape["layer"], []).append(shape)
    return out


def _foreign_margin_cells(shapes, tok, ax, ay, bx, by, hw_cells) -> float:
    """Worst clearance (cells, edge-to-edge) from segment a-b's copper to any
    foreign pad shape on the bucket. Large when nothing is near."""
    best = 1e9
    lo_x, hi_x = min(ax, bx), max(ax, bx)
    lo_y, hi_y = min(ay, by), max(ay, by)
    for sh in shapes:
        if sh["tok"] == tok:
            continue
        reach = max(sh["half_w"], sh["half_h"]) + hw_cells + 12.0
        if (sh["cx"] < lo_x - reach or sh["cx"] > hi_x + reach or
                sh["cy"] < lo_y - reach or sh["cy"] > hi_y + reach):
            continue
        for t in range(6):
            f = t / 5.0
            d = _shape_sdf_cells(sh, ax + (bx - ax) * f, ay + (by - ay) * f)
            best = min(best, d - hw_cells)
    return best


def emit_sexprs(geometry: list, frame: BoardFrame, board) -> str:
    """Render extracted geometry as segment/via s-expressions (KiCad v6+ syntax)."""
    res = frame.resolution
    # Default-class geometry, falling back to model values, then to the max rule.
    base_w = (frame.default_track_width or frame.rule_track_width
              or 2.0 * board.trace_half_width_cells() * res)
    base_via = (frame.default_via_dia or frame.rule_via_dia
                or 2.0 * board.via_radius_cells() * res)
    base_drill = frame.default_via_drill or frame.rule_via_drill or round(base_via / 2.0, 2)
    if base_via - base_drill < 0.2:
        base_via = round(base_drill + 0.2, 3)

    # KiCad's default minimum annular ring (board setup constraints): a via
    # whose (size - drill)/2 falls below it is flagged on every placement.
    # Some harvested boards carry STALE netclass via dims that violate it
    # (0.3/0.2 declared, human used other sizes); emitting them verbatim
    # fails annular_width. Keep the class drill, grow the size minimally.
    MIN_ANNULAR = 0.1

    def net_geom(net_id: int):
        w = base_w
        vs, vd = base_via, base_drill
        if frame.net_widths and net_id < len(frame.net_widths) and frame.net_widths[net_id]:
            w = frame.net_widths[net_id]
        if (frame.net_via_dias and net_id < len(frame.net_via_dias)
                and frame.net_via_dias[net_id]):
            vs = frame.net_via_dias[net_id]
            vd = (frame.net_via_drills[net_id]
                  if frame.net_via_drills and frame.net_via_drills[net_id]
                  else round(vs / 2.0, 2))
        if vs - vd < 2.0 * MIN_ANNULAR:
            vs = round(vd + 2.0 * MIN_ANNULAR, 3)
        return w, vs, vd

    top, bottom = frame.layer_names[0], frame.layer_names[-1]
    snaps = _terminal_snaps(board, frame)
    shapes_by_layer = _shapes_by_layer(board)

    def net_ref(net_id: int) -> str:
        # numeric-coded files (v4..v10) emit (net N); only v11 name-keyed
        # files (synthetic negative codes) emit (net "NAME") — net_names is
        # now populated for EVERY dialect (project-file matching), so the
        # code sign decides, not name presence.
        code = frame.original_net_ids[net_id]
        if code < 0 and frame.net_names and frame.net_names[net_id]:
            return '"' + frame.net_names[net_id] + '"'
        return str(code)

    lines = []
    for geo in geometry:
        file_net = net_ref(geo.net_id)
        track_w, via_size, via_drill = net_geom(geo.net_id)
        tok = geo.net_id + 1
        # Endpoint multiplicity: a cell shared by 2+ runs is a PASS-THROUGH or
        # junction — the net's copper already continues across the pad there,
        # so a center stub adds nothing but protrusion toward neighbors
        # (measured 10um graze from a junction stub on a 0.5mm GND track).
        end_count = {}
        for _l, _x0, _y0, _x1, _y1 in geo.runs:
            for k in ((_l, _x0, _y0), (_l, _x1, _y1)):
                end_count[k] = end_count.get(k, 0) + 1
        stubbed = set()
        for layer, x0, y0, x1, y1 in geo.runs:
            # Terminal snapping must neither TILT a long merged run (rotating an
            # 11.9mm segment by half a cell dips its middle into a bystander
            # pad's clearance — Robobuoy 0.1923mm) nor leave the terminal at the
            # quantized grid cell (juts past a fine-pitch pad — jackco 0.186mm).
            # Resolution: the FINAL CELL STEP absorbs the snap. Long runs are
            # pulled back one step to a grid point; a one-step stub goes from
            # there to the exact pad center. Both stub endpoints have safe
            # lateral positions, and the segment interpolates between them.
            run_cells = max(abs(x1 - x0), abs(y1 - y0))
            hw_cells = track_w * 0.5 / res
            def _snap_pt(key, gx, gy):
                ent = snaps.get(key)
                if ent is None:
                    return None
                center, shape = ent
                lshapes = shapes_by_layer.get(layer, ())
                # The stub only needs to reach the FIRST point (from the grid
                # endpoint toward the pad center) where the track end is fully
                # covered by own pad copper; running all the way to the center
                # drags copper toward tight neighbors (0.198 vs 0.2 on a
                # 0.49mm-pitch IC). Pads smaller than the track width keep the
                # full center snap (necking case: center is the best cover).
                own_depth = _shape_sdf_cells(shape, gx, gy)
                if own_depth <= -hw_cells:
                    return None      # already fully on the pad: no stub at all
                if own_depth < 0.0 and end_count.get((layer, gx, gy), 0) >= 2:
                    return None      # junction already touching own copper
                if (layer, gx, gy) in stubbed:
                    return None      # one stub per terminal cell is enough
                ccx, ccy = shape["cx"], shape["cy"]
                if _shape_sdf_cells(shape, ccx, ccy) > -hw_cells:
                    # Pad can't cover the track anywhere: NECK DOWN — the
                    # terminal piece is emitted at the pad's minor dimension
                    # (interactive-router convention). A full-width end here
                    # protrudes past the pad toward neighbors (measured 0.19
                    # vs 0.2 for a 0.5mm GND track on a 0.42mm pad).
                    neck = max(0.1, 2.0 * min(shape["half_w"],
                                              shape["half_h"]) * res)
                    return (center, min(neck, track_w))
                lo, hi = 0.0, 1.0    # t: 0 = grid endpoint, 1 = center
                for _ in range(12):
                    mid = 0.5 * (lo + hi)
                    px2 = gx + (ccx - gx) * mid
                    py2 = gy + (ccy - gy) * mid
                    if _shape_sdf_cells(shape, px2, py2) <= -hw_cells:
                        hi = mid
                    else:
                        lo = mid
                px2 = gx + (ccx - gx) * hi
                py2 = gy + (ccy - gy) * hi
                # Foreign-aware choice (measured: a minimal stub can STILL graze
                # a 0.72mm-away neighbor by 10um). Score the clipped stub vs the
                # no-stub option (valid when the endpoint center already touches
                # own copper — electrically connected) by their worst clearance
                # to foreign pads, and take the safer one.
                stub_m = _foreign_margin_cells(lshapes, tok, gx, gy, px2, py2,
                                               hw_cells)
                if own_depth < 0.0:
                    none_m = _foreign_margin_cells(lshapes, tok, gx, gy, gx, gy,
                                                   hw_cells)
                    if none_m > stub_m:
                        return None
                return ((frame.origin_x + px2 * res, frame.origin_y + py2 * res),
                        None)
            s0 = _snap_pt((tok, layer, x0, y0), x0, y0)
            s1 = _snap_pt((tok, layer, x1, y1), x1, y1)
            run_w0 = run_w1 = track_w        # per-end neck widths
            stubs = []                       # (ux, uy, vx, vy, width)
            if run_cells > 0:
                dx = (x1 - x0) // run_cells
                dy = (y1 - y0) // run_cells
            else:
                dx = dy = 0
            if s0 and run_cells > 2:
                x0, y0 = x0 + dx, y0 + dy
                stubs.append((*frame.mm(x0, y0), *s0[0], s0[1] or track_w))
                s0 = None
            if s1 and run_cells > 2:
                x1, y1 = x1 - dx, y1 - dy
                stubs.append((*frame.mm(x1, y1), *s1[0], s1[1] or track_w))
                s1 = None
            if s0:
                ax, ay = s0[0]
                if s0[1]:
                    run_w0 = s0[1]
            else:
                ax, ay = frame.mm(x0, y0)
            if s1:
                bx, by = s1[0]
                if s1[1]:
                    run_w1 = s1[1]
            else:
                bx, by = frame.mm(x1, y1)
            # a short run absorbing a NECKED snap is emitted at the neck width
            run_w = min(run_w0, run_w1)
            for ux, uy, vx, vy, sw in stubs:
                stubbed.add((layer, round((ux - frame.origin_x) / res),
                             round((uy - frame.origin_y) / res)))
                if abs(ux - vx) > 1e-6 or abs(uy - vy) > 1e-6:
                    lines.append(
                        f"  (segment (start {_fmt(ux)} {_fmt(uy)}) (end {_fmt(vx)} {_fmt(vy)}) "
                        f"(width {_fmt(sw)}) (layer \"{frame.layer_names[layer]}\") (net {file_net}))")
            lines.append(
                f"  (segment (start {_fmt(ax)} {_fmt(ay)}) (end {_fmt(bx)} {_fmt(by)}) "
                f"(width {_fmt(run_w)}) (layer \"{frame.layer_names[layer]}\") (net {file_net}))")
        # Corner fillets (smooth-curve option): KiCad-native circular arc tracks.
        # Endpoints are tangent points on the straight runs (never pads), so no
        # terminal snapping applies. The (mid) point sits on the arc.
        for layer, x0, y0, mx, my, x1, y1 in geo.arcs:
            ax, ay = frame.mm(x0, y0)
            mmx, mmy = frame.mm(mx, my)
            bx, by = frame.mm(x1, y1)
            lines.append(
                f"  (arc (start {_fmt(ax)} {_fmt(ay)}) (mid {_fmt(mmx)} {_fmt(mmy)}) "
                f"(end {_fmt(bx)} {_fmt(by)}) (width {_fmt(track_w)}) "
                f"(layer \"{frame.layer_names[layer]}\") (net {file_net}))")
        for x, y in geo.vias:
            vx, vy = frame.mm(x, y)
            lines.append(
                f"  (via (at {_fmt(vx)} {_fmt(vy)}) (size {_fmt(via_size)}) "
                f"(drill {_fmt(via_drill)}) (layers \"{top}\" \"{bottom}\") (net {file_net}))")
    return "\n".join(lines)


def splice(input_text: str, block: str) -> str:
    """Insert the emitted block before the closing paren of (kicad_pcb ...)."""
    if not block:
        return input_text
    pos = input_text.rindex(")")
    return input_text[:pos] + block + "\n" + input_text[pos:]


def write_routed(input_text: str, board, frame: BoardFrame) -> str:
    """Full pipeline: routed board + original file text -> routed file text."""
    geometry = extract_geometry(board)
    return splice(input_text, emit_sexprs(geometry, frame, board))
