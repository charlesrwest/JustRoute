"""JustRoute IPC action: route the open board's unrouted nets, undoably.

Flow (KiCad >= 9 IPC API via kipy; KICAD_API_SOCKET/KICAD_API_TOKEN are set
by KiCad when it launches this process):
  1. save the open board (the engine reads the .kicad_pcb file — the loader
     treats nets with existing copper as fixed pre-routed obstacles);
  2. route headlessly with the justroute engine (retention + blame reorder +
     negotiation finisher, wall budget);
  3. write the routed geometry back through board.create_items() inside ONE
     commit, so the whole routing job is a single undo step.

If the IPC write-back fails (kipy API drift — step 3 is the only part that
cannot be exercised without a live GUI), the routed board is written next to
the original as <name>.routed.kicad_pcb and a message says so: the file
path is the always-works fallback.

Configuration (optional, environment):
  JUSTROUTE_LIB    path containing the built routing_env module and the
                  justroute package (defaults to this plugin's directory —
                  a PCM build ships them alongside this script)
  JUSTROUTE_BUDGET wall budget in seconds (default 120)
"""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

_LIB = os.environ.get("JUSTROUTE_LIB", str(Path(__file__).resolve().parent))
for p in (_LIB, str(Path(_LIB) / "build-opt")):
    if p not in sys.path:
        sys.path.insert(0, p)


def _route_file_to(board_path: Path, out_path: Path, budget_s: float,
                   only_nets: set | None = None,
                   log=lambda m: print(f"[JustRoute] {m}"),
                   should_stop=lambda: False, route_poured: bool = False,
                   fillet_mm: float = 0.0):
    """Headless route of the saved board file. Returns (frame, board, result).

    Uses the sidecar .kicad_pro (always present in a live KiCad session) for
    netclass widths/clearances, auto grid resolution, and — when `only_nets`
    (net NAMES) is given — routes just those nets by treating every other
    unrouted net as untouchable (its pads stay obstacles via net order and a
    restricted engine pass).
    """
    from justroute.cli import (route_file, _load_core, _tuned_protocol,
                              FINE_PITCH_MM, COARSE_RES, FINE_RES)
    from justroute.writer import BoardFrame, extract_geometry
    from justroute import project as _project

    from justroute.cli import friendly_load_error
    try:
        rc = _load_core()
    except Exception as e:                      # CoreUnavailable: already actionable
        log(str(e))
        return None, None, None
    text = board_path.read_text(encoding="utf-8", errors="replace")
    skip = not route_poured
    env = rc.RoutingEnv(2, 10, 10, COARSE_RES, 5.0, 1.0, 0.1)
    try:
        probe = env.load_kicad_pcb_info(text, COARSE_RES, skip_poured=skip)
    except RuntimeError as e:
        if "all nets already routed" in str(e):
            log("every net is already routed — nothing to do")
            return None, None, None
        friendly = friendly_load_error(e)
        if friendly:
            log(friendly)
            return None, None, None
        raise
    res = (FINE_RES if probe.get("min_pad_spacing", 99.0) < FINE_PITCH_MM
           else COARSE_RES)
    env = rc.RoutingEnv(2, 10, 10, res, 5.0, 1.0, 0.1)
    info = env.load_kicad_pcb_info(text, res, skip_poured=skip)
    frame = BoardFrame.from_info(info, res)
    def plural(n, one, many):
        return one if n == 1 else many.format(n=n)

    poured = list(info.get("pour_fed_nets", []) or [])
    if poured:
        log(plural(len(poured), "1 pour-fed net left to its zone: ",
                   "{n} pour-fed nets left to their zones: ")
            + ", ".join(poured[:8]) + (" …" if len(poured) > 8 else "")
            + '  (set "route_poured_nets": true in justroute.json to route them)')
    # pre_routed_nets counts the pour-skipped ones too — report them apart
    pre = int(info.get("pre_routed_nets", 0) or 0) - len(poured)
    part = int(info.get("partial_nets", 0) or 0)
    if pre > 0:
        log(plural(pre, "1 net is already fully routed — leaving it",
                   "{n} nets are already fully routed — leaving them")
            + f" untouched, routing the remaining {len(env.board().nets())}")
    if part:
        log(plural(part, "1 net has partial copper — completing it",
                   "{n} nets have partial copper — completing them")
            + " (existing stubs stay in place)")
    pro = _project.find_project(board_path)
    if pro is not None:
        try:
            _project.apply_project_rules(env, env.board(), frame, info, pro,
                                         res, log=log)
        except Exception as e:
            log(f"project file ignored ({type(e).__name__}: {e})")
    _tuned_protocol(env)

    if only_nets:
        # True selection scope: route ONLY the selected nets (the semantics a
        # user expects from "route selection"). Everything else stays exactly
        # as it is — other nets' pads remain hard obstacles automatically.
        from justroute.engine import route_selected
        names = info.get("net_names", [])
        sel_ids = [i for i, nm in enumerate(names) if nm in only_nets]
        if not sel_ids:
            log("the selected nets are already fully routed — nothing to do")
            return None, None, None
        want = sorted(only_nets)
        log("routing ONLY the selected net"
            + ("s" if len(sel_ids) != 1 else "") + ": "
            + ", ".join(want[:8]) + (" …" if len(want) > 8 else ""))
        r = route_selected(env, sel_ids, budget_s, log=log,
                           should_stop=should_stop)
    else:
        from justroute.engine import route_best
        r = route_best(env, budget_s, log=log, should_stop=should_stop)

    # Post-route smoothing: straighten the grid staircase into DRC-clean
    # segments (Board::smooth_paths). fillet_mm>0 also rounds corners with
    # validated arcs. Both the IPC push and the file fallback read the same
    # smoothed geometry via extract_geometry. Disable with JUSTROUTE_SMOOTH=0.
    if os.environ.get("JUSTROUTE_SMOOTH", "1") != "0" and hasattr(
            env.board(), "smooth_paths"):
        try:
            fillet_cells = (fillet_mm / res) if fillet_mm > 0 else 0.0
            env.board().smooth_paths(1.5, fillet_cells)
            if fillet_cells > 0:
                log(f"rounded corners with {fillet_mm}mm fillet arcs")
        except Exception as e:
            log(f"smoothing skipped ({type(e).__name__}: {e})")

    # always produce the file fallback too
    from justroute.writer import write_routed
    out_path.write_text(write_routed(text, env.board(), frame), encoding="utf-8")
    # return ENV, not board: the board binding is a non-owning reference —
    # letting env be garbage-collected dangles it (measured live as a
    # std::bad_alloc on .nets() after return)
    return frame, env, r


def _mm_to_nm(v: float) -> int:
    return int(round(v * 1_000_000))


def _push_via_ipc(kicad, frame, model_board) -> int:
    """Create the routed tracks/vias on the live board in one commit.

    Returns the number of items created. Raises on any API mismatch — the
    caller falls back to the file path. Field names follow kipy docs
    (Board.create_items / begin_commit / push_commit); geometry re-derives
    from the same extract_geometry the file writer uses.
    """
    from kipy.board_types import Track, Via, ArcTrack  # type: ignore
    from kipy.geometry import Vector2  # type: ignore

    from justroute.writer import extract_geometry

    board = kicad.get_board()
    nets_by_name = {n.name: n for n in board.get_nets()}

    res = frame.resolution
    base_w = (frame.default_track_width or frame.rule_track_width or 0.2)
    base_via = (frame.default_via_dia or frame.rule_via_dia or 0.6)
    base_drill = frame.default_via_drill or frame.rule_via_drill or round(base_via / 2, 2)

    items = []
    for geo in extract_geometry(model_board):
        name = (frame.net_names[geo.net_id]
                if frame.net_names and frame.net_names[geo.net_id] else None)
        net = nets_by_name.get(name) if name else None
        w = base_w
        vs, vd = base_via, base_drill
        if frame.net_widths and frame.net_widths[geo.net_id]:
            w = frame.net_widths[geo.net_id]
        if frame.net_via_dias and frame.net_via_dias[geo.net_id]:
            vs = frame.net_via_dias[geo.net_id]
            vd = frame.net_via_drills[geo.net_id] or round(vs / 2, 2)
        for layer, x0, y0, x1, y1 in geo.runs:
            t = Track()
            ax, ay = frame.mm(x0, y0)
            bx, by = frame.mm(x1, y1)
            t.start = Vector2.from_xy(_mm_to_nm(ax), _mm_to_nm(ay))
            t.end = Vector2.from_xy(_mm_to_nm(bx), _mm_to_nm(by))
            t.width = _mm_to_nm(w)
            t.layer = _layer_enum(frame.layer_names[layer])
            if net is not None:
                t.net = net
            items.append(t)
        for layer, x0, y0, mx, my, x1, y1 in getattr(geo, "arcs", ()):
            a = ArcTrack()
            ax, ay = frame.mm(x0, y0)
            mmx, mmy = frame.mm(mx, my)
            bx, by = frame.mm(x1, y1)
            a.start = Vector2.from_xy(_mm_to_nm(ax), _mm_to_nm(ay))
            a.mid = Vector2.from_xy(_mm_to_nm(mmx), _mm_to_nm(mmy))
            a.end = Vector2.from_xy(_mm_to_nm(bx), _mm_to_nm(by))
            a.width = _mm_to_nm(w)
            a.layer = _layer_enum(frame.layer_names[layer])
            if net is not None:
                a.net = net
            items.append(a)
        for x, y in geo.vias:
            v = Via()
            vx, vy = frame.mm(x, y)
            v.position = Vector2.from_xy(_mm_to_nm(vx), _mm_to_nm(vy))
            v.diameter = _mm_to_nm(vs)
            v.drill_diameter = _mm_to_nm(vd)
            if net is not None:
                v.net = net
            items.append(v)

    commit = board.begin_commit()
    try:
        board.create_items(items)
        board.push_commit(commit, "JustRoute: autoroute")
    except Exception:
        board.drop_commit(commit)
        raise

    # Refill copper pours so the board LOOKS right immediately: new tracks
    # crossing a zone leave it stale (hatched) until a manual 'B'. KiCad's
    # own tools refill after editing; this matches that. Best-effort and
    # non-fatal — the routing is already committed either way. It lands as
    # its own undo step (a separate KiCad operation), so a user who wants
    # the pre-route look undoes twice; documented in the summary.
    refilled = False
    try:
        if board.get_zones():
            board.refill_zones()
            refilled = True
    except Exception as e:
        print(f"[JustRoute] (zones not auto-refilled: {type(e).__name__}: {e})")
    return len(items), refilled


def _layer_enum(name: str):
    from kipy.board_types import BoardLayer  # type: ignore
    mapping = {"F.Cu": BoardLayer.BL_F_Cu, "B.Cu": BoardLayer.BL_B_Cu}
    if name in mapping:
        return mapping[name]
    if name.startswith("In") and name.endswith(".Cu"):
        return getattr(BoardLayer, f"BL_{name.replace('.', '_')}")
    return getattr(BoardLayer, "BL_" + name.replace(".", "_"))


def _selection_net_names(board) -> set:
    """Net names of currently selected tracks/vias/pads (empty = no scope)."""
    names = set()
    try:
        for item in board.get_selection():
            net = getattr(item, "net", None)
            nm = getattr(net, "name", None) if net is not None else None
            if nm:
                names.add(str(nm))
    except Exception:
        pass
    return names


def main() -> int:
    try:
        from kipy import KiCad  # type: ignore
    except Exception as e:
        print("[JustRoute] the KiCad API library (kicad-python / kipy) is not "
              "available to this plugin's Python.")
        print("  Install it with:  pip install kicad-python")
        print(f"  ({type(e).__name__}: {e})")
        return 1
    try:
        kicad = KiCad()
        board = kicad.get_board()
    except Exception as e:
        print("[JustRoute] could not connect to KiCad's API server.")
        print("  Enable it under Preferences > Plugins ('Enable KiCad API'),")
        print("  restart KiCad, and make sure a board is open in the PCB editor.")
        print(f"  ({type(e).__name__}: {e})")
        return 1
    board.save()
    # board.name is just the FILENAME (measured live); the full path is the
    # project's directory from the DocumentSpecifier.
    doc = board.document
    proj = getattr(doc, "project", None)
    proj_dir = getattr(proj, "path", "") if proj is not None else ""
    board_path = (Path(proj_dir) / doc.board_filename if proj_dir
                  else Path(board.name).resolve())
    if not board_path.exists():
        print(f"[JustRoute] cannot locate the board file ({board_path}) — "
              "save the board once and retry.")
        return 1

    cfg = {}
    try:
        import json
        for cand in (board_path.with_name("justroute.json"),):
            if cand.exists():
                cfg = json.loads(cand.read_text(encoding="utf-8"))
    except Exception:
        cfg = {}
    # budget: env > justroute.json > size-scaled default (1s/net, 60..600)
    budget = float(os.environ.get("JUSTROUTE_BUDGET", 0) or cfg.get("budget_s", 0) or 0)
    os.environ.setdefault("JUSTROUTE_EFFORT", str(cfg.get("effort", "full")))
    route_poured = bool(cfg.get("route_poured_nets", False))
    fillet_mm = float(os.environ.get("JUSTROUTE_FILLET", 0)
                      or cfg.get("fillet_radius_mm", 0) or 0)
    only = _selection_net_names(board)
    if budget <= 0:
        try:
            n_nets = len(board.get_nets())
        except Exception:
            n_nets = 100
        budget = max(60.0, min(600.0, float(n_nets)))
    scope = f" — {len(only)} selected nets" if only else ""
    from progress_ui import make_progress
    progress = make_progress(f"JustRoute — {board_path.name}")
    try:
        from justroute.cli import _load_core
        _rc = _load_core()
        progress.on_cancel = _rc.request_route_stop   # abort in-flight pass fast
        progress.poller = getattr(_rc, "route_progress", None)  # live "12/46"
    except Exception:
        pass
    progress.log(f"routing {board_path.name} (budget {budget:.0f}s{scope})")
    if not only:
        progress.log("tip: cancel any time — everything routed so far is kept")

    out_path = board_path.with_suffix(".routed.kicad_pcb")
    res: dict = {}

    def work():
        frame, env, r = _route_file_to(board_path, out_path, budget,
                                       only_nets=only or None,
                                       log=progress.log,
                                       should_stop=progress.should_stop,
                                       route_poured=route_poured,
                                       fillet_mm=fillet_mm)
        if env is None:
            res["final"] = "every net is already routed — nothing to do"
            return
        res.update(frame=frame, env=env, r=r)
        model_board = env.board()
        st = r["stats"]
        cancelled = " (cancelled — kept best so far)" if progress.should_stop() else ""
        progress.log(f"done in {r['wall_s']}s{cancelled}: "
                     f"unrouted={st.unrouted_count} "
                     f"unconnected_pins={st.total_unconnected_pins}")
        if st.unrouted_count:
            nets_m = model_board.nets()
            missing = [frame.net_names[i] if frame.net_names and frame.net_names[i]
                       else f"net#{nets_m[i].id}"
                       for i, u in enumerate(st.unrouted) if u]
            progress.log("unrouted nets: " + ", ".join(missing[:20])
                         + (" ..." if len(missing) > 20 else ""))
            # tell the user what to DO about them (closes the loop to the
            # settings action / selection scope)
            progress.log("their airwires stay visible — raise the budget in "
                         "Autorouter settings, or select them and re-run")
        try:
            n, refilled = _push_via_ipc(kicad, frame, model_board)
            head = ("✓ board fully routed — " if st.unrouted_count == 0
                    else "")
            res["final"] = (f"{head}{n} tracks/vias added "
                            "(one undo step removes them all)"
                            + (" · pours refilled (a second undo reverts that)"
                               if refilled else ""))
            out_path.unlink(missing_ok=True)
        except Exception as e:
            res["final"] = (f"IPC write-back unavailable ({type(e).__name__}: {e}); "
                            f"routed board written to {out_path.name}")
        progress.log(res["final"])

    try:
        progress.run(work)
    except Exception as e:
        progress.close(f"routing failed: {type(e).__name__}: {e}")
        raise
    progress.close(res.get("final", "finished"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
