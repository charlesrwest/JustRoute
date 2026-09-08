"""JustRoute: route a .kicad_pcb headlessly and write the result back.

    python3 -m justroute.cli in.kicad_pcb [-o out.kicad_pcb] [--resolution MM]
                            [--no-certify]

v1 routes with the plain full-pass greedy (board order). The production
stack — conflict-genome order search plus the PathFinder negotiation
finisher — is wired in behind this same entry point in the next step;
the writer/certify plumbing is identical either way. CPU-only.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from .writer import BoardFrame, write_routed


def _load_core():
    # Shared ABI-aware loader: picks the routing_env binary matching this
    # interpreter from those bundled, with an actionable error if none fit.
    from .coreloader import load_core
    return load_core()


def _tuned_protocol(env) -> None:
    """The corpus-tuned greedy setup (run_corpus_par 'tuned_greedy'): topological
    net order, tuned via cost, RUDY congestion field. Falls back to plain board
    order if the rl package is not importable."""
    try:
        repo = Path(__file__).resolve().parents[2]
        rl_path = str(repo / "routing_env" / "python")
        if rl_path not in sys.path:
            sys.path.insert(0, rl_path)
        from rl.topo import topo_order, apply_net_order
        env.board().set_via_cost(2.71)
        apply_net_order(env, topo_order(env.board()))
        # RUDY static congestion steering measured to HURT completion on the
        # corpus (fast single pass +5.4 pts, full engine +2.7 pts, and fewer
        # unconnected pins) — it double-counts with the negotiation finisher's
        # own dynamic (history-cost) congestion, forcing detours that fail.
        # Disabled (weight 0); the setter stays for easy experimentation.
        env.board().set_congestion_weight(0.0)
    except ImportError:
        pass


FINE_PITCH_MM = 0.45     # min pad spacing below this needs the fine grid
COARSE_RES = 0.05
FINE_RES = 0.025


def friendly_load_error(e: Exception) -> str | None:
    """Plain-English reason for a loader RuntimeError a user might hit, or
    None if it's not one we recognize (caller should re-raise / show raw)."""
    m = str(e)
    if "no nets/pads to route" in m or "no routable nets" in m:
        return ("nothing to route: every net is already connected, or the "
                "board has no routable pads.")
    if "board has no copper layers" in m or "missing 'layers'" in m:
        return "this board has no copper layers defined — nothing to route on."
    if "could not determine board bounds" in m or "grid out of supported bounds" in m:
        return ("could not fit the board to the routing grid (missing edge "
                "cuts, or an extreme board size).")
    if any(s in m for s in ("expected '('", "unbalanced", "unterminated",
                            "trailing content", "nesting too deep", "empty atom")):
        return "the .kicad_pcb file could not be parsed (unexpected format)."
    if "non-numeric" in m:
        return "the board contains a malformed coordinate the loader rejected."
    return None


def _load_config(input_path: Path) -> dict:
    """Optional justroute.json next to the board (or the project): persistent
    user preferences — {"budget_s": 300, "effort": "full", "resolution": 0.05}.
    CLI flags override config values; config overrides defaults."""
    import json
    for cand in (input_path.with_name("justroute.json"),
                 input_path.parent / "justroute.json"):
        if cand.exists():
            try:
                return json.loads(cand.read_text(encoding="utf-8"))
            except Exception:
                return {}
    return {}


def route_file(input_path: Path, output_path: Path, resolution: float | None = None,
               certify_result: bool = True, budget_s: float = 0.0,
               effort: str = "fast", project: Path | None = None,
               route_poured: bool = False, smooth: bool = True,
               smooth_dev: float = 1.5, fillet_mm: float = 0.0,
               log=lambda m: None) -> dict:
    rc = _load_core()
    text = input_path.read_text(encoding="utf-8", errors="replace")

    def _already_routed(e: Exception) -> bool:
        return "all nets already routed" in str(e)

    if resolution is None:
        # Auto-resolution: fine-pitch parts (0.4mm QFNs) have ZERO clearance
        # slack — any half-cell discretization at 0.05 shows up as real KiCad
        # violations, and the same board certifies clean at 0.025 (measured).
        env = rc.RoutingEnv(2, 10, 10, COARSE_RES, 5.0, 1.0, 0.1)
        try:
            probe = env.load_kicad_pcb_info(text, COARSE_RES,
                                            skip_poured=not route_poured)
        except RuntimeError as e:
            if _already_routed(e):
                output_path.write_text(text, encoding="utf-8")
                log("board is already fully routed — nothing to do")
                return {"input": str(input_path), "output": str(output_path),
                        "nets": 0, "pre_routed_nets": -1,
                        "project_class_nets": 0, "unrouted_nets": [],
                        "unrouted": 0, "unconnected_pins": 0,
                        "drc_violations_model": 0, "vias": 0, "wall_s": 0.0,
                        "already_complete": True}
            raise
        resolution = (FINE_RES if probe.get("min_pad_spacing", 99.0) < FINE_PITCH_MM
                      else COARSE_RES)

    env = rc.RoutingEnv(2, 10, 10, resolution, 5.0, 1.0, 0.1)
    try:
        info = env.load_kicad_pcb_info(text, resolution,
                                       skip_poured=not route_poured)
    except RuntimeError as e:
        if _already_routed(e):
            output_path.write_text(text, encoding="utf-8")
            log("board is already fully routed — nothing to do")
            return {"input": str(input_path), "output": str(output_path),
                    "nets": 0, "pre_routed_nets": -1,
                    "project_class_nets": 0, "unrouted_nets": [],
                    "unrouted": 0, "unconnected_pins": 0,
                    "drc_violations_model": 0, "vias": 0, "wall_s": 0.0,
                    "already_complete": True}
        raise
    frame = BoardFrame.from_info(info, resolution)
    # Sidecar project file: KiCad 6+ keeps netclass definitions/assignments
    # in <project>.kicad_pro, not the pcb — apply them to the physics tiers
    # and the emitted geometry (explicit --project beats auto-detection).
    from . import project as _project
    pro = project or _project.find_project(input_path)
    applied_classes = 0
    if pro is not None and Path(pro).exists():
        try:
            applied_classes = _project.apply_project_rules(
                env, env.board(), frame, info, Path(pro), resolution, log=log)
        except Exception as e:
            log(f"project file ignored ({type(e).__name__}: {e})")
    _tuned_protocol(env)
    board = env.board()
    if budget_s > 0.0:
        board.set_route_time_budget_s(budget_s)

    t0 = time.monotonic()
    if effort == "full":
        from .engine import route_best
        # default budget scales with board size (1s/net, clamped 60..600) —
        # the same rule the KiCad plugin uses
        auto_budget = max(60.0, min(600.0, float(info["nets"])))
        r = route_best(env, budget_s if budget_s > 0 else auto_budget, log=log)
        stats = r["stats"]
    else:
        stats = board.route_all()
    wall = time.monotonic() - t0

    # Post-route smoothing: collapse the grid staircase into straight segments
    # (validated DRC-clean per Board::smooth_paths). On by default; JUSTROUTE_SMOOTH=0
    # or smooth=False disables it.
    if smooth and hasattr(board, "smooth_paths"):
        try:
            fillet_cells = (fillet_mm / resolution) if fillet_mm > 0 else 0.0
            n = board.smooth_paths(smooth_dev, fillet_cells)
            if fillet_cells > 0:
                log(f"smoothed + filleted corners (radius {fillet_mm}mm)")
        except Exception as e:
            log(f"smoothing skipped ({type(e).__name__}: {e})")

    output_path.write_text(write_routed(text, board, frame), encoding="utf-8")

    names = info.get("net_names", [])
    unrouted_names = [names[i] if i < len(names) and names[i] else f"net#{i}"
                      for i, u in enumerate(getattr(stats, "unrouted", []) or []) if u]
    result = {
        "input": str(input_path),
        "output": str(output_path),
        "nets": info["nets"],
        "pre_routed_nets": info.get("pre_routed_nets", 0),
        "pour_fed_nets": list(info.get("pour_fed_nets", []) or []),
        "project_class_nets": applied_classes,
        "unrouted_nets": unrouted_names,
        "unrouted": stats.unrouted_count,
        "unconnected_pins": stats.total_unconnected_pins,
        "drc_violations_model": stats.drc_violations,
        "vias": stats.total_vias,
        "wall_s": round(wall, 2),
    }
    if certify_result:
        from . import certify
        try:
            result["certification"] = certify.certify(input_path, output_path)
        except certify.KicadCliMissing as e:
            result["certification"] = {"skipped": str(e)}
    return result


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="JustRoute", description=__doc__)
    ap.add_argument("input", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None)
    ap.add_argument("--resolution", type=float, default=None,
                    help="grid resolution in mm (default: auto — 0.05, or 0.025 "
                         "when the board has sub-0.45mm pad pitch)")
    ap.add_argument("--json", action="store_true",
                    help="machine-readable full result instead of the summary")
    ap.add_argument("--no-certify", action="store_true",
                    help="skip the kicad-cli drc baseline-diff check")
    ap.add_argument("--budget", type=float, default=0.0,
                    help="wall-time routing budget in seconds (0 = unlimited; "
                         "--effort full defaults to 120)")
    ap.add_argument("--effort", choices=("fast", "full"), default=None,
                    help="fast: one tuned-greedy pass; full (default): retention "
                         "+ genome evolution + blame reorder + negotiation")
    ap.add_argument("--project", type=Path, default=None,
                    help=".kicad_pro with netclass definitions (default: the "
                         "sidecar next to the board, when present)")
    ap.add_argument("--route-poured", action="store_true",
                    help="also route nets owned by a copper pour (default: "
                         "leave them to their zones)")
    ap.add_argument("--no-smooth", action="store_true",
                    help="skip post-route trace smoothing (keep the raw grid "
                         "staircase geometry)")
    ap.add_argument("--fillet", type=float, nargs="?", const=0.5, default=None,
                    metavar="MM",
                    help="round trace corners with arcs of this radius in mm "
                         "(default 0.5 when given with no value); DRC-validated")
    args = ap.parse_args(argv)

    out = args.output or args.input.with_suffix(".routed.kicad_pcb")
    cfg = _load_config(args.input)
    resolution = args.resolution if args.resolution is not None else cfg.get("resolution")
    budget = args.budget if args.budget else float(cfg.get("budget_s", 0.0))
    effort = args.effort or cfg.get("effort", "full")
    route_poured = args.route_poured or bool(cfg.get("route_poured_nets", False))
    smooth = (not args.no_smooth and cfg.get("smooth", True)
              and os.environ.get("JUSTROUTE_SMOOTH", "1") != "0")
    fillet_mm = (args.fillet if args.fillet is not None
                 else float(cfg.get("fillet_radius_mm", 0.0) or 0.0))
    try:
        result = route_file(args.input, out, resolution,
                            certify_result=not args.no_certify,
                            budget_s=budget, effort=effort, project=args.project,
                            route_poured=route_poured, smooth=smooth,
                            fillet_mm=fillet_mm,
                            log=lambda m: print(f"[JustRoute] {m}", file=sys.stderr))
    except RuntimeError as e:
        friendly = friendly_load_error(e)
        if friendly is None:
            raise
        print(f"JustRoute: {friendly}", file=sys.stderr)
        return 2
    cert = result.get("certification", {})
    if args.json:
        json.dump(result, sys.stdout, indent=2)
        print()
        return 0 if cert.get("passed", True) else 1
    # human summary (progressive disclosure: --json for the full record)
    routed = result["nets"] - result["unrouted"]
    print(f"routed {routed}/{result['nets']} nets"
          + (f" ({result['pre_routed_nets']} already routed, kept)"
             if result["pre_routed_nets"] else "")
          + f" in {result['wall_s']}s -> {result['output']}")
    if result.get("project_class_nets"):
        print(f"netclasses from project file applied to "
              f"{result['project_class_nets']} nets")
    if result.get("pour_fed_nets"):
        pf = result["pour_fed_nets"]
        print("left to their pours: " + ", ".join(pf[:10])
              + (" ..." if len(pf) > 10 else "")
              + "  (--route-poured to route them)")
    if result.get("unrouted_nets"):
        miss = result["unrouted_nets"]
        print("unrouted: " + ", ".join(miss[:15])
              + (" ..." if len(miss) > 15 else ""))
    if cert and "skipped" not in cert:
        ok = cert.get("passed")
        nv = sum(cert.get("new_violations", {}).values())
        print(("DRC clean (kicad-cli): no new violations" if ok else
               f"DRC: {nv} new violation(s) — see --json for detail")
              + f"; unconnected {cert.get('unconnected_before')} -> "
              + f"{cert.get('unconnected_after')}")
    elif cert:
        print(f"certification skipped: {cert['skipped']}")
    return 0 if cert.get("passed", True) else 1


if __name__ == "__main__":
    sys.exit(main())
