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
        # Optional GLOBAL PAD REPULSION (experiment; off unless env-set): every
        # net pays a soft penalty for hugging non-target pads, so routes bow
        # around dense pad fields instead of wedging through them and blocking
        # later nets. Sweep-tuned sweet spot pad_mult 4-6, falloff radius 4.
        # Validated through the gates + full engine before any default change.
        pad_mult = float(os.environ.get("JUSTROUTE_PAD_REPULSION", "0") or 0)
        falloff = float(os.environ.get("JUSTROUTE_PAD_FALLOFF", "0") or 0)
        if pad_mult > 0:
            b = env.board()
            if falloff > 0:
                b.set_falloff_radius(falloff)
            for i in range(len(b.nets())):
                b.set_avoidance(i, pad_mult, 0.0)
    except ImportError:
        pass


FINE_PITCH_MM = 0.45     # min pad spacing below this needs the fine grid
COARSE_RES = 0.05
FINE_RES = 0.025
# High-fanout defer (OPT-IN, default OFF): optionally skip nets with more than
# this many pins. This was investigated as a fix for "hard" boards but the
# premise was flawed — the failing high-fanout nets were power/ground nets that
# are POURED on real boards (handled by skip_poured); they only looked like
# routing failures because the test harness strips zones. Deferring gives +0
# real completion and "pour it" doesn't solve a genuine routing failure. Kept as
# an explicit knob (--max-fanout N) for anyone who wants it; 0 = route every net.
DEFAULT_MAX_FANOUT = 0


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
               max_fanout: int = DEFAULT_MAX_FANOUT,
               auto_pour: bool = False,
               log=lambda m: None) -> dict:
    rc = _load_core()
    text = input_path.read_text(encoding="utf-8", errors="replace")

    # Auto-pour (opt-in): a from-scratch board's biggest net (GND) is a plane a
    # human pours, not a track bundle. Generate the zone BEFORE loading — the
    # loader's zone-glue then skips the net (skip_poured), the writer keeps the
    # zone, and kicad-cli --refill-zones verifies it truly connects the net.
    pour_gen = None
    if auto_pour:
        from .pour import inject_pour
        text, pour_gen = inject_pour(text)
        if pour_gen:
            log(f"generated a {pour_gen['name']} pour on {pour_gen['layer']} "
                f"({pour_gen['pads']} pads) — connectivity checked by DRC")

    def _already_routed(e: Exception) -> bool:
        return "all nets already routed" in str(e)

    if resolution is None:
        # Auto-resolution: fine-pitch parts (0.4mm QFNs) have ZERO clearance
        # slack — any half-cell discretization at 0.05 shows up as real KiCad
        # violations, and the same board certifies clean at 0.025 (measured).
        env = rc.RoutingEnv(2, 10, 10, COARSE_RES, 5.0, 1.0, 0.1)
        try:
            probe = env.load_kicad_pcb_info(text, COARSE_RES,
                                            skip_poured=not route_poured,
                                            max_fanout=max_fanout)
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

    from . import project as _project
    pro = project or _project.find_project(input_path)
    t0 = time.monotonic()
    attempt = 0
    while True:
        env = rc.RoutingEnv(2, 10, 10, resolution, 5.0, 1.0, 0.1)
        try:
            info = env.load_kicad_pcb_info(text, resolution,
                                           skip_poured=not route_poured,
                                           max_fanout=max_fanout)
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

        if effort == "full":
            from .engine import route_best
            # default budget scales with board size (1s/net, clamped 60..600) —
            # the same rule the KiCad plugin uses
            auto_budget = max(60.0, min(600.0, float(info["nets"])))
            r = route_best(env, budget_s if budget_s > 0 else auto_budget, log=log)
            stats = r["stats"]
        else:
            stats = board.route_all()

        # Failure-driven pour (auto-pour's second trigger): the router just
        # PROVED a mid-fanout net won't route as tracks — give it the plane a
        # designer would, and re-route the board around it. Fires at most once,
        # and inject_pour's no-existing-zone guard makes it a no-op whenever
        # any zone exists (including one we generated pre-route).
        if not (auto_pour and attempt == 0 and stats.unrouted_count > 0):
            break
        from .pour import inject_pour as _inject_pour
        from .pour import _pad_counts, _net_names
        FAILURE_POUR_MIN_PINS = 12
        # map positions -> stable ids: the engine reorders nets, and net_names
        # is keyed by load order == id (position-indexing named wrong nets)
        names0 = info.get("net_names", [])
        _nets0 = board.nets()
        failed_names = [names0[_nets0[i].id]
                        for i, u in enumerate(stats.unrouted)
                        if u and _nets0[i].id < len(names0) and names0[_nets0[i].id]]
        counts = _pad_counts(text)
        by_name = {}
        for num, nm in _net_names(text).items():
            by_name.setdefault(nm, num)
        cand_name, cand_pads = None, 0
        for nm in failed_names:
            num = by_name.get(nm)
            if num is not None and counts.get(num, 0) > cand_pads:
                cand_name, cand_pads = nm, counts[num]
        if cand_name is None or cand_pads < FAILURE_POUR_MIN_PINS:
            break
        text2, gen2 = _inject_pour(text, min_pins=FAILURE_POUR_MIN_PINS,
                                   target_name=cand_name)
        if gen2 is None:
            break
        text, pour_gen = text2, gen2
        log(f"net '{cand_name}' won't route as tracks ({cand_pads} pads) — "
            f"generating its pour and re-routing")
        attempt += 1
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

    # Stitching vias: pads the generated pour can't reach (SMD pads on the far
    # layer) are reported by kicad-cli with exact positions — drop a same-net
    # via on each (via-in-pad: no clearance rule against its own pad; the
    # barrel reaches the pour layer). Kept only if the judge says the board
    # did not get worse; otherwise reverted whole.
    if pour_gen is not None and certify_result:
        from . import certify as _certify
        from .pour import add_stitch_vias, plane_unconnected_positions
        try:
            rep = _certify.run_drc(output_path)
            spots = plane_unconnected_positions(rep, pour_gen["name"])
            if spots:
                routed_text = output_path.read_text(encoding="utf-8")
                n_viol0 = len(rep.get("violations", []))

                def _viol_positions(report):
                    out = []
                    for v in report.get("violations", []):
                        for it in v.get("items", []):
                            p = it.get("pos") or {}
                            if "x" in p:
                                out.append((float(p["x"]), float(p["y"])))
                    return out

                # place all candidates, then drop the ones the judge rejects
                # (a via near a new violation position); at most two passes.
                keep = list(spots)
                for _round in (0, 1):
                    output_path.write_text(
                        add_stitch_vias(routed_text, keep, pour_gen["net"]),
                        encoding="utf-8")
                    rep2 = _certify.run_drc(output_path)
                    if len(rep2.get("violations", [])) <= n_viol0:
                        break
                    bad = _viol_positions(rep2)
                    keep = [s for s in keep
                            if not any(abs(s[0] - bx) < 1.0 and abs(s[1] - by) < 1.0
                                       for bx, by in bad)]
                    if not keep:
                        break
                if not keep or len(rep2.get("violations", [])) > n_viol0:
                    output_path.write_text(routed_text, encoding="utf-8")
                    log("stitching reverted (vias would add violations)")
                else:
                    pour_gen["stitch_vias"] = len(keep)
                    dropped = len(spots) - len(keep)
                    log(f"added {len(keep)} stitching via(s) to reach the "
                        f"{pour_gen['name']} pour"
                        + (f" ({dropped} spot(s) skipped)" if dropped else ""))
        except _certify.KicadCliMissing:
            pass
        except Exception as e:
            log(f"stitching skipped ({type(e).__name__}: {e})")

    # stats.unrouted is indexed by CURRENT net position; the engine reorders
    # nets (topo/genome/blame), so map through the stable net id — net_names
    # is keyed by load order == id. Position-indexing named the WRONG nets.
    names = info.get("net_names", [])
    _bnets = board.nets()
    unrouted_names = [
        (names[_bnets[i].id] if _bnets[i].id < len(names) and names[_bnets[i].id]
         else f"net#{_bnets[i].id}")
        for i, u in enumerate(getattr(stats, "unrouted", []) or []) if u]
    result = {
        "input": str(input_path),
        "output": str(output_path),
        "nets": info["nets"],
        "pre_routed_nets": info.get("pre_routed_nets", 0),
        "pour_fed_nets": list(info.get("pour_fed_nets", []) or []),
        "generated_pour": pour_gen,
        "deferred_fanout_nets": list(info.get("deferred_fanout_nets", []) or []),
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
    ap.add_argument("--auto-pour", action="store_true",
                    help="generate a copper pour for the board's biggest net "
                         "(GND-style planes) when the file has no zones yet — "
                         "what a designer would draw before routing; "
                         "connectivity is verified by the DRC check")
    ap.add_argument("--max-fanout", type=int, default=None, metavar="N",
                    help=f"defer nets with more than N pins as plane/pour "
                         f"candidates instead of routing them (default "
                         f"{DEFAULT_MAX_FANOUT}; 0 routes every net). Such nets "
                         f"route ~0%% of the time as tracks.")
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
    max_fanout = (args.max_fanout if args.max_fanout is not None
                  else int(cfg.get("max_fanout", DEFAULT_MAX_FANOUT)))
    try:
        result = route_file(args.input, out, resolution,
                            certify_result=not args.no_certify,
                            budget_s=budget, effort=effort, project=args.project,
                            route_poured=route_poured, smooth=smooth,
                            fillet_mm=fillet_mm, max_fanout=max_fanout,
                            auto_pour=(args.auto_pour
                                       or bool(cfg.get("auto_pour", False))),
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
    # deferred high-fanout nets are held in pre_routed_nets; don't call them "kept"
    kept = result["pre_routed_nets"] - len(result.get("deferred_fanout_nets", []))
    print(f"routed {routed}/{result['nets']} nets"
          + (f" ({kept} already routed, kept)" if kept > 0 else "")
          + f" in {result['wall_s']}s -> {result['output']}")
    if result.get("project_class_nets"):
        print(f"netclasses from project file applied to "
              f"{result['project_class_nets']} nets")
    if result.get("generated_pour"):
        gp = result["generated_pour"]
        print(f"generated a {gp['name']} pour on {gp['layer']} "
              f"({gp['pads']} pads) — see the DRC line for connectivity")
    if result.get("pour_fed_nets"):
        pf = [n for n in result["pour_fed_nets"]
              if not (result.get("generated_pour")
                      and n == result["generated_pour"]["name"])]
        if pf:
            print("left to their pours: " + ", ".join(pf[:10])
                  + (" ..." if len(pf) > 10 else "")
                  + "  (--route-poured to route them)")
    if result.get("deferred_fanout_nets"):
        df = result["deferred_fanout_nets"]
        print("high-fanout, pour these (not routed): " + ", ".join(df[:10])
              + (" ..." if len(df) > 10 else "")
              + "  (--max-fanout 0 to route them anyway)")
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
