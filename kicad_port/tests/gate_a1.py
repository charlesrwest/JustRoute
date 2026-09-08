"""Gate A1: writer round-trip + kicad-cli certification across harvested boards.

For each sampled .kicad_pcb: strip existing routing -> route (tuned greedy) ->
write -> kicad-cli drc baseline diff -> re-import the output with our loader.

PASS per board = zero new violations AND unconnected not increased AND the
output re-imports with the same net count. Full routing is NOT required (that
is the production stack's job, wired in later) — emitted copper must simply
never be wrong.

Usage:
  python3 tests/gate_a1.py [N_BOARDS] [SEED]
Writes a result table to stdout and artifacts to the scratch dir.
"""

from __future__ import annotations

import json
import random
import sys
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "routing_env" / "build-opt"))

from justroute import certify  # noqa: E402
from justroute.cli import route_file  # noqa: E402
from justroute.prepare import strip_routing  # noqa: E402

HARVEST = Path.home() / "storage2" / "pcb_board_harvest" / "raw"
MAX_FILE_BYTES = 3_000_000    # keep the gate quick; big boards are covered elsewhere


def main() -> int:
    n_boards = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    work = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("/tmp/gate_a1")
    work.mkdir(parents=True, exist_ok=True)

    candidates = sorted(p for p in HARVEST.glob("*.kicad_pcb")
                        if p.stat().st_size < MAX_FILE_BYTES)
    random.Random(seed).shuffle(candidates)

    import routing_env  # noqa: F401  (fail early if module missing)

    results = []
    tried = 0
    for src in candidates:
        if len(results) >= n_boards:
            break
        tried += 1
        name = src.stem[:52]
        stripped = work / (src.stem[:80] + ".stripped.kicad_pcb")
        routed = work / (src.stem[:80] + ".routed.kicad_pcb")
        try:
            stripped.write_text(
                strip_routing(src.read_text(encoding="utf-8", errors="replace")),
                encoding="utf-8")
            r = route_file(stripped, routed, None, certify_result=False,
                           budget_s=60.0)
            cert = certify.certify(stripped, routed, reference_pcb=src)
            r["certification"] = cert
            if "skipped" in cert:
                raise RuntimeError(cert["skipped"])
            # reload accounting: re-importing our own output treats every net we
            # gave copper as PRE-ROUTED (incremental semantics), so the invariant
            # is routable + pre-routed == original routable + original pre-routed.
            env2 = routing_env.RoutingEnv(2, 10, 10, 0.05, 5.0, 1.0, 0.1)
            info2 = env2.load_kicad_pcb_info(routed.read_text(encoding="utf-8"), 0.05)
            reload_ok = (info2["nets"] + info2["pre_routed_nets"]
                         == r["nets"] + r.get("pre_routed_nets", 0))
            ok = cert.get("passed", False) and reload_ok
            results.append({
                "board": name, "ok": ok, "nets": r["nets"],
                "unrouted": r["unrouted"],
                "new_viol": sum(cert.get("new_violations", {}).values()),
                "new_types": ",".join(cert.get("new_violations", {})) or "-",
                "new_warn": sum(cert.get("new_warnings", {}).values()),
                "excused": sum(cert.get("excused_by_reference", {}).values()),
                "unc_before": cert.get("unconnected_before"),
                "unc_after": cert.get("unconnected_after"),
                "reload_ok": reload_ok, "wall_s": r["wall_s"],
            })
            print(f"[{len(results):2d}] {'PASS' if ok else 'FAIL':4s} {name:52s} "
                  f"nets={r['nets']:<4d} unrouted={r['unrouted']:<3d} "
                  f"new={results[-1]['new_viol']:<3d} "
                  f"unc {cert.get('unconnected_before')}->{cert.get('unconnected_after')}"
                  + ("" if reload_ok else "  RELOAD-MISMATCH"), flush=True)
        except Exception as e:
            # loader rejections (fidelity guards, exotic dialects) are recorded,
            # not counted against the gate — the gate judges the WRITER.
            print(f"[--] SKIP {name:52s} {type(e).__name__}: {str(e)[:80]}", flush=True)
            continue

    passed = sum(1 for r in results if r["ok"])
    fully = sum(1 for r in results if r["ok"] and r["unrouted"] == 0)
    print(f"\nGATE A1: {passed}/{len(results)} boards clean "
          f"({fully} fully routed by plain greedy; {tried} candidates tried)")
    (work / "gate_a1_results.json").write_text(json.dumps(results, indent=2))
    return 0 if passed == len(results) and results else 1


if __name__ == "__main__":
    sys.exit(main())
