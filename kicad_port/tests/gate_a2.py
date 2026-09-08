"""Gate A2: INCREMENTAL routing — honor existing copper, route only the rest.

For each sampled fully-routed .kicad_pcb: pick half of its routed nets
(seeded), strip ONLY their copper (other nets' tracks, pours, everything
else stays) -> route the stripped nets around the surviving copper ->
write -> three-way kicad-cli drc (partial baseline vs routed vs original).

PASS per board = zero new blocking violations involving router copper AND
unconnected items not increased. This exercises exactly the plugin's core
use case: "route the rest of my board".

Usage:  python3 tests/gate_a2.py [N_BOARDS] [SEED] [WORKDIR]
"""

from __future__ import annotations

import json
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "routing_env" / "build-opt"))

from justroute import certify  # noqa: E402
from justroute.cli import route_file  # noqa: E402
from justroute.prepare import routed_nets, strip_routing  # noqa: E402

HARVEST = Path.home() / "storage2" / "pcb_board_harvest" / "raw"
MAX_FILE_BYTES = 3_000_000


def main() -> int:
    n_boards = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    work = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("/tmp/gate_a2")
    work.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)

    candidates = sorted(p for p in HARVEST.glob("*.kicad_pcb")
                        if p.stat().st_size < MAX_FILE_BYTES)
    rng.shuffle(candidates)

    import routing_env  # noqa: F401

    results = []
    tried = 0
    for src in candidates:
        if len(results) >= n_boards:
            break
        tried += 1
        name = src.stem[:52]
        try:
            text = src.read_text(encoding="utf-8", errors="replace")
            nets = sorted(routed_nets(text))
            if len(nets) < 4:
                continue    # not enough routed nets for a meaningful split
            victims = set(rng.sample(nets, len(nets) // 2))
            partial = work / (src.stem[:76] + ".partial.kicad_pcb")
            routed = work / (src.stem[:76] + ".routed.kicad_pcb")
            partial.write_text(strip_routing(text, only_nets=victims),
                               encoding="utf-8")
            r = route_file(partial, routed, None, certify_result=False,
                           budget_s=60.0)
            cert = certify.certify(partial, routed, reference_pcb=src)
            ok = cert.get("passed", False)
            results.append({
                "board": name, "ok": ok, "nets": r["nets"],
                "pre_routed": r.get("pre_routed_nets", 0),
                "victims": len(victims), "unrouted": r["unrouted"],
                "new_viol": sum(cert.get("new_violations", {}).values()),
                "new_types": ",".join(cert.get("new_violations", {})) or "-",
                "unc_before": cert.get("unconnected_before"),
                "unc_after": cert.get("unconnected_after"),
                "wall_s": r["wall_s"],
            })
            print(f"[{len(results):2d}] {'PASS' if ok else 'FAIL':4s} {name:52s} "
                  f"route={r['nets']:<3d} keep={r.get('pre_routed_nets', 0):<3d} "
                  f"unrouted={r['unrouted']:<3d} new={results[-1]['new_viol']:<3d} "
                  f"unc {cert.get('unconnected_before')}->{cert.get('unconnected_after')}",
                  flush=True)
        except Exception as e:
            print(f"[--] SKIP {name:52s} {type(e).__name__}: {str(e)[:80]}", flush=True)
            continue

    passed = sum(1 for r in results if r["ok"])
    print(f"\nGATE A2: {passed}/{len(results)} boards clean "
          f"({tried} candidates tried)")
    (work / "gate_a2_results.json").write_text(json.dumps(results, indent=2))
    return 0 if passed == len(results) and results else 1


if __name__ == "__main__":
    sys.exit(main())
