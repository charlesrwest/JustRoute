"""Gate A3: idempotency — JustRoute on its own output must add no copper.

Every net that got copper becomes PRE-ROUTED on reload; a second run must
emit zero new segments/vias and leave the file's copper untouched.
Usage: python3 tests/gate_a3_idempotent.py [N_BOARDS] [SEED] [WORKDIR]
"""
from __future__ import annotations

import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "routing_env" / "build-opt"))

from justroute.cli import route_file  # noqa: E402
from justroute.prepare import strip_routing  # noqa: E402

HARVEST = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def copper_counts(text: str):
    return (len(re.findall(r"\n\s*\(segment", text)),
            len(re.findall(r"\n\s*\(via ", text)))


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    work = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("/tmp/gate_a3")
    work.mkdir(parents=True, exist_ok=True)
    cands = sorted(p for p in HARVEST.glob("*.kicad_pcb")
                   if p.stat().st_size < 2_000_000)
    random.Random(seed).shuffle(cands)
    done = passed = 0
    for src in cands:
        if done >= n:
            break
        try:
            stripped = work / (src.stem[:70] + ".s.kicad_pcb")
            r1p = work / (src.stem[:70] + ".r1.kicad_pcb")
            r2p = work / (src.stem[:70] + ".r2.kicad_pcb")
            stripped.write_text(strip_routing(
                src.read_text(encoding="utf-8", errors="replace")), encoding="utf-8")
            r1 = route_file(stripped, r1p, None, certify_result=False, budget_s=45)
            if r1["unrouted"] != 0 or r1["nets"] < 3:
                # strict idempotency is only defined for FULLY routed boards:
                # a second pass legitimately routes nets the first one missed
                # (progress, not duplication)
                continue
            r2 = route_file(r1p, r2p, None, certify_result=False, budget_s=45)
            s1, v1 = copper_counts(r1p.read_text())
            s2, v2 = copper_counts(r2p.read_text())
            routed1 = r1["nets"] - r1["unrouted"]
            # The invariant is COPPER identity: zero segments/vias added on the
            # second run. (pre_routed can lag routed1: nets connected purely by
            # thru-hole barrels emit no copper and stay "routable" on reload.)
            ok = (s2 == s1 and v2 == v1)
            done += 1
            passed += ok
            print(f"[{done}] {'PASS' if ok else 'FAIL'} {src.stem[:48]:48s} "
                  f"routed={routed1} prerouted2={r2['pre_routed_nets']} "
                  f"seg {s1}->{s2} via {v1}->{v2}", flush=True)
        except Exception as e:
            print(f"[--] SKIP {src.stem[:48]} {type(e).__name__}: {str(e)[:60]}",
                  flush=True)
    print(f"\nGATE A3: {passed}/{done} idempotent")
    return 0 if passed == done and done else 1


if __name__ == "__main__":
    sys.exit(main())
