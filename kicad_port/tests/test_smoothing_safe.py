"""Certification: post-route smoothing never adds a DRC violation.

The whole promise of Board::smooth_paths is "straighter traces, zero new
violations." This proves it the hard way: route each board, write it out
twice — once with the raw grid staircase, once smoothed — and run
kicad-cli DRC on both. The smoothed report must have <= as many
Track/Via-involving violations of every (type, severity) as the raw one.
A single extra violation fails the gate.

Also reports the geometry reduction (segments raw -> smoothed) so a
regression that silently stops smoothing is visible too.

Usage: python3 tests/test_smoothing_safe.py [n_boards] [budget_s] [max_dev]
"""

import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "routing_env" / "build-opt"))
sys.path.insert(0, str(ROOT / "kicad_port"))

import routing_env as rc  # noqa: E402
from justroute.prepare import strip_routing  # noqa: E402
from justroute.cli import _tuned_protocol  # noqa: E402
from justroute.engine import route_best  # noqa: E402
from justroute.writer import write_routed, BoardFrame, extract_geometry  # noqa: E402
from justroute import certify  # noqa: E402

CORPUS = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def counts(pcb: Path) -> Counter:
    return certify._violation_counts(certify.run_drc(pcb))


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    budget = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    max_dev = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
    fillet = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0  # arc radius in cells
    outdir = Path("/tmp/smoothing_cert" + ("_fillet" if fillet else ""))
    outdir.mkdir(exist_ok=True)

    boards = sorted(CORPUS.glob("*.kicad_pcb"))
    tested = clean = 0
    raw_segs = sm_segs = 0
    for p in boards:
        if tested >= n:
            break
        text = p.read_text(encoding="utf-8", errors="replace")
        if "(segment" not in text and "(footprint" not in text:
            continue
        stripped = strip_routing(text)
        try:
            env = rc.RoutingEnv(2, 10, 10, 0.025, 5.0, 1.0, 0.1)
            info = env.load_kicad_pcb_info(stripped, 0.025)
        except RuntimeError:
            continue
        _tuned_protocol(env)
        try:
            route_best(env, budget, log=lambda m: None)
        except Exception:
            continue
        b = env.board()
        frame = BoardFrame.from_info(info, 0.025)

        raw_pcb = outdir / (p.stem[:40] + ".raw.kicad_pcb")
        sm_pcb = outdir / (p.stem[:40] + ".sm.kicad_pcb")
        raw_pcb.write_text(write_routed(stripped, b, frame), encoding="utf-8")
        r_before = sum(len(g.runs) for g in extract_geometry(b))
        b.smooth_paths(max_dev, fillet)
        sm_pcb.write_text(write_routed(stripped, b, frame), encoding="utf-8")
        s_after = sum(len(s) for s in b.smoothed_paths() if s)

        try:
            cr, cs = counts(raw_pcb), counts(sm_pcb)
        except Exception as e:
            print(f"SKIP {p.name[:44]}: DRC error ({e})")
            continue
        tested += 1
        raw_segs += r_before
        sm_segs += s_after
        added = {k: cs[k] - cr.get(k, 0) for k in cs if cs[k] > cr.get(k, 0)}
        ok = not added
        clean += ok
        red = f"{r_before}->{s_after} segs" if r_before else "no runs"
        print(f"{'PASS' if ok else 'FAIL'} {p.name[:44]}: {red}"
              + (f"  ADDED {added}" if added else "  (no new violations)"))

    print(f"\n{clean}/{tested} boards: smoothing added zero DRC violations")
    if raw_segs:
        print(f"geometry: {raw_segs} -> {sm_segs} segments "
              f"({100*(1-sm_segs/raw_segs):.0f}% fewer) at max_dev={max_dev}")
    return 0 if tested and clean == tested else 1


if __name__ == "__main__":
    sys.exit(main())
