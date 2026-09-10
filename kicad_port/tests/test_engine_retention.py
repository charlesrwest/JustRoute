"""Regression: route_best's returned stats must describe the LIVE board.

The engine's best-state retention had a dead guard: `if _score(st) > best`
where st tracks the best-so-far stats — _score(st) == best by construction,
so the final restore never fired. When the genome loop's last trial was worse
than the retained best (the usual case) and no later stage re-routed, the
board handed to the writer was the last-attempted state while the returned
stats described the best one (measured: Thanos reported 4 unrouted, board
really had 19). The fix recomputes the LIVE board's score at every landing
point; this test asserts reported == live on boards that exercise the genome
loop (some residue after the first pass).
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "routing_env" / "build-opt"))
sys.path.insert(0, str(REPO / "routing_env" / "python"))

import routing_env as rc  # noqa: E402
from justroute.prepare import strip_routing  # noqa: E402
from justroute.cli import _tuned_protocol  # noqa: E402
from justroute.engine import route_best  # noqa: E402

HARVEST = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def main() -> int:
    cache = REPO / "tools" / "spread_boards.json"
    boards = json.loads(cache.read_text()) if cache.exists() else []
    checked = passed = 0
    for name in boards:
        src = HARVEST / name
        if not src.exists() or checked >= 4:
            continue
        text = strip_routing(src.read_text(encoding="utf-8", errors="replace"),
                             forms=("segment", "via", "arc", "group"))
        env = rc.RoutingEnv(2, 10, 10, 0.05, 5.0, 1.0, 0.1)
        try:
            env.load_kicad_pcb_info(text, 0.05, skip_poured=True)
        except RuntimeError:
            continue
        _tuned_protocol(env)
        b = env.board()
        st = route_best(env, 20.0, log=lambda m: None)["stats"]
        if st.unrouted_count == 0:
            continue        # genome loop not exercised -> not a probe of the bug
        live = b.collect_stats(False)
        ok = (st.unrouted_count == live.unrouted_count
              and st.total_unconnected_pins == live.total_unconnected_pins)
        checked += 1
        passed += ok
        print(f"[{checked}] {'PASS' if ok else 'FAIL'} {name[:48]:48} "
              f"reported={st.unrouted_count}/{st.total_unconnected_pins} "
              f"live={live.unrouted_count}/{live.total_unconnected_pins}",
              flush=True)
    print(f"\nengine retention: {passed}/{checked} reported==live")
    return 0 if passed == checked and checked else 1


if __name__ == "__main__":
    sys.exit(main())
