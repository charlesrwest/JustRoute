"""Regression: unrouted-net NAMES must survive engine reordering.

RouteStats.unrouted is indexed by CURRENT net position, while
info["net_names"] is keyed by load order (== stable net id). The engine
reorders nets (topo order, genome search, blame reorder), so position-indexed
name lookups attributed failures to the WRONG nets — the CLI's "unrouted:"
list and the plugin's "unrouted nets:" line were affected (writer was not:
it keys by net.id).

The invariant: for every position i with st.unrouted[i], the reported name is
names[nets[i].id], and looking that name back up by id lands on a net that
really has unconnected pins.
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
from rl.topo import topo_order, apply_net_order  # noqa: E402

HARVEST = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def main() -> int:
    cache = REPO / "tools" / "spread_boards.json"
    boards = json.loads(cache.read_text()) if cache.exists() else []
    checked = 0
    for name in boards:
        src = HARVEST / name
        if not src.exists():
            continue
        text = strip_routing(src.read_text(encoding="utf-8", errors="replace"))
        env = rc.RoutingEnv(2, 10, 10, 0.05, 5.0, 1.0, 0.1)
        try:
            info = env.load_kicad_pcb_info(text, 0.05, skip_poured=True)
        except RuntimeError:
            continue
        b = env.board()
        apply_net_order(env, topo_order(b))       # force displacement
        nets = b.nets()
        displaced = sum(1 for i, n in enumerate(nets) if n.id != i)
        if displaced == 0:
            continue                              # no reorder -> nothing to test
        b.set_route_time_budget_s(4.0)
        st = b.route_all()
        if not st.unrouted_count:
            continue
        names = info.get("net_names", [])
        nets = b.nets()
        id2pos = {n.id: i for i, n in enumerate(nets)}
        ok = True
        for i, u in enumerate(st.unrouted):
            if not u:
                continue
            nid = nets[i].id
            # the fixed mapping: reported name = names[nid]; its net must
            # really be unrouted (round-trip through the id)
            assert id2pos[nid] == i
            if nets[i].unconnected_pins <= 0:
                print(f"FAIL {name[:40]}: pos {i} (id {nid}, "
                      f"'{names[nid] if nid < len(names) else '?'}') reported "
                      f"unrouted but has 0 unconnected pins")
                ok = False
        checked += 1
        print(f"[{checked}] {'PASS' if ok else 'FAIL'} {name[:48]:48} "
              f"displaced={displaced} unrouted={st.unrouted_count}", flush=True)
        if not ok:
            return 1
        if checked >= 6:
            break
    print(f"\nunrouted-name mapping: {checked} reordered boards verified")
    return 0 if checked else 1


if __name__ == "__main__":
    sys.exit(main())
