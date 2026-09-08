"""Regression: skip_poured leaves pour-fed nets to their zones.

A net that owns a copper pour is completed by the refilled zone, not by
tracks — routing tracks all over it is the classic autorouter surprise.
With skip_poured=True (the KiCad-plugin/CLI default) such a net is
pre-routed even without any track copper, and reported by name in
pour_fed_nets. With skip_poured=False (campaigns/RL) it stays routable.

This also guards the invariant that skip_poured NEVER changes the verdict
for a net that has no pour: only pour-owning nets move.

Usage: python3 tests/test_pour_skip.py [n_boards]
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "routing_env" / "build-opt"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import routing_env as rc  # noqa: E402
from justroute.prepare import strip_routing  # noqa: E402

CORPUS = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def acct(text: str, skip: bool):
    env = rc.RoutingEnv(2, 10, 10, 0.05, 5.0, 1.0, 0.1)
    info = env.load_kicad_pcb_info(text, 0.05, skip_poured=skip)
    return (info.get("pre_routed_nets", 0),
            list(info.get("pour_fed_nets", []) or []),
            len(env.board().nets()))


def main() -> int:
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    boards = sorted(CORPUS.glob("*.kicad_pcb"))
    tested = passed = with_pour = 0
    for p in boards:
        if tested >= want:
            break
        text = p.read_text(encoding="utf-8", errors="replace")
        # a zone with a real net owner is what skip_poured acts on
        if not re.search(r'\(zone\b', text):
            continue
        # strip loose copper but KEEP zones, so pour-only completion is the
        # only thing that can mark a net pre-routed
        try:
            stripped = strip_routing(text, forms=("segment", "via", "arc"))
            noskip = acct(stripped, False)
            skip = acct(stripped, True)
        except RuntimeError:
            continue  # boards the loader rejects (fidelity guard) are out of scope
        tested += 1

        ok = True
        # every reported pour-fed net must actually be newly pre-routed
        if len(skip[1]) != skip[0] - noskip[0]:
            ok = False
        # skip never routes MORE than no-skip; the delta is exactly the pours
        if skip[2] != noskip[2] - len(skip[1]):
            ok = False
        # names must be non-empty and unique
        if len(set(skip[1])) != len(skip[1]) or any(not n for n in skip[1]):
            ok = False
        if skip[1]:
            with_pour += 1
        passed += ok
        tag = f" pour-fed={skip[1][:4]}" if skip[1] else " (no pour-fed nets)"
        print(f"{'PASS' if ok else 'FAIL'} {p.name[:52]}: "
              f"noskip(pre={noskip[0]},routable={noskip[2]}) "
              f"skip(pre={skip[0]},routable={skip[2]}){tag}")
    print(f"\n{passed}/{tested} boards consistent; "
          f"{with_pour} exercised a real pour-fed skip")
    return 0 if tested and passed == tested and with_pour else 1


if __name__ == "__main__":
    sys.exit(main())
