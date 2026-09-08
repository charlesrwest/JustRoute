"""Regression: name-keyed copper net refs mark nets pre-routed.

KiCad 10 saves board copper with `(net "NAME")` (name-keyed) instead of
`(net N)`. The loader resolves name keys to synthetic NEGATIVE codes; the
pre-routed scan once filtered with `n > 0`, silently dropping every
name-keyed net — a routed board re-loaded as fully UNROUTED, with its own
copper as anonymous blocking obstacles (found live in the KiCad GUI loop,
2026-09-06: second run re-routed and duplicated already-routed nets).

Protocol: take routed corpus boards (numeric refs), rewrite ONLY the root
copper's net refs to name-keyed form, and require identical pre-routed
accounting from both texts.

Usage: python3 tests/test_namekeyed_preroute.py [n_boards]
"""

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "routing_env" / "build-opt"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import routing_env as rc  # noqa: E402

CORPUS = Path.home() / "storage2" / "pcb_board_harvest" / "raw"


def name_key_copper(text: str) -> str | None:
    """Rewrite root segment/via/arc `(net N)` -> `(net "NAME")`.

    Returns None when the board declares no usable net table.
    """
    names = {}
    for m in re.finditer(r'\(net (\d+) "((?:[^"\\]|\\.)*)"\)', text):
        names[m.group(1)] = m.group(2)
    if not names:
        return None

    out = []
    pos = 0
    # Root copper forms are indented with exactly one tab/two spaces in
    # KiCad files; matching net refs inside any segment/via/arc block is
    # simpler and equally correct here because pads spell `(net N "NAME")`
    # (three tokens) while copper spells `(net N)` (two).
    for m in re.finditer(r"\(net (\d+)\)", text):
        nm = names.get(m.group(1))
        if nm is None or nm == "":
            continue
        out.append(text[pos:m.start()])
        out.append('(net "' + nm.replace('\\', '\\\\').replace('"', '\\"') + '")')
        pos = m.end()
    out.append(text[pos:])
    return "".join(out)


def accounting(text: str) -> tuple[int, int]:
    env = rc.RoutingEnv(2, 10, 10, 0.05, 5.0, 1.0, 0.1)
    info = env.load_kicad_pcb_info(text, 0.05)
    return int(info.get("pre_routed_nets", 0)), len(env.board().nets())


def main() -> int:
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    boards = sorted(CORPUS.glob("*.kicad_pcb"))
    tested = passed = 0
    for p in boards:
        if tested >= want:
            break
        text = p.read_text(encoding="utf-8", errors="replace")
        if "(segment" not in text or '(net "' in text.split("(segment")[1][:400]:
            continue  # unrouted, or already name-keyed
        keyed = name_key_copper(text)
        if keyed is None:
            continue
        try:
            base = accounting(text)
        except Exception:
            continue  # boards the loader rejects are out of scope here
        if base[0] == 0:
            continue  # no pre-routed nets to compare
        tested += 1
        got = accounting(keyed)
        ok = got == base
        passed += ok
        print(f"{'PASS' if ok else 'FAIL'} {p.name[:60]}: "
              f"numeric(pre={base[0]}, routable={base[1]}) "
              f"name-keyed(pre={got[0]}, routable={got[1]})")
    print(f"\n{passed}/{tested} boards agree")
    return 0 if tested and passed == tested else 1


if __name__ == "__main__":
    sys.exit(main())
