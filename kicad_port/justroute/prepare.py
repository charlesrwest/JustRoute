"""Strip existing routing from a .kicad_pcb (test prep / re-route workflows).

Removes root-level (segment ...), (via ...), (arc ...) — track arcs, not
footprint graphics, which live inside (footprint ...) and are untouched —
plus (zone ...) and (group ...). Copper-pour zones go because the router
doesn't model pours yet (Phase A step 2 honors them as obstacles instead);
RULE-AREA zones — (zone ... (keepout ...)) — are KEPT: they are design
rules, not routing, and the loader models them. Groups go because they may
reference removed uuids.

This mirrors the FreeRouting-style "unroute then autoroute" workflow and
is how gate A1 builds unrouted test inputs from harvested boards, which
are nearly all fully routed.
"""

from __future__ import annotations

STRIP_FORMS = ("segment", "via", "arc", "zone", "group")
COPPER_FORMS = ("segment", "via", "arc")

_NET_RE = None


def _form_net(form_text: str) -> int:
    """The (net N) number of a copper form, or 0."""
    global _NET_RE
    if _NET_RE is None:
        import re
        _NET_RE = re.compile(r"\(net\s+(\d+)\s*\)")
    m = _NET_RE.search(form_text)
    return int(m.group(1)) if m else 0


def routed_nets(text: str) -> set:
    """File net numbers that own root-level routed copper (segment/via/arc)."""
    import re
    out = set()
    for m in re.finditer(r"\n\s*\((segment|via|arc)[\s(]", text):
        j = _balanced_end(text, text.index("(", m.start() + 1))
        out.add(_form_net(text[m.start():j + 1]))
    out.discard(0)
    return out


def _balanced_end(text: str, open_pos: int) -> int:
    d = 0
    j = open_pos
    n = len(text)
    while j < n:
        c = text[j]
        if c == '"':
            j += 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
        elif c == "(":
            d += 1
        elif c == ")":
            d -= 1
            if d == 0:
                return j
        j += 1
    return n - 1


def _form_name(text: str, open_pos: int) -> str:
    i = open_pos + 1
    while i < len(text) and text[i].isspace():
        i += 1
    j = i
    while j < len(text) and not text[j].isspace() and text[j] not in "()":
        j += 1
    return text[i:j]


def strip_routing(text: str, forms: tuple = STRIP_FORMS,
                  only_nets: set | None = None) -> str:
    """Remove matching top-level forms (depth 1 under kicad_pcb), string-aware.

    With `only_nets`, strips ONLY copper forms (segment/via/arc) whose (net N)
    is in the set — pours, groups, and other nets' routing stay untouched.
    This builds partial boards for incremental-routing tests (gate A2).
    """
    if only_nets is not None:
        forms = COPPER_FORMS
    out = []
    depth = 0
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            out.append(text[i:j + 1])
            i = j + 1
            continue
        if c == "(":
            name = _form_name(text, i) if depth == 1 else ""
            if depth == 1 and name in forms:
                # find the whole balanced form, string-aware
                d = 0
                j = i
                while j < n:
                    cj = text[j]
                    if cj == '"':
                        j += 1
                        while j < n and text[j] != '"':
                            j += 2 if text[j] == "\\" else 1
                    elif cj == "(":
                        d += 1
                    elif cj == ")":
                        d -= 1
                        if d == 0:
                            break
                    j += 1
                keep = (name == "zone" and "(keepout" in text[i:j + 1])
                if only_nets is not None and name in COPPER_FORMS \
                        and _form_net(text[i:j + 1]) not in only_nets:
                    keep = True   # another net's copper: incremental strip spares it
                if keep:
                    out.append(text[i:j + 1])
                    i = j + 1
                    continue
                i = j + 1
                # swallow the trailing newline/indent the form owned
                while i < n and text[i] in " \t":
                    i += 1
                if i < n and text[i] == "\n":
                    i += 1
                continue
            depth += 1
            out.append(c)
            i += 1
            continue
        if c == ")":
            depth -= 1
        out.append(c)
        i += 1
    return "".join(out)
