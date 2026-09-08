"""Topological pre-routing analysis: flight-line crossing graph and planar-subset
(MPS-style) net ordering.

The ordering is STRUCTURAL, not per-net triage (measured: individual crossing counts
do not predict which nets fail — the value is in minimizing global resource conflict):
Round 1 is a greedy maximal set of mutually non-crossing nets (a maximal independent
set of the circle-graph analog), routed shortest-first; Round 2 is the conflicted
remainder, least-conflicted first. Crossings are computed exactly on MST flight
segments, the same decomposition the RUDY congestion field uses.
"""
from __future__ import annotations

import math


def _mst_edges(pins):
    """Prim MST over pin (x,y) with octile distance -> [(p, q)] segment endpoints."""
    P = len(pins)
    if P < 2:
        return []
    pts = [(p.x, p.y) for p in pins]

    def octile(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + 0.41421356 * min(dx, dy)

    in_tree = [False] * P
    in_tree[0] = True
    best = [octile(pts[i], pts[0]) for i in range(P)]
    frm = [0] * P
    edges = []
    for _ in range(P - 1):
        b = min((i for i in range(P) if not in_tree[i]), key=lambda i: best[i])
        in_tree[b] = True
        edges.append((pts[frm[b]], pts[b]))
        for i in range(P):
            if not in_tree[i]:
                d = octile(pts[i], pts[b])
                if d < best[i]:
                    best[i], frm[i] = d, b
    return edges


def _seg_cross(a1, a2, b1, b2):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) > (q[1] - p[1]) * (r[0] - p[0])
    return ccw(a1, b1, b2) != ccw(a2, b1, b2) and ccw(a1, a2, b1) != ccw(a1, a2, b2)


def crossing_graph(board):
    """{net_id: set(net_ids its flight lines cross)} plus per-net flight length."""
    nets = board.nets()
    edges = {n.id: _mst_edges(n.pins) for n in nets}
    length = {nid: sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in segs)
              for nid, segs in edges.items()}
    ids = list(edges)
    conflicts = {nid: set() for nid in ids}
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if any(_seg_cross(p1, q1, p2, q2)
                   for p1, q1 in edges[a] for p2, q2 in edges[b]):
                conflicts[a].add(b)
                conflicts[b].add(a)
    return conflicts, length


def topo_order(board, r1_key="length", r2_key="conflicts_length"):
    """Net-id order: planar subset first, then the conflicted remainder.
    r1_key: 'length' (shortest-first) | 'conflicts' (fewest-crossing-first)
    r2_key: 'conflicts_length' | 'length' — sort keys are config-search knobs."""
    conflicts, length = crossing_graph(board)
    ids = sorted(conflicts, key=lambda nid: (len(conflicts[nid]), length[nid]))
    round1, excluded = [], set()
    for nid in ids:                      # greedy MIS: low-degree nets first
        if nid not in excluded:
            round1.append(nid)
            excluded.update(conflicts[nid])
    keyf = {"length": lambda nid: (length[nid],),
            "conflicts": lambda nid: (len(conflicts[nid]), length[nid]),
            "conflicts_length": lambda nid: (len(conflicts[nid]), length[nid])}
    r1 = sorted(round1, key=keyf[r1_key])
    rest = [nid for nid in conflicts if nid not in set(round1)]
    r2 = sorted(rest, key=keyf[r2_key])
    return r1 + r2


def bundle_sweep_order(board):
    """Bus-aware ordering (LLM-roleplay finding, 2026-08-16): cluster nets by
    pin-pair direction (30-degree buckets); the largest cluster is the bus.
    Route it FIRST, consecutively, sorted by its perpendicular (sweep)
    coordinate — routed into open space back-to-back, cheapest paths pack into
    parallel lanes at bare clearance (Teensy gap board: 27 stranded -> 6).
    Remaining nets follow shortest-first. Crossing-graph MIS cannot find this
    structure: parallel bus nets never cross, so MIS admits-but-scatters them."""
    nets = {n.id: n for n in board.nets()}

    def direction(n):
        pts = [(p.x, p.y) for p in n.pins]
        (x1, y1), (x2, y2) = min(pts), max(pts)
        return round(math.atan2(y2 - y1, x2 - x1) / (math.pi / 6))

    buckets = {}
    for nid, n in nets.items():
        buckets.setdefault(direction(n), []).append(nid)
    bus_dir, bus = max(buckets.items(), key=lambda kv: len(kv[1]))
    ang = bus_dir * math.pi / 6

    def sweep(nid):
        pts = [(p.x, p.y) for p in nets[nid].pins]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        return -cx * math.sin(ang) + cy * math.cos(ang)

    def length(nid):
        pts = [(p.x, p.y) for p in nets[nid].pins]
        (x1, y1), (x2, y2) = min(pts), max(pts)
        return math.hypot(x2 - x1, y2 - y1)

    return sorted(bus, key=sweep) + sorted(
        (i for i in nets if i not in set(bus)), key=length)


def apply_net_order(env, order):
    """Reorder the board's nets to `order` (net ids) via move_net. No routing."""
    for target, nid in enumerate(order):
        pos = env.position_of(nid)
        if pos >= 0 and pos != target:
            env.board().move_net(pos, target)
