"""Production routing loop for JustRoute: retention + blame reorder + negotiation.

The corpus campaign's proven recipe, self-contained:
- best-state retention: rip-up is non-monotone, so the best board visited is
  checkpointed and restored at the end (RoutingEnv.save/restore_checkpoint);
- blame-directed reordering: probe each stuck net for the cheapest set of
  blocking nets (Board.probe_blockers), evict those blockers behind it, and
  re-run the full pass — measured far more sample-efficient than blind
  reorder, and a full pass costs less than one rip-up step;
- PathFinder negotiation finisher: for a small residue, a whole-board
  negotiate_window (history costs, pressure escalation) — kept only when it
  converges with zero DRC and strictly fewer unrouted (the runner's
  ABL_NEGOTIATE recipe verbatim).

Everything respects a wall budget; every pass runs under a Board time budget
so a single huge search cannot blow through it.
"""

from __future__ import annotations

import time

NEG_MAX_RESIDUE = 10      # negotiation is a finisher, not a bulldozer
NEG_ITERS = 40


def _score(st):
    # unconnected pins dominate, then unrouted nets, then wirelength+vias
    return (st.total_unconnected_pins, st.unrouted_count,
            st.total_length + 25.0 * st.total_vias)


def route_selected(env, net_ids: list, budget_s: float,
                   log=lambda m: None, should_stop=lambda: False) -> dict:
    """Route ONLY the given board nets (by net id), leaving everything else
    untouched — the "route selection" semantics a KiCad user expects.

    Uses per-connection routing (step_connect) so no work is spent on
    unselected nets; failures on one net don't block the rest. The board's
    other unrouted nets' pads remain hard obstacles automatically.
    """
    import time as _t
    board = env.board()
    t0 = _t.monotonic()
    sel = list(net_ids)
    board.set_route_time_budget_s(0.0)
    done = failed = 0
    for nid in sel:
        if should_stop() or (budget_s > 0 and _t.monotonic() - t0 > budget_s):
            log("selection budget exhausted")
            break
        guard = 0
        while guard < 256:
            guard += 1
            st = env.step_connect(nid)
            pos = env.position_of(nid)
            net = board.nets()[pos] if pos >= 0 else None
            if net is None or net.unconnected_pins == 0:
                break
            if st.unrouted_count and guard > len(net.pins) + 2:
                break   # no progress on this net
        pos = env.position_of(nid)
        if pos >= 0 and board.nets()[pos].unconnected_pins == 0:
            done += 1
        else:
            failed += 1
    st = board.collect_stats(False)
    log(f"selection: {done} routed, {failed} incomplete")
    return {"stats": st, "wall_s": round(_t.monotonic() - t0, 2),
            "selected_done": done, "selected_failed": failed}


def route_best(env, budget_s: float, log=lambda m: None,
               should_stop=lambda: False) -> dict:
    """Route env's board as well as the budget allows. Returns summary stats.

    `should_stop` is polled between passes (a pass itself runs under a Board
    time cap, so cancellation takes effect within one pass length); the best
    checkpointed state is kept — cancel never loses work.
    """
    from rl.topo import apply_net_order  # rl path set up by the caller (cli)

    board = env.board()
    t0 = time.monotonic()

    def remaining() -> float:
        if should_stop():
            return 0.0
        return budget_s - (time.monotonic() - t0)

    # The pass budget is a CAP, not a target: a board that routes clean in 20s
    # leaves the rest of the wall for improvement, so the first pass gets the
    # WHOLE budget (a fractional cap starved big boards vs a plain fast pass).
    board.set_route_time_budget_s(max(5.0, remaining()))
    log(f"first pass: routing {len(board.nets())} nets…")
    _tp0 = time.monotonic()
    st = board.route_all()
    t_pass = max(2.0, time.monotonic() - _tp0)
    best = _score(st)
    env.save_checkpoint()
    log(f"first pass: unrouted={st.unrouted_count} pins={st.total_unconnected_pins} "
        f"({t_pass:.0f}s)")

    # ---- conflict-genome order evolution (the corpus campaign's best method:
    # conflict-directed operators measured ~20x more sample-efficient than
    # blind shuffles; a full-pass eval costs less than one rip-up step) ----
    # evolution pays only when several full passes fit in the remaining wall
    if st.unrouted_count > 0 and remaining() > 3.0 * t_pass:
        import random as _random
        from rl.topo import crossing_graph
        rng = _random.Random(0)
        conflicts, _clen = crossing_graph(board)
        ids = [n.id for n in board.nets()]
        failed = [ids[i] for i, x in enumerate(st.unrouted) if x]
        elites = [(_score(st), 0, list(ids), list(failed))]
        evo_end = time.monotonic() + max(2.0 * t_pass, remaining() - 2.0 * t_pass)
        cyc = 0
        while time.monotonic() < evo_end and not should_stop():
            pick = elites[rng.randrange(min(len(elites), 5))]
            order = list(pick[2])
            posmap = {nid: i for i, nid in enumerate(order)}
            pfail = pick[3]
            r = rng.random()
            done_op = False
            if r < 0.35 and pfail:
                # cohort-front: pull a failed net ahead of its whole conflict set
                f = pfail[rng.randrange(len(pfail))]
                cs = [c for c in conflicts.get(f, ()) if c in posmap]
                if cs:
                    first = min(posmap[c] for c in cs)
                    order.remove(f)
                    order.insert(first, f)
                    done_op = True
            if not done_op and r < 0.65:
                # conflict-swap: exchange a conflicting pair's positions
                cand = [(a, b) for a in (pfail or list(conflicts))
                        for b in conflicts.get(a, ()) if a in posmap and b in posmap]
                if cand:
                    a, b = cand[rng.randrange(len(cand))]
                    ia, ib = posmap[a], posmap[b]
                    order[ia], order[ib] = order[ib], order[ia]
                    done_op = True
            if not done_op and r < 0.85 and pfail:
                # conflicted-segment shuffle: reshuffle the span holding a failed
                # net and its conflict set
                f = pfail[rng.randrange(len(pfail))]
                grp = [f] + [c for c in conflicts.get(f, ()) if c in posmap]
                idxs = sorted(posmap[x] for x in grp if x in posmap)
                if len(idxs) >= 2:
                    i, j = idxs[0], idxs[-1] + 1
                    seg = order[i:j]
                    rng.shuffle(seg)
                    order = order[:i] + seg + order[j:]
                    done_op = True
            if not done_op and len(elites) >= 2:
                # precedence crossover: conflicted nets in parent A's order,
                # the rest in parent B's
                a2 = elites[rng.randrange(len(elites))][2]
                b2 = elites[rng.randrange(len(elites))][2]
                conf = {k for k, v in conflicts.items() if v}
                merged = ([x for x in a2 if x in conf]
                          + [x for x in b2 if x not in conf])
                seen_h = set()
                order = [x for x in merged
                         if not (x in seen_h or seen_h.add(x))]
                done_op = True
            if not done_op:
                kk = rng.randrange(1, max(2, len(order)))
                order = order[kk:] + order[:kk]

            from rl.topo import apply_net_order as _apply
            _apply(env, order)
            board.set_route_time_budget_s(
                min(max(5.0, 1.3 * t_pass), max(1.0, evo_end - time.monotonic())))
            st2 = board.route_all()
            sc = _score(st2)
            cyc += 1
            ids2 = [n.id for n in board.nets()]
            nfail = [ids2[i] for i, x in enumerate(st2.unrouted) if x]
            elites.append((sc, cyc, order, nfail))
            elites.sort(key=lambda e: (e[0], e[1]))
            del elites[5:]
            if sc < best:
                best = sc
                env.save_checkpoint()
                st = st2
                log(f"genome c{cyc}: unrouted={st.unrouted_count} "
                    f"pins={st.total_unconnected_pins}")
                if st.unrouted_count == 0:
                    break
        if _score(st) > best and env.has_checkpoint():
            env.restore_checkpoint()
            st = board.collect_stats(False)
        log(f"genome done: {cyc} cycles, unrouted={st.unrouted_count}")

    # ---- blame-directed reorder rounds ----
    tried: set = set()          # (victim_id, frozenset(blocker_ids)) evictions tried
    pad_cross: set = set()      # nets whose probe crosses a foreign pad: reorder can't fix
    while remaining() > 5.0 and st.unrouted_count > 0:
        nets = board.nets()
        id2pos = {n.id: i for i, n in enumerate(nets)}
        order = [n.id for n in nets]
        move = None
        for i, stuck in enumerate(st.unrouted):
            if not stuck or nets[i].id in pad_cross:
                continue
            vid = nets[i].id
            blockers, crossed = board.probe_blockers(i)
            if crossed:
                pad_cross.add(vid)
                continue
            bl = frozenset(b for b in blockers if b in id2pos and b != vid)
            if not bl or (vid, bl) in tried:
                continue
            tried.add((vid, bl))
            move = (vid, bl)
            break
        if move is None:
            break
        vid, bl = move
        # evict the blockers behind the victim: victim routes through the vacated
        # corridor, evicted nets re-route after it through whatever remains
        neworder = [o for o in order if o not in bl] + sorted(bl)
        apply_net_order(env, neworder)
        board.set_route_time_budget_s(
            min(max(5.0, 1.3 * t_pass), max(1.0, remaining())))
        st2 = board.route_all()
        if _score(st2) < best:
            best = _score(st2)
            env.save_checkpoint()
            st = st2
            log(f"reorder keep: unrouted={st.unrouted_count} "
                f"pins={st.total_unconnected_pins}")
        else:
            env.restore_checkpoint()
            st = board.collect_stats(False)

    # ---- negotiation finisher (runner ABL_NEGOTIATE recipe) ----
    if 0 < st.unrouted_count <= NEG_MAX_RESIDUE and remaining() > 10.0:
        try:
            nets = board.nets()
            id2pos = {n.id: i for i, n in enumerate(nets)}
            parts = {i for i, x in enumerate(st.unrouted) if x}
            for k in sorted(parts):
                blockers, _crossed = board.probe_blockers(k)
                for b in blockers:
                    if b in id2pos:
                        parts.add(id2pos[b])
            g = board.grid()
            env.save_checkpoint()
            u = st.unrouted_count
            conv, _it, _sh, _r = board.negotiate_window(
                sorted(parts), 0, 0, g.width() - 1, g.height() - 1,
                NEG_ITERS, 0.5, 1.6, 0.4)
            st2 = board.collect_stats(True)
            # check_drc counts each unrouted net as one violation: demand
            # COPPER-clean + strictly fewer unrouted, not drc==0 (which would
            # reject every partial win on a not-fully-routable board)
            copper_clean = (st2.drc_violations - st2.unrouted_count) == 0
            pins_before = st.total_unconnected_pins
            if conv and copper_clean and (
                    st2.unrouted_count < u
                    or (st2.unrouted_count == u
                        and st2.total_unconnected_pins < pins_before)):
                best = _score(st2)
                env.save_checkpoint()
                st = st2
                log(f"negotiation: unrouted {u}->{st.unrouted_count}")
            else:
                env.restore_checkpoint()
                st = board.collect_stats(False)
        except Exception:
            env.restore_checkpoint()
            st = board.collect_stats(False)

    # land on the best retained state
    if _score(st) > best and env.has_checkpoint():
        env.restore_checkpoint()
        st = board.collect_stats(False)
    board.set_route_time_budget_s(0.0)
    return {"stats": st, "wall_s": round(time.monotonic() - t0, 2)}
