# JustRoute in KiCad: interface design (2026-09-06)

The guiding question: what does a KiCad user actually experience between
"my board is placed" and "my board is routed"? This document is the
design rationale for every interface decision, current and planned.

## The three journeys we design for

1. **"Just route it."** A hobbyist finishes placement and wants copper.
   They will not read documentation. Their first click must produce a
   good result, and a bad first experience is unrecoverable — they
   uninstall.
2. **"Fill in around my work."** An engineer hand-routes what matters
   (RF, supplies, a diff pair) and wants the remaining 80% routed
   without their work being touched, at the widths their netclasses
   specify.
3. **"Iterate."** Route, dislike a region, rip it up in the editor,
   re-route. The tool must compose with KiCad's native editing loop
   instead of imposing its own.

## Principles

**Speak KiCad, not JustRoute.** Every concept a user must know already
exists in KiCad: *netclasses* set widths and clearances (we read the
project file — never a parallel width config); *existing copper* is law
(incremental loader); *selection* is scope; *undo* is the escape hatch
(the whole job is ONE commit); *the ratsnest* reports what remains
(we never fake connectivity). A user who knows KiCad already knows how
to drive this router.

**Progressive disclosure.** Layer 0: one click, zero configuration —
auto grid resolution from measured pad pitch, budget scaled to board
size (1 s/net, clamped 60..600 s), netclasses from the sidecar
`.kicad_pro` automatically. Layer 1: a `justroute.json` next to the
project for the persistent 10% (`budget_s`, `effort`, `resolution`).
Layer 2: environment variables and the CLI for developers and CI.

**Honesty over polish.** When routing is partial: the unrouted nets are
listed BY NAME in the output, their airwires stay visible in the editor,
and the copper that WAS placed is complete and rule-clean (the
certification gates guarantee "zero new DRC violations" is the norm,
not the goal). No progress bar theater: real phase logs (first pass,
genome cycles, negotiation) with measured times.

**Never surprise.** Append-only file writes (the user's file layout,
comments, and ordering survive byte-for-byte outside the added copper);
one undo step for the entire routing job; idempotent re-runs (routing a
routed board adds nothing — gate A3); a second run after a manual
rip-up routes exactly the ripped nets (incremental by construction).

## What exists today

| Journey moment | Interface | Status |
|---|---|---|
| First click | "Route board" action, zero config | shipped |
| Widths per function | `.kicad_pro` netclasses -> per-net-pair physics + emitted widths/vias | shipped (this round) |
| Protect hand-routing | existing tracks/vias/rule areas are fixed obstacles | shipped |
| Scope to selection | routes ONLY the selected nets | shipped (this round) |
| Sane time budget | 1 s/net auto-scale, config/env override | shipped (this round) |
| Persistent prefs | `justroute.json` (budget_s, effort, resolution) | shipped (this round) |
| Failure guidance | API-server help text, unrouted-net names | shipped (this round) |
| Undo | single commit via IPC | VALIDATED live (2026-09-06): 917 items, one undo step, DRC-clean |
| Headless/CI | `JustRoute` CLI, human summary by default (`--json` for machines), same auto-budget rule | shipped |
| Live progress + cancel | tkinter window (stdlib-only): live phase log, elapsed, Cancel-keeps-best; cancel aborts the in-flight pass in ~0.2s via a cross-thread stop flag the C++ deadline checks | VALIDATED live (2026-09-06): cancelled at 20s -> 986 items pushed, second run resumed cleanly |
| Resume after cancel / partial rip-up | pre-routed = copper CONNECTS ALL PADS (per-net union-find over pads+tracks+vias; pour-owning nets trusted); partial nets stay routable and get completed pad-to-pad around their stubs | VALIDATED live: 24 complete kept, 2 partial completed, 20 fresh routed; verdicts matched KiCad's own airwire list exactly |
| Already-routed board | graceful no-op ("every net is already routed"), never an error | shipped (this round) |
| Live pass progress | "14/45 · 26s" in the window title bar while a pass runs (atomic nets-done/total counter published by the route loops, polled by the UI) | VALIDATED live (2026-09-06) |
| Pour-fed nets | nets owned by a copper pour are left to their zones BY DEFAULT, reported by name ("1 pour-fed net left to its zone: GND"); `route_poured_nets` config / `--route-poured` CLI flag to route them | VALIDATED live: zero GND track spaghetti on a poured board |
| Settings without a text editor | "Autorouter settings" action: stdlib tkinter form (budget, effort, pour-fed nets, grid) writing `justroute.json`; only non-default keys are saved | VALIDATED live: round-trips existing config |
| Result looks right immediately | copper pours auto-refill after the push (a stale hatched zone otherwise needs a manual `B`); best-effort, its own undo step | shipped (round 3) |
| What to do about unrouted nets | the summary points to raising the budget or selecting-and-re-running, not just the names | shipped (round 3) |
| Discoverability | a plugin README (two actions, config table, selection/pour behavior) ships in the PCM package | shipped (round 3) |
| Professional-looking copper | post-route smoothing straightens the grid staircase into straight segments, validated DRC-clean (51-board kicad-cli cert, zero new violations); on by default, `--no-smooth` to disable | shipped (round 4) |
| Rounded corners (optional) | corner fillets as KiCad-native arc tracks, validated the same way; off by default, `--fillet [mm]` / settings dialog / `fillet_radius_mm` to enable | shipped (round 4) |

## Planned (in order of user value)

1. **Differential pairs / length tuning** — recognized in the file model but not
   routed as pairs; out of scope until the core supports coupled-pair
   search. Documented limitation, not a silent gap.

## Anti-goals

- No custom width/rule configuration language — netclasses are the one
  source of truth; if KiCad can't express it, we read KiCad's own custom
  DRC rules someday rather than inventing our own.
- No modal blocking UI in the editor; the plugin is an external process
  and behaves like one.
- No partial commits: either the routing job lands as one undoable
  commit, or the file fallback is written and the board is untouched.
