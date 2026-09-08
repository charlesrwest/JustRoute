# kicad_port — KiCad integration of the pi router

Working tree for turning the router into something KiCad would accept,
per `docs/project/kicad-integration-report.md` (2026-08-31). File-first,
three phases; this folder is Phase A and the substrate for B/C.

## Phase A — headless companion tool (current)

`JustRoute in.kicad_pcb -o out.kicad_pcb`: read a `.kicad_pcb` natively
(existing loader), route, write tracks/vias back as standard s-exprs,
certify with `kicad-cli pcb drc`. KiCad is only the judge — no KiCad
code linked, no third-party dependencies (standing rule: the library
stays standalone).

Layout:

- `justroute/writer.py` — routed Board state -> `(segment ...)` /
  `(via ...)` s-exprs spliced into the input file. Exact inverse of the
  loader transform: `mm = origin + cell * resolution` where origin is
  `KicadPcbInfo.border_min_{x,y}` (the loader's post-margin minimum; no
  y-flip — KiCad file y already grows downward). Net codes are mapped
  through `KicadPcbInfo.original_net_ids` (board net *id* -> file net
  number; board ids are compacted build indices, NOT file numbers).
  Collinear cell runs merge into single segments; layer transitions at
  a fixed (x,y) become through-vias spanning first..last copper layer.
- `justroute/certify.py` — `kicad-cli pcb drc --format json` harness
  with the baseline-diff protocol (decided 2026-08-29, ~1.4 s/board):
  DRC the input, DRC the routed output, attribute only the delta. Gate:
  zero new violations, unconnected items strictly reduced.
- `justroute/cli.py` — the `JustRoute` entry point. `--effort fast` is a
  single tuned-greedy pass; `--effort full` (default, 120s budget) runs
  `engine.py`: best-state retention + blame-directed reorder rounds
  (probe_blockers -> evict blockers -> full repass) + the PathFinder
  negotiation finisher for small residues (the corpus runner's
  ABL_NEGOTIATE recipe). A/B at equal budget: full is never worse
  (retention) and posts real wins (altimeter 138->71 unconnected pins).
- `tests/` — round-trip gates on harvested corpus boards.

### Gate A1 (writer v1) — PASSING 50/50 (2026-08-31)
Round-trip on harvested boards (`tests/gate_a1.py N SEED WORKDIR`):
strip existing routing -> route (tuned greedy, 60s budget) -> write ->
three-way `kicad-cli pcb drc` (stripped baseline vs routed vs the
original human routing as reference; only violations involving a
Track/Via item count; new types excused when the shipped human copper
incurs at least as many; new warnings reported, never failing) ->
re-import with our loader. Seed 0: 20/20; seed 1: 30/30; 22/50 fully
routed by the plain greedy. The shakedown fixed fourteen real model
bugs — see the 2026-08-31 sections of
docs/project/failure-taxonomy.md.

### Gate A2 (incremental) — PASSING 20/20 (2026-09-02)
`tests/gate_a2.py`: strip HALF of a routed board's nets, route them
around the surviving copper, certify. Exercises the plugin's core use
case ("route the rest of my board"). The loader treats nets with root
segment/via/arc copper as PRE-ROUTED: dropped from the routable set,
their copper/pads loaded as fixed obstacles, the file's routing kept.
Copper pours are NOT obstacles (KiCad refills; certification passes
--refill-zones). Rule areas apply at any nesting depth (board-frame
pts). Netclass clearance = MAX across populated classes (uniform-model
conservative; true per-net-pair physics is step 3). Model routes with
a half-cell safety margin so continuous emitted geometry (snap stubs,
diagonals) stays rule-clean between cell centers.

### .kicad_pro netclass support (2026-09-06)
KiCad 6+ keeps netclass definitions AND assignments in the sidecar
`<project>.kicad_pro`, not the pcb — `justroute/project.py` parses it
(v6 nets arrays, v7+ wildcard netclass_patterns, v8/9 assignments) and
feeds the same per-net-pair physics tiers and per-net emission the
embedded tables use. Auto-detected next to the board (CLI `--project`
overrides); the plugin gets it for free since live sessions always
have the project file. Verified end-to-end with a synthetic sidecar:
pattern-matched Power nets routed AND emitted at 0.5mm beside 0.25mm
signals, certified clean by kicad-cli WITH the sidecar visible to the
judge. UX design rationale: docs/project/kicad-ux.md; justroute.json
config, size-scaled budgets, selection-scoped routing v1, friendly
errors, unrouted-nets-by-name reporting.

### Per-netclass physics (uniform-model projection, 2026-09-03)
The MODEL routes at the max width/clearance/via across populated
classes (one swath must satisfy every pair); the WRITER emits each net
at its OWN class geometry (a 0.6mm power net gets its 0.6mm track and
class via, signals get Default's). Emitted copper never exceeds the
modeled swath, so certified clearances hold a fortiori. Class
membership is matched by net name (add_net), including v11 name-keyed
files. Verified on the mixed 4-class NixieShield: 415 power segments
at 0.6mm beside 0.2mm signals, zero new violations.

### Per-net-pair physics (2026-09-03) — DONE
Core physics tiers: Default class is tier 0; wider classes get their
own claim masks stamped at exact pair radii. NixieShield unrouted
14 -> 11; fully-routed counts up ~50% across gates (the class-max
projection taxed every net on mixed boards). Via emergence and
negotiation remain max-tier (safe). Single-class boards take legacy
paths bit-identically.

### Phase A remaining (from the report)
5. DRC-parity summary doc (the gates already enumerate divergences).

## Phase B — IPC plugin (skeleton, 2026-09-03)

`pcm/` holds the KiCad >= 9 plugin package:

- `pcm/plugins/justroute_plugin/plugin.json` — IPC plugin manifest
  (validated against go.kicad.org/api/schemas/v1: python runtime, one
  `route-board` action scoped to pcb).
- `.../route_action.py` — the action: save the open board -> route the
  file headlessly with the proven justroute engine -> push tracks/vias
  back through `board.create_items()` inside ONE commit (single undo
  step). Falls back to writing `<name>.routed.kicad_pcb` next to the
  board if the IPC write-back fails.
- `pcm/metadata.json` — PCM package metadata (BSD-3-Clause).

Verification status: everything headless is tested (the action's
routing path runs the gate-proven engine; all kipy names used —
Track/Via fields, Vector2.from_xy, BoardLayer enums, Board.get_nets/
create_items/begin_commit/push_commit/save, Net.name — are validated
against installed kicad-python). The live-GUI round trip
(KICAD_API_SOCKET session + create_items rendering) is the one part
that needs a human with a running KiCad; see the flow comments in
route_action.py. Binary distribution of the C++ core per-platform is
the known PCM packaging risk (report: Phase B workplan item 7).

## Requirements

- Python >= 3.10, the built `routing_env` module on `PYTHONPATH`
  (`routing_env/build-opt`).
- `kicad-cli` >= 10 on PATH for certification (tested against 10.0.4).

## License

BSD-3-Clause (see the repository LICENSE file). GPL-compatible, so the
PCM plugin and any eventual upstreaming into KiCad (GPLv3) are both
clean. The embedded newstroke font subset is CC0.
