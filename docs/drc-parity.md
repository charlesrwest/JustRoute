# DRC parity: our model vs kicad-cli (Phase A deliverable, 2026-09-04)

The integration report asked for a divergence enumeration between our
internal DRC/model and KiCad's own checker. It was produced the hard way:
four iterated certification gates (three from-scratch seeds + one
incremental) across ~160 distinct harvested boards, each divergence
root-caused and fixed until every sampled board certified with zero new
violations. This document is the summary; the mechanism-by-mechanism
record lives in failure-taxonomy.md (2026-08-31 .. 2026-09-03 sections).

## End state

- Gates: **110/110 boards certify clean** (A1 seeds 0/1/7: 20+30+40,
  A2 incremental: 20) — zero new Track/Via-involving violations of any
  type, judged by `kicad-cli pcb drc --severity-all --refill-zones`
  against a stripped baseline and the original human routing.
- Every dialect from v4 (2017) through the v11 nightly (2026, name-keyed
  nets) loads and routes.

## Divergence classes found and closed (26 total)

Geometry/model (our DRC could never see these — it validates against the
same model the router uses; kicad-cli validates the model itself):
1.  clearance truncation ((int) vs ceil — model ran 0.15mm claiming 0.2)
2.  pad rotation sign (y-down frame; visible only at 45 degrees)
3.  rotated-pad scan boxes (max vs hypot — corners unstamped)
4.  pin-cell quantization (terminals jut past fine-pitch pads)
5.  snap stubs tilting long runs (mid-span dips)
6.  snap stubs running to pad centers (0.49mm-pitch neighbors)
7.  diagonal snap dips (res*sqrt(2)/2 > half-cell margin)
8.  scanline fill rows missing polygon tips (10um keepout escape)
9.  arc/circle chord sagitta (0.5mm inside a 60mm outline)

Unmodeled objects (obstacles KiCad knows that we didn't load):
10. no-net pads (mounting holes, NPTH, fiducials)
11. board outline / Margin edge bands (with stroke width)
12. copper-layer graphics (gr_*/fp_* strokes, filled polygons)
13. copper text (justify-aware conservative bboxes)
14. graphic + far-side mask apertures (solder_mask_bridge)
15. keepout rule areas (tracks/vias, any nesting depth, board-frame pts)
16. existing tracks/vias/arcs (incremental pre-routed obstacles)
17. single-sided THT holes (drilled pads with partial copper layers)
18. power/mixed-type copper layers + renamed-layer aliases

Rules (KiCad rules we didn't read or apply):
19. netclass table (clearance/width/via per class -> per-net-pair tiers)
20. per-pad/footprint (clearance) and (solder_mask_margin) overrides
21. setup minima + pad_to_mask_clearance
22. hole-clearance floors incl. (drill (offset ...)) pads
23. same-net hole-to-hole via spacing (drill rules are net-blind)
24. edge-clearance rule (0.5mm vs copper clearance)

File semantics:
25. v11 name-keyed nets (no net codes anywhere)
26. mangled net tables -> fidelity-reject (KiCad reconciles legacy
    duplicates by rules we refuse to guess)

## Judge protocol (what "parity" means operationally)

Three-way `kicad-cli pcb drc`: stripped baseline vs our routed output vs
the original human routing. Only violations involving an item the router
created (Track/Via) count; a new violation type is excused when the
shipped human copper incurs at least as many; new warnings never fail
the gate; pours are refilled before judging. This attributes exactly the
router's own copper, symmetric across all three reports.

## Glyph stroking (2026-09-04)

Copper text is now REAL geometry: the ASCII subset of KiCad's newstroke
font (CC0, 3.3KB embedded header) is stroked into per-segment capsule
obstacles with justify/mirror/rotation/multiline handling, CALIBRATED
against kicad-cli's own SVG renders of single-text test boards (worst
inside shortfall 0.031mm, covered by stroke thickness). Two conventions
were only discoverable by measurement: glyph y is negated vs the raw
font table, and a -0.64*fh baseline offset applies. Mirror comes ONLY
from the explicit justify token (back-copper text carries it in the
file; layer-based auto-mirror double-flips). Knockout (inverted) text
blocks its full plate box. The SVG-calibration harness pattern is the
tool for any future text-convention question.

## Steady-state expectation

Fresh 50-board samples certify at 96-100%; the residual is a long tail
of per-board exotica (1-3 violations each) that shuffles with any
routing change. Tail round of 2026-09-04 closed four more classes:
v5-era text baseline (dialect-dependent -0.265*fh vs v6+'s -0.64*fh,
measured against the board's own render), trapezoid (rect_delta) pads,
DIMENSION annotations on copper (v4/5 feature lines and v6+ derived
chords + nested rotated text), and a stroke end-cap slack. 2026-09-05
round: terminal geometry became foreign-aware (junction stub
suppression, no-stub-vs-clipped-stub scoring against nearby pads,
NECK-DOWN when the pad is narrower than the track — the interactive-
router convention); v5 text calibration is transform-ambiguous, so v5
text strokes a dual (plain) or quad (rotated) union of the candidate
conventions. Five-gate sweep: 159/160; finder list: Blast-Furnace
(one shorting_item, v5 rotated-text corner) and Phaser_Overdrive (v11
nightly, THT/snap-stub interactions — 2026-09-05, under diagnosis).
Gate A3 (idempotency): JustRoute on its own output adds zero copper;
barrel-connected nets (all pins one true-center drilled column) load
as fixed obstacles. Auto-resolution: boards with sub-0.45mm pad pitch route
on the 0.025mm grid (fine-pitch QFNs have ZERO clearance slack at 0.05
— same board, same route, clean at 0.025). Stale netclass via dims that
violate KiCad's 0.1mm min-annular are emitted with minimally grown
sizes.

## Live-GUI validation (2026-09-06)

Ran the real IPC round trip against a headless-launched KiCad 10 pcbnew
(API server enabled, board open). Three findings, all fixed or logged:
- board.name returns the FILENAME only; the full path is
  document.project.path + document.board_filename (fixed in the plugin);
- the Board binding is a non-owning reference into RoutingEnv — returning
  it while env is GC'd dangles (std::bad_alloc on .nets()); the action now
  keeps env alive (fixed);
- KiCad rewrites the sidecar .kicad_pro on save (its own serialization),
  so netclasses must be registered in KiCad, not just present in the file
  — correct behavior, worth knowing for tests.
Result: 917 tracks/vias created as ONE undo step, net assignments correct,
kicad-cli DRC clean except one warning-severity same-net hole-to-hole
(two vias 0.177mm apart — rare emission edge, logged).

## Live-GUI iteration round 2 (2026-09-06): cancel/resume loop

Drove cancel-mid-route -> resume -> selection-scope against the live GUI.
Two MODEL bugs no corpus gate had caught (both fixed + regression-tested):

- **Name-keyed copper never marked nets pre-routed.** KiCad 10 saves
  board copper as `(net "NAME")`; the resolver maps names to synthetic
  NEGATIVE codes, and the pre-routed scan filtered `n > 0` — so a board
  KiCad itself had saved re-loaded as fully UNROUTED with its own copper
  as anonymous obstacles (second run duplicated already-routed nets).
  Fix: any nonzero key counts, and declared names now seed their positive
  codes so mixed numeric/name references unify.
  `tests/test_namekeyed_preroute.py` requires identical pre-routed
  accounting for numeric vs name-keyed forms of the same board (12/12).
- **"Any copper => pre-routed" trapped partially-routed nets.** A
  cancelled run (or a user's partial rip-up) leaves nets with dangling
  copper; the old rule dropped them from routing forever. Pre-routed now
  requires the net's copper to CONNECT ALL ITS PADS (strict per-net
  union-find over pads + tracks + vias, rotated-rect pad distance;
  a doubtful touch counts as NOT connected — the safe direction).
  Nets owning a pour are trusted (the refilled zone completes them —
  measured: the human jackco original is 0-unconnected only thanks to
  its GND pours). Partial nets stay routable and are completed
  pad-to-pad around their own stubs (same-net detours, never shorts).
  Verdicts matched KiCad's own airwire list exactly on the live board
  (22 incomplete nets, both sides).

A fully-routed board is now a graceful no-op end to end ("every net is
already routed"), and cancel aborts the in-flight pass in ~0.2s via a
cross-thread stop flag polled by the same deadline checks A* already
does every 1024 expansions. Final live state after cancelled run +
selection run + resume run: 1122 tracks / 28 vias, zero DRC errors from
routed copper (2 warning-severity same-net hole-to-hole, known residual).

## Round 3 (2026-09-06): pour-fed nets skipped by default

The plugin and CLI now load with `skip_poured=True`: a net owning a
copper pour is treated as pre-routed even without tracks (the refilled
zone is trusted to connect it) and reported by name. This kills the
classic autorouter surprise — track spaghetti across a ground-poured
board. `route_poured_nets` in justroute.json / `--route-poured` restores
the old behavior; direct `load_kicad_pcb_info` calls (campaigns, RL)
default to false and are unchanged.

## Round 4 (2026-09-06): post-route trace smoothing

Grid routing renders a shallow slope as a staircase of tiny H/diagonal
steps (on the jackco board, 112 of 151 runs on +5V were sub-0.2mm).
`Board::smooth_paths(max_dev)` replaces each net's per-layer vertex chain
with the fewest straight segments that (a) keep every skipped vertex
within `max_dev` cells of the chord (a load-bearing detour is never
shortcut away — its vertices are far from the chord) and (b) whose swept
copper is DRC-safe.

Safety is structural, not a post-hoc check. A chord's copper (the
half-width disk along its centreline) may only cover cells that are
EITHER (a) already this net's original copper footprint, OR (b) genuinely
free — outside every FOREIGN clearance zone / pad keepout and not already
reserved by another net's smoothing this pass. (a) is essential: at
minimum spacing a trace's own copper edge sits ON the neighbour's zone
boundary, so a naive "avoid foreign zones" test rejects the net's own
straight run and explodes the output into per-cell fragments. (b) keeps
any NEW copper >= clearance from all foreign copper (zones are baked at
foreign_hw + clearance) and, via a shared reservation mask, stops two
nets corner-cutting into the same gap. No grid mutation; original clean
copper plus only-into-free moves can never raise a violation.

Proven: 51 corpus boards, each written raw and smoothed, kicad-cli DRC on
both — the smoothed report had <= the raw report's Track/Via violations
of every (type, severity) on ALL 51 (boards with pre-existing violations
kept the identical count: Mimikyu 1->1, Project_OAK 4->4). Geometry on
jackco: ~53% fewer segments at max_dev=1.5. On by default in the CLI and
plugin; `--no-smooth` / `JUSTROUTE_SMOOTH=0` / `"smooth": false` disables.
Regression tests: tests/test_smoothing_safe.py (kicad-cli), and the
writer only swaps in smoothed runs when Board::has_smoothed() (untouched
otherwise, so the gates are unaffected).

Corner rounding (optional): `smooth_paths(max_dev, fillet_radius_cells>0)`
also replaces each interior corner with a tangent circular arc — the
KiCad-native smooth-curve primitive (`(arc ...)` track). Each arc is
validated by the SAME per-cell own-footprint-or-free check as a straight
segment (supersampled along the arc centreline), so it cannot introduce a
violation; the writer emits `(arc (start)(mid)(end))` and the IPC plugin
creates kipy ArcTrack items. Off by default (square corners);
`--fillet [mm]` / `"fillet_radius_mm"` / JUSTROUTE_FILLET / the settings
dialog turn it on. Electrically a wash for these boards (a 45° chamfer
already removes the corner's excess copper) — it is an aesthetic option.

## Residual known gaps (deliberate)

- Copper pours are not routing obstacles (KiCad refills around new
  tracks; certification refills before judging).
- Between two above-Default netclasses the pair clearance is exact;
  via emergence and negotiation use max-tier standoffs (conservative).
- Custom DRC rules (the KiCad rule language) are not interpreted.
- Project-file (.kicad_pro) settings are invisible when judging a bare
  .kicad_pcb — kicad-cli itself has the same limitation.
- Text obstacles are conservative bboxes, not stroked glyphs.
