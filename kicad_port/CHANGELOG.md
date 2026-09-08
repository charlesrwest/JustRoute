# Changelog — JustRoute KiCad plugin

## 0.1.0 (unreleased) — first PCM candidate

First public build. Batch autorouter that runs as a KiCad IPC plugin.

### Features
- **Route board** action: routes every unrouted net around existing
  copper, honoring netclasses and rule areas, and writes the result back
  as a single undoable commit; copper pours are refilled afterward.
- **Selection scope**: with tracks/pads/nets selected, routes only those.
- **Incremental**: fully-routed nets are left alone; partially-routed
  nets (a cancelled run, a manual rip-up) are completed around their
  existing copper.
- **Pour-fed nets** (GND planes, etc.) are left to their zones by default
  and reported by name; `route_poured_nets` routes them anyway.
- **Live progress window**: phase log, a net-by-net progress bar, and a
  Cancel button that keeps everything routed so far (stops in ~0.2 s).
- **Trace smoothing**: the grid staircase is straightened into DRC-clean
  segments (on by default). **Corner rounding** with arc fillets is
  available as an option (off by default).
- **Autorouter settings** action: a small dialog that writes
  `justroute.json` (budget, effort, pour-fed nets, grid, corner rounding).
- **Headless CLI** (`justroute.cli`) with the same engine, for CI/scripts.

### Quality
- Every release is judged by KiCad's own DRC. Across the certification
  gates (samples of harvested real boards) the routed copper adds zero
  new DRC errors; measured ~0.85x human wirelength and ~0.76x human vias.
- Trace smoothing certified DRC-clean on 51 boards; arc fillets on 30+.

### Known limitations / residuals
- **Same-net hole-to-hole (warning)**: in rare layouts the router can
  place a via whose hole is closer than the hole-to-hole rule to an
  existing same-net hole. KiCad rates this a *warning*, not an error; it
  does not affect connectivity. A targeted fix is planned.
- **Differential pairs / length tuning** are not routed as coupled pairs
  (recognized in the file model, but out of scope for this engine).
- **Custom DRC rules** (the KiCad rule language) are not interpreted;
  netclass widths/clearances and rule areas are.
- A small number of exotic boards are deliberately **rejected** with a
  plain-English reason rather than routed to a lower-fidelity result.

### Compatibility
- KiCad 9+ (IPC API). Arc-fillet write-back to the live board needs
  KiCad 10; the file output uses arcs on 9+ as well.
- The compiled core ships per (OS, CPython) inside one package; the
  plugin loads the binary matching KiCad's Python at runtime, and reports
  a clear message if none matches.
