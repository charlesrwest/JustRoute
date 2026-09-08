# JustRoute

One-click autorouting for KiCad. JustRoute routes the unrouted nets of your
board around the copper you've already placed, honors your netclass rules,
and drops the result in as a single undoable commit — so wiring a board
around a microcontroller stops being a day of tedium.

BSD-3-Clause.

## What it does

- Reads `.kicad_pcb` natively (every dialect v4 → v11), treating existing
  tracks, vias, rule areas, and graphics as fixed obstacles.
- Routes **only what's unrouted**, per-netclass widths/vias, with best-state
  retention, blame-directed rip-up, and PathFinder-style negotiated
  congestion. CPU-only, headless engine.
- **Smooths** traces off the grid staircase (optional rounded corner fillets).
- Runs as a KiCad **IPC plugin**: a progress window with a cancel-keeps-best
  button, results written back as one undo step, pours refilled.
- **Judges itself with KiCad**: every release is certified with `kicad-cli
  pcb drc` across a suite of real harvested boards — zero new violations is
  the bar, not the goal.

Measured against the human reference routing on ~200 boards, emitted routes
average 0.85× the human wirelength and 0.76× the vias.

## Install

Via KiCad's Plugin & Content Manager once released, or locally:

```sh
# build the core, then install the plugin for your KiCad
cmake -S routing_env -B routing_env/build-opt -DCMAKE_BUILD_TYPE=Release
cmake --build routing_env/build-opt -j
kicad_port/pcm/install_local.sh    # KiCad >= 9, API server enabled
```

Then enable KiCad's API server (Preferences → Plugins), restart, open a
board, and run **Route board**. See
[kicad_port/pcm/plugins/justroute_plugin/README.md](kicad_port/pcm/plugins/justroute_plugin/README.md)
for usage and troubleshooting.

## Command line

```sh
cd kicad_port
PYTHONPATH=../routing_env/build-opt python3 -m justroute.cli in.kicad_pcb -o out.kicad_pcb
```

## Layout

- `routing_env/` — the standalone C++ core (no third-party solver deps) +
  pybind11 bindings.
- `kicad_port/` — the KiCad-facing product: `justroute/` (writer, certifier,
  engine, CLI), `pcm/` (IPC plugin + packaging), `tests/` (certification
  gates).
- `docs/` — how the model reaches parity with KiCad's own DRC, and the
  interface design.

## License

BSD-3-Clause (see LICENSE). The embedded newstroke stroke-font subset is CC0
from the KiCad project; bundled `third_party/` (doctest, nlohmann/json) is MIT.
