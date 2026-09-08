# JustRoute autorouter (KiCad plugin)

Batch-routes the unrouted nets of the open board and adds the result as a
single undoable commit. CPU-only, DRC-certified engine. Existing copper is
never touched.

## Setup (once)

1. Preferences → Plugins → **Enable KiCad API server**, then restart KiCad.
2. Open a board in the PCB editor.

## Two actions (Tools → External Plugins, or the toolbar)

**Route board.** Routes every unrouted net. A progress window shows the
live phase log and a net counter; **Cancel** stops immediately and keeps
everything routed so far. When finished, the tracks/vias appear as one
undo step and copper pours are refilled.

- **Select** tracks, pads, or nets first to route *only* those nets —
  everything else is left exactly as it is.
- Nets that already have complete copper are left untouched; a net with
  partial copper (from a cancelled run or a manual rip-up) is completed
  around its existing stubs.
- Nets owned by a copper **pour** are left to their zones by default (no
  track spaghetti across a ground plane). Set `route_poured_nets` to route
  them anyway.
- Anything that couldn't be routed keeps its airwires — raise the budget
  or select those nets and re-run.
- Routed traces are smoothed (the grid staircase becomes straight
  segments) without changing any clearance — this is validated against
  KiCad's own DRC. It is on by default; set `JUSTROUTE_SMOOTH=0` to keep
  the raw grid geometry.

**Autorouter settings.** A small form that writes `justroute.json` next to
your project, so preferences persist without editing text. Everything here
is optional — the defaults are automatic.

## Preferences (`justroute.json`, next to the board)

| Key | Default | Meaning |
|---|---|---|
| `budget_s` | auto (1 s/net, 60–600) | wall-clock routing budget in seconds |
| `effort` | `full` | `full` keeps improving until the budget runs out; `fast` is one pass |
| `route_poured_nets` | `false` | also route nets owned by a copper pour |
| `resolution` | auto (0.05, or 0.025 for fine pitch) | routing grid in mm |
| `fillet_radius_mm` | 0 (square corners) | round trace corners with arcs of this radius |

Netclass widths, via sizes, and clearances come from the project's
netclasses automatically — there is no separate width configuration.

## Troubleshooting

Every failure prints a specific message; the common ones:

- **"could not connect to KiCad's API server"** — enable it under
  Preferences → Plugins ("Enable KiCad API"), restart KiCad, and open a
  board in the PCB editor.
- **"the KiCad API library (kicad-python / kipy) is not available"** —
  the plugin's Python is missing kipy: `pip install kicad-python`.
- **"no routing-core binary for this interpreter"** — the package didn't
  include a core build for the Python/OS KiCad launched the plugin with.
  The message names the exact interpreter and platform it needs; grab a
  build for that platform (or report it).
- **"nothing to route" / "could not be parsed"** — the board is already
  fully connected, has no routable pads, or the file is malformed. These
  are reported plainly, not as a crash.
- **Write-back unavailable** — if the live commit fails for any reason,
  the routed board is written next to yours as `<name>.routed.kicad_pcb`;
  the file is always the fallback.

## Privacy / safety

Runs entirely on your machine (CPU only) — nothing is uploaded. It writes
only added copper; your file layout, comments, and ordering are preserved
byte-for-byte outside the routed tracks, and the whole job is one undo
step.
