#!/bin/bash
# Install the JustRoute IPC plugin into the local KiCad plugins directory.
#
# Ships everything the action needs alongside plugin.json so no JUSTROUTE_LIB
# environment is required: the justroute package, the built routing_env module,
# and the rl helpers it imports (topo). After installing:
#   1. KiCad -> Preferences -> Plugins -> enable the API server, refresh plugins
#   2. open a board in the PCB editor -> Tools/toolbar: "Route board"
# Output appears in the plugin's console; on IPC write-back failure the routed
# board is written next to the original as <name>.routed.kicad_pcb.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# KiCad's documents home varies in case across installs; prefer an existing dir.
KICAD_VER="${1:-10.0}"
for base in "$HOME/.local/share/kicad" "$HOME/.local/share/KiCad"; do
    if [ -d "$base/$KICAD_VER" ]; then DOCS="$base/$KICAD_VER"; break; fi
done
: "${DOCS:=$HOME/.local/share/kicad/$KICAD_VER}"
DEST="$DOCS/plugins/justroute_plugin"

SO=$(ls "$REPO"/routing_env/build-opt/routing_env.cpython-*.so 2>/dev/null | head -1)
if [ -z "$SO" ]; then
    echo "error: routing_env module not built (expected in routing_env/build-opt)" >&2
    exit 1
fi

mkdir -p "$DEST"
cp "$HERE/plugins/justroute_plugin/plugin.json" \
   "$HERE/plugins/justroute_plugin/route_action.py" \
   "$HERE/plugins/justroute_plugin/progress_ui.py" \
   "$HERE/plugins/justroute_plugin/settings_action.py" \
   "$HERE/plugins/justroute_plugin/requirements.txt" "$DEST/"
cp "$SO" "$DEST/"
rm -rf "$DEST/justroute" "$DEST/rl"
cp -r "$REPO/kicad_port/justroute" "$DEST/justroute"
# minimal rl subset used by the engine/CLI (tuned protocol)
mkdir -p "$DEST/rl"
cp "$REPO/routing_env/python/rl/__init__.py" "$DEST/rl/" 2>/dev/null || touch "$DEST/rl/__init__.py"
cp "$REPO/routing_env/python/rl/topo.py" "$DEST/rl/"

echo "installed to $DEST"
echo "next: enable the KiCad API server (Preferences > Plugins), restart KiCad,"
echo "open a board and run the 'Route board' action."
