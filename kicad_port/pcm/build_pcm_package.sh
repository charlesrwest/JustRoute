#!/bin/bash
# Build a PCM-installable zip for THIS platform (dev/local convenience).
#
# For a WIDE-compatibility release, don't use this — let CI build wheels for
# every (OS, CPython) and run assemble_pcm.py over all of them (see
# pcm-release.yml). This script just builds one wheel for the current
# interpreter and assembles a single-binary package for quick local testing.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
OUT="${1:-$HERE/dist}"

WHEEL_DIR="$(mktemp -d)"
trap 'rm -rf "$WHEEL_DIR"' EXIT
( cd "$REPO/routing_env" && pip wheel . --no-deps --no-build-isolation -q -w "$WHEEL_DIR" )

python3 "$HERE/assemble_pcm.py" --out "$OUT" "$WHEEL_DIR"/routing_env-*.whl
