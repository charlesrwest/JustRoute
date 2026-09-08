#!/usr/bin/env python3
"""Assemble a PCM package from one or more prebuilt routing_env wheels.

Every wheel handed in contributes its compiled core binary
(routing_env.*.so / .pyd) to a SINGLE package; the plugin's ABI-aware
loader (justroute.coreloader) picks the right one at runtime. So:

  - locally:  build one wheel for this interpreter, get a dev package.
  - in CI:    collect wheels for every (OS, CPython) from the matrix and
              assemble ONE universal zip that installs everywhere.

Usage:
  assemble_pcm.py --out dist/ WHEEL [WHEEL ...]
  assemble_pcm.py --out dist/ --wheels-dir /path/to/collected/wheels

The pure-Python parts (justroute, rl.topo, the action scripts, metadata,
LICENSE) are taken from the repo tree next to this script.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
PLUGIN_SRC = HERE / "plugins" / "justroute_plugin"

# pure-python files copied verbatim into the plugin dir
PLUGIN_FILES = ["plugin.json", "route_action.py", "progress_ui.py",
                "settings_action.py", "requirements.txt", "README.md"]


def _version() -> str:
    meta = json.loads((HERE / "metadata.json").read_text(encoding="utf-8"))
    vs = meta.get("versions") or [{}]
    return vs[0].get("version", "0.0.0")


def _core_binary_in(wheel: Path) -> str | None:
    with zipfile.ZipFile(wheel) as z:
        for n in z.namelist():
            base = n.rsplit("/", 1)[-1]
            if base.startswith("routing_env") and (base.endswith(".so")
                                                    or base.endswith(".pyd")):
                return n
    return None


def _extract_core(wheel: Path, dest: Path) -> str:
    inner = _core_binary_in(wheel)
    if inner is None:
        raise SystemExit(f"no routing_env binary inside {wheel.name}")
    base = inner.rsplit("/", 1)[-1]
    with zipfile.ZipFile(wheel) as z, z.open(inner) as src, \
            open(dest / base, "wb") as out:
        shutil.copyfileobj(src, out)
    return base


def _stage_plugin(plugin_dir: Path) -> None:
    plugin_dir.mkdir(parents=True, exist_ok=True)
    for f in PLUGIN_FILES:
        src = PLUGIN_SRC / f
        if src.exists():
            shutil.copy2(src, plugin_dir / f)
    # the justroute package (writer, cli, engine, coreloader, project, ...)
    shutil.copytree(REPO / "kicad_port" / "justroute", plugin_dir / "justroute",
                    dirs_exist_ok=True)
    # rl.topo (net ordering) as a minimal rl package
    (plugin_dir / "rl").mkdir(exist_ok=True)
    (plugin_dir / "rl" / "__init__.py").write_text("", encoding="utf-8")
    shutil.copy2(REPO / "routing_env" / "python" / "rl" / "topo.py",
                 plugin_dir / "rl" / "topo.py")
    # drop bytecode caches
    for pyc in plugin_dir.rglob("__pycache__"):
        shutil.rmtree(pyc, ignore_errors=True)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("wheels", nargs="*", type=Path)
    ap.add_argument("--wheels-dir", type=Path, default=None,
                    help="add every routing_env-*.whl in this directory")
    ap.add_argument("--out", type=Path, default=HERE / "dist")
    args = ap.parse_args()

    wheels = list(args.wheels)
    if args.wheels_dir:
        wheels += sorted(args.wheels_dir.glob("routing_env-*.whl"))
    if not wheels:
        ap.error("no wheels given (positional or --wheels-dir)")

    args.out.mkdir(parents=True, exist_ok=True)
    version = _version()

    with tempfile.TemporaryDirectory() as td:
        stage = Path(td) / "stage"
        plugin = stage / "plugins" / "justroute_plugin"
        _stage_plugin(plugin)
        shutil.copy2(HERE / "metadata.json", stage / "metadata.json")
        lic = REPO / "LICENSE"
        if lic.exists():
            shutil.copy2(lic, plugin / "LICENSE")
        changelog = REPO / "kicad_port" / "CHANGELOG.md"
        if changelog.exists():
            shutil.copy2(changelog, plugin / "CHANGELOG.md")
        # PCM icon: bundled at resources/icon.png in the package root
        icon = HERE / "resources" / "icon.png"
        if icon.exists():
            (stage / "resources").mkdir(exist_ok=True)
            shutil.copy2(icon, stage / "resources" / "icon.png")

        bins = []
        for w in wheels:
            bins.append(_extract_core(w, plugin))
        bins = sorted(set(bins))
        print(f"bundled {len(bins)} core binaries:")
        for b in bins:
            print("   ", b)

        pkg = args.out / f"justroute-pcm-{version}.zip"
        pkg.unlink(missing_ok=True)
        with zipfile.ZipFile(pkg, "w", zipfile.ZIP_DEFLATED) as z:
            for f in sorted(stage.rglob("*")):
                if f.is_file():
                    z.write(f, f.relative_to(stage).as_posix())

        size = pkg.stat().st_size
        # install size = uncompressed total (PCM uses this for the progress bar)
        install = sum(f.stat().st_size for f in stage.rglob("*") if f.is_file())
        print(f"\nbuilt {pkg}")
        print(f"  download_size:  {size}")
        print(f"  install_size:   {install}")
        print(f"  download_sha256:{_sha256(pkg)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
