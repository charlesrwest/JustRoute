"""Locate and import the compiled routing core across Python versions / OSes.

The core is a pybind11 extension (`routing_env`), so a build is specific to
one (OS, CPU arch, CPython minor). A PCM package aiming at wide compatibility
therefore bundles SEVERAL binaries, named by CPython's own ABI tag, e.g.
    routing_env.cpython-311-x86_64-linux-gnu.so     (Linux, CPython 3.11)
    routing_env.cpython-312-darwin.so               (macOS, CPython 3.12)
    routing_env.cp313-win_amd64.pyd                 (Windows, CPython 3.13)
This module picks the one matching the running interpreter — first by letting
the normal import machinery try (a correctly-named file on sys.path wins), then
by scanning the search dirs for an exact ABI match, and finally by failing with
a message that names the interpreter and platform we looked for (never a bare
ImportError, which tells a user nothing actionable).

`load_core(extra_dirs=...)` returns the imported module. `describe_target()`
returns the (tag, platform) we require, for diagnostics and packaging.
"""

from __future__ import annotations

import importlib
import importlib.util
import os
import sys
import sysconfig
from pathlib import Path


def describe_target() -> tuple[str, str]:
    """(cpython abi tag, platform slug) the running interpreter needs, e.g.
    ("cp311", "linux-x86_64"). Matches how the wheels/binaries are named."""
    tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    plat = sysconfig.get_platform().replace(".", "-")  # e.g. macosx-11-arm64
    return tag, plat


def _matches_running(fname: str) -> bool:
    """True if an extension filename is loadable by THIS interpreter.

    CPython names extensions with an EXT_SUFFIX it will accept, e.g.
    `.cpython-311-x86_64-linux-gnu.so` / `.cp311-win_amd64.pyd`; the exact
    suffix for the running interpreter is sysconfig's EXT_SUFFIX. A file that
    ends with it is guaranteed importable here. We also accept the abi-only
    infix (cpython-311 / cp311) as a looser cross-arch fallback.
    """
    suffix = sysconfig.get_config_var("EXT_SUFFIX") or ""
    if suffix and fname.endswith(suffix):
        return True
    ver = f"{sys.version_info.major}{sys.version_info.minor}"
    return (f"cpython-{ver}-" in fname or f".cp{ver}-" in fname
            or f"-cp{ver}-" in fname)


def _search_dirs(extra_dirs) -> list[Path]:
    dirs: list[Path] = []
    for d in extra_dirs or ():
        if d:
            dirs.append(Path(d))
    env = os.environ.get("JUSTROUTE_LIB")
    if env:
        dirs.append(Path(env))
    here = Path(__file__).resolve()
    dirs.append(here.parent)              # alongside justroute/
    dirs.append(here.parent.parent)       # the plugin dir (parent of justroute)
    # repo dev build
    dirs.append(here.parents[2] / "routing_env" / "build-opt")
    # de-dup, keep order, keep only existing
    seen, out = set(), []
    for d in dirs:
        rd = str(d)
        if rd not in seen and d.is_dir():
            seen.add(rd)
            out.append(d)
    return out


def find_core_binary(extra_dirs=None) -> Path | None:
    """The bundled routing_env binary matching this interpreter, if any."""
    for d in _search_dirs(extra_dirs):
        for f in sorted(d.iterdir()):
            n = f.name
            if not (n.startswith("routing_env") and
                    (n.endswith(".so") or n.endswith(".pyd"))):
                continue
            if _matches_running(n):
                return f
    return None


def available_binaries(extra_dirs=None) -> list[str]:
    """Every routing_env binary present, for diagnostics."""
    out = []
    for d in _search_dirs(extra_dirs):
        for f in sorted(d.iterdir()):
            n = f.name
            if n.startswith("routing_env") and (n.endswith(".so") or n.endswith(".pyd")):
                out.append(n)
    return out


class CoreUnavailable(RuntimeError):
    pass


def load_core(extra_dirs=None):
    """Import and return the routing core, or raise CoreUnavailable with an
    actionable message naming the interpreter + platform and what was found."""
    # 1) normal import path (a matching file already reachable on sys.path)
    for d in _search_dirs(extra_dirs):
        sd = str(d)
        if sd not in sys.path:
            sys.path.insert(0, sd)
    try:
        return importlib.import_module("routing_env")
    except ImportError:
        pass

    # 2) explicit ABI-matched file, loaded by path
    match = find_core_binary(extra_dirs)
    if match is not None:
        try:
            spec = importlib.util.spec_from_file_location("routing_env", str(match))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)        # type: ignore[union-attr]
            sys.modules["routing_env"] = mod
            return mod
        except Exception as e:                  # a matched file that still won't load
            raise CoreUnavailable(_diagnostic(extra_dirs, str(e))) from e

    # 3) nothing usable — say exactly why
    raise CoreUnavailable(_diagnostic(extra_dirs, None))


def _diagnostic(extra_dirs, err: str | None) -> str:
    tag, plat = describe_target()
    have = available_binaries(extra_dirs)
    lines = [
        "JustRoute: no routing-core binary for this interpreter.",
        f"  need: {tag} on {plat} "
        f"(Python {sys.version_info.major}.{sys.version_info.minor}, "
        f"{sysconfig.get_platform()})",
    ]
    if have:
        lines.append("  bundled binaries: " + ", ".join(have))
        lines.append("  -> none match; run KiCad's plugin with a matching "
                     "Python, or install a build for this platform.")
    else:
        lines.append("  no routing_env*.so/.pyd found next to the plugin — "
                     "the package may be missing this platform's binary.")
    if err:
        lines.append(f"  (load error: {err})")
    return "\n".join(lines)
