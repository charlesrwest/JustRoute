"""Packaging / wide-compatibility regression tests (no network, no build).

Covers the pieces that decide whether a released PCM package installs and
runs across platforms/Python versions:
  - coreloader picks the ABI-matching binary and fails legibly otherwise;
  - metadata.json is PCM-schema-shaped (and, if jsonschema + the cached
    schema are present, fully valid);
  - gen_metadata fills every download field a release needs.

Usage: python3 tests/test_pcm_package.py
"""

import json
import re
import subprocess
import sys
import sysconfig
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
KP = ROOT / "kicad_port"
PCM = KP / "pcm"
sys.path.insert(0, str(KP))

from justroute import coreloader as cl  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    print(f"{'PASS' if cond else 'FAIL'} {name}" + (f" — {detail}" if detail else ""))
    if not cond:
        FAILS.append(name)


def test_coreloader():
    tag, plat = cl.describe_target()
    check("describe_target tag", re.fullmatch(r"cp3\d\d?", tag) is not None, tag)

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        ver = f"{sys.version_info.major}{sys.version_info.minor}"
        suffix = sysconfig.get_config_var("EXT_SUFFIX")
        # a file that THIS interpreter can load
        (d / f"routing_env{suffix}").write_bytes(b"\x7fELF stub")
        # foreign ones
        (d / "routing_env.cpython-38-x86_64-linux-gnu.so").write_bytes(b"x")
        (d / "routing_env.cp39-win_amd64.pyd").write_bytes(b"x")
        cl._search_dirs = lambda extra: [d]  # type: ignore
        picked = cl.find_core_binary()
        check("find_core_binary picks matching ABI",
              picked is not None and cl._matches_running(picked.name),
              picked.name if picked else "None")
        check("does NOT match foreign 3.8",
              not cl._matches_running("routing_env.cpython-38-x86_64-linux-gnu.so"))

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        (d / "routing_env.cpython-38-x86_64-linux-gnu.so").write_bytes(b"x")
        cl._search_dirs = lambda extra: [d]  # type: ignore
        try:
            cl.load_core()
            check("graceful error on no-match", False, "loaded unexpectedly")
        except cl.CoreUnavailable as e:
            msg = str(e)
            check("graceful error names target + what's bundled",
                  tag in msg and "cpython-38" in msg)


def test_metadata():
    meta = json.loads((PCM / "metadata.json").read_text())
    for key in ("name", "description", "description_full", "identifier",
                "type", "author", "license", "versions"):
        check(f"metadata has '{key}'", key in meta)
    v = meta["versions"][0]
    check("version matches PCM pattern",
          re.fullmatch(r"\d{1,2}(\.\d{1,2}(\.\d{1,2})?)?", v["version"]) is not None)
    check("kicad_version present + valid",
          re.fullmatch(r"\d{1,2}(\.\d{1,2}(\.\d{1,2})?)?", v.get("kicad_version", "")) is not None)
    check("no empty kicad_version_max (must be omitted, not '')",
          v.get("kicad_version_max", None) != "")
    check("platforms declared", set(v.get("platforms", [])) <= {"linux", "macos", "windows"}
          and bool(v.get("platforms")))

    # full schema validation when the tooling + cached schema are available
    schema_f = PCM / "resources" / "pcm-v1-schema.json"
    try:
        import jsonschema
    except Exception:
        print("SKIP full schema validation (jsonschema not installed)")
        return
    if not schema_f.exists():
        print("SKIP full schema validation (no cached schema)")
        return
    schema = json.loads(schema_f.read_text())
    try:
        jsonschema.validate(meta, schema)
        check("metadata validates against cached PCM v1 schema", True)
    except jsonschema.ValidationError as e:
        check("metadata validates against cached PCM v1 schema", False, e.message)


def test_version_consistency():
    meta_v = json.loads((PCM / "metadata.json").read_text())["versions"][0]["version"]
    pyproj = (ROOT / "routing_env" / "pyproject.toml").read_text()
    m = re.search(r'^version\s*=\s*"([^"]+)"', pyproj, re.M)
    check("wheel version matches PCM metadata version",
          m is not None and m.group(1) == meta_v,
          f"pyproject={m.group(1) if m else '?'} metadata={meta_v}")


def test_gen_metadata():
    # build a tiny fake zip and check gen_metadata fills the download block
    with tempfile.TemporaryDirectory() as td:
        import zipfile
        z = Path(td) / "justroute-pcm-0.1.0.zip"
        with zipfile.ZipFile(z, "w") as zf:
            zf.writestr("metadata.json", "{}")
        out = Path(td) / "metadata.json"
        r = subprocess.run(
            [sys.executable, str(PCM / "gen_metadata.py"), "--zip", str(z),
             "--tag", "v0.1.0", "--repo", "owner/justroute", "--out", str(out)],
            capture_output=True, text=True)
        ok = r.returncode == 0 and out.exists()
        check("gen_metadata runs", ok, r.stderr.strip()[:80])
        if ok:
            v = json.loads(out.read_text())["versions"][0]
            for f in ("download_url", "download_sha256", "download_size", "install_size"):
                check(f"gen_metadata fills {f}", f in v)
            check("release status not 'development'", v.get("status") != "development")


def main() -> int:
    test_coreloader()
    test_metadata()
    test_version_consistency()
    test_gen_metadata()
    print(f"\n{'ALL PASS' if not FAILS else str(len(FAILS)) + ' FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
