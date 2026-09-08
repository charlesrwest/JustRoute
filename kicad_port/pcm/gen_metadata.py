#!/usr/bin/env python3
"""Fill a release's metadata.json download fields from the built PCM zip.

PCM requires each version entry to carry the archive's download URL, SHA256,
compressed size, and uncompressed (install) size. Computing those by hand is
error-prone; this derives them from the actual zip so the numbers are always
right. Intended for CI (see pcm-release.yml) but runnable locally.

  gen_metadata.py --zip dist/justroute-pcm-0.1.0.zip \
                  --tag v0.1.0 --repo owner/justroute --out dist/metadata.json

The base metadata (name/description/author/license) comes from the checked-in
metadata.json; only the first versions[] entry is completed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def install_size(zip_path: Path) -> int:
    with zipfile.ZipFile(zip_path) as z:
        return sum(i.file_size for i in z.infolist())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--zip", type=Path, required=True)
    ap.add_argument("--tag", required=True, help="git tag, e.g. v0.1.0")
    ap.add_argument("--repo", required=True, help="owner/name on GitHub")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--base", type=Path, default=HERE / "metadata.json")
    args = ap.parse_args()

    meta = json.loads(args.base.read_text(encoding="utf-8"))
    zp = args.zip
    url = (f"https://github.com/{args.repo}/releases/download/"
           f"{args.tag}/{zp.name}")

    if not meta.get("versions"):
        meta["versions"] = [{}]
    v = meta["versions"][0]
    v["download_url"] = url
    v["download_sha256"] = sha256(zp)
    v["download_size"] = zp.stat().st_size
    v["install_size"] = install_size(zp)
    # NOTE: status is intentionally NOT auto-promoted. It reflects the
    # checked-in metadata (testing/stable/...), a deliberate maturity call —
    # a tagged build does not by itself make a package "stable".

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    print(f"  version:        {v.get('version')}")
    print(f"  download_url:   {url}")
    print(f"  download_size:  {v['download_size']}")
    print(f"  install_size:   {v['install_size']}")
    print(f"  download_sha256:{v['download_sha256']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
