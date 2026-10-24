#!/usr/bin/env python3
"""Prepare dependency metadata for a cache restored at a new prefix."""

import argparse
from pathlib import Path
import subprocess


def prepare_config(prefix: Path, output: Path, pkgconfig: Path) -> None:
    prefix = prefix.resolve()
    configs = prefix / "lib/wx/config"
    default = prefix / "bin/wx-config"
    if default.is_symlink():
        # wxWidgets installs an absolute symlink, which still names the original runner.
        selected = configs / default.readlink().name
    else:
        candidates = sorted(path for path in configs.iterdir() if path.is_file())
        if len(candidates) != 1:
            raise ValueError(f"Cannot select the cached wxWidgets configuration in {configs}")
        selected = candidates[0]
    if not selected.is_file():
        raise FileNotFoundError(f"Cached wxWidgets configuration is missing: {selected}")

    # --prefix overrides do not fix absolute dependency paths in the linker flags.
    # Rewrite a temporary copy, leaving the restored vendor files unchanged.
    original = subprocess.check_output([str(selected), "--prefix"], text=True).strip()
    if not original or not Path(original).is_absolute():
        raise ValueError(f"Invalid cached wxWidgets prefix: {original!r}")
    output.write_text(selected.read_text().replace(original, str(prefix)))
    output.chmod(0o755)

    # FFmpeg and transitive GTK dependencies also embed the install prefix.
    # Prefer these copies through PKG_CONFIG_PATH during CMake configuration.
    pkgconfig.mkdir(parents=True, exist_ok=True)
    for directory in ("lib/pkgconfig", "lib64/pkgconfig", "share/pkgconfig"):
        for source in (prefix / directory).glob("*.pc"):
            metadata = source.read_text().replace(original, str(prefix))
            metadata = metadata.replace("${pcfiledir}", str(source.parent))
            (pkgconfig / source.name).write_text(metadata)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("pkgconfig", type=Path)
    args = parser.parse_args()
    prepare_config(args.prefix, args.output, args.pkgconfig)
