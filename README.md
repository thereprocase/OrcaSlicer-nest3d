# Repro OrcaSlicer

**[Latest passing nightly](https://github.com/thereprocase/OrcaSlicer-nest3d/releases/tag/repro-nightly)** · **[Build archive](https://github.com/thereprocase/OrcaSlicer-nest3d/releases)** · **[CI builds in progress](https://github.com/thereprocase/OrcaSlicer-nest3d/actions/workflows/build_all.yml?query=branch%3Arepro-dev)**

An OrcaSlicer fork for repeatable, inspectable slicing and local automation. It combines the normal desktop slicer with Nest3D plate arrangement, profile-management commands, a local GUI API, and retraction controls shared by the GUI and CLI.

This is **thereprocase’s development fork**, not an official OrcaSlicer release. It retains Orca’s project/profile compatibility and credits its upstream authors. Printer operation remains an explicit user action; building or testing this repository does not send a print.

[![Repro build and test](https://github.com/thereprocase/OrcaSlicer-nest3d/actions/workflows/build_all.yml/badge.svg?branch=repro-dev)](https://github.com/thereprocase/OrcaSlicer-nest3d/actions/workflows/build_all.yml?query=branch%3Arepro-dev)

## Downloads

The nightly link always points to the latest published passing build. Each published build also has a retained `repro-build-RUN_ID-COMMIT` release with its own downloads and checksums. Completed archived releases are left unchanged. CI artifacts are temporary and may require GitHub sign-in; use releases for bookmarks and retained downloads.

[Repro development builds](https://github.com/thereprocase/OrcaSlicer-nest3d/releases/tag/repro-nightly) contain the graphical and command-line application for Linux x86_64/aarch64, Windows x64/arm64, and universal macOS, plus Linux Flatpak packages. Windows portable archives include application resources. macOS builds are unsigned.

A download is published only after its commit passes the platform builds, unit tests, CLI regression tests, and profile-slicing checks. The release identifies that commit and includes `SHA256SUMS`. A green source build is not a certification of a printer, material, or print recipe. If the latest build fails, the last successful release stays available.

## What this fork adds

| Capability | What it is for | Documentation |
|---|---|---|
| Nest3D arranging | Pack parts onto plates using the integrated alternative arranger | [Development overview](docs/repro-development.md) |
| CLI automation | Inspect and prepare projects with explicit inputs and machine-readable results | [CLI reference](docs/agent-cli.md) |
| Profile management | Resolve, inspect and manage presets without guessing flattened inheritance | [Profile commands](docs/agent-cli.md) |
| Local GUI API | Work with an existing GUI session through an authenticated local client | [GUI client](scripts/agent_gui.py) |
| Retraction controls | Limit optional travel retractions; preserve required material-change behavior | [Policy and limits](docs/repro-development.md) |

Retraction controls are under **Printer → Extruder → Retraction** in Advanced mode and **Filament → Setting overrides → Retraction**. GUI defaults preserve existing profiles. Headless slicing has a documented starting policy; `--retraction-guard 0` disables injection of that policy. It does not erase settings already stored in a project.

## Build, test and contribute

`repro-dev` is the integration branch. Base fork changes and pull requests on it. See [development and release details](docs/repro-development.md), [repository conventions](AGENTS.md), and [test conventions](tests/AGENTS.md).

The **Repro build and test** workflow is the release gate. It builds and tests the same commit, then publishes its existing artifacts. Scheduled builds run at 02:15 UTC; manual and integration-push runs use the same checks. External regression fixtures are pinned in [orca-test-repo.ref](scripts/ci/orca-test-repo.ref), so an upstream test change cannot silently change our gate.

Useful checks without a full application build:

```sh
python3 tests/cli/test_ci_contract.py
python3 tests/cli/test_nightly_publisher.py
python3 tests/cli/test_agent_gui.py
```

For a configured Release build with tests enabled:

```sh
cmake --build build --config Release
ctest --test-dir build/tests -C Release --output-on-failure
```

Use the platform build instructions in [the upstream developer guide](https://www.orcaslicer.com/wiki/developer_reference/). Preserve project compatibility and add regression coverage for changes to settings, geometry or automation behavior.

## Upstream and license

Built on [OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer), with its Bambu Studio, PrusaSlicer and Slic3r ancestry and the many contributors credited by those projects. General slicer documentation is available in the [OrcaSlicer wiki](https://www.orcaslicer.com/wiki). For official upstream releases, use [OrcaSlicer’s downloads](https://github.com/OrcaSlicer/OrcaSlicer/releases).

The source remains under [GNU AGPLv3](LICENSE.txt); bundled dependencies retain their own licenses. The [upstream README](https://github.com/OrcaSlicer/OrcaSlicer#readme) contains the broader upstream feature list and acknowledgements. Archived upstream repository-management workflows live under `.github/upstream-workflows/` and are not active in this fork.
