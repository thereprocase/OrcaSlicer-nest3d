# Repro development builds

`repro-dev` is this fork's integration and default branch. It includes the clean Nest3D arranger, the CLI automation/bridge features described in [agent-cli.md](agent-cli.md), and shared GUI/CLI retraction protection. Both interfaces are built from the same sources. Historical `nest3d`, rewrite and clean branches remain available as development history; the integration branch uses the tested upstream-candidate implementation already in `agent`.

GUI retraction controls are under **Printer → Extruder → Retraction** in Advanced mode, with filament controls under **Setting overrides → Retraction**. GUI defaults preserve existing profiles. Headless slicing supplies the documented local starting policy unless `--retraction-guard 0` is given. The CLI also offers automation controls without a corresponding GUI button, including bridge upload and machine-readable reports.

## Builds and downloads

Pushes and pull requests targeting `repro-dev` run **Build all**. The multi-platform pipeline builds the graphical and command-line application for Linux x86_64/aarch64, Windows x64/arm64 and universal macOS, including platform test artifacts. Linux also exercises the CLI retraction policy with the built application. The existing profile slice checks and Flatpak checks remain enabled.

At **02:15 UTC daily**, the default branch's schedule builds this integration branch. A successful scheduled or manually dispatched Build all run publishes a rolling prerelease at [Repro development nightly](https://github.com/thereprocase/OrcaSlicer-nest3d/releases/tag/repro-nightly). Push CI produces run artifacts without replacing the nightly release. The publisher waits for the listed platform build, unit-test, Flatpak and profile-slice gates. `SHA256SUMS` accompanies the downloads. The release notes and tag identify the tested commit. macOS fork builds use the existing unsigned packaging path.

Only the publisher job can write release contents; normal builds use read-only repository contents. External action versions used by this build/check workflow chain are pinned to full commit SHAs, recorded in [action-pins.json](action-pins.json). Upstream's separate release and signing routes keep their upstream-only guards.

The upstream Nest3D PR remains separate and is not opened by these builds. No printer job, local installed GUI replacement or bridge deployment is performed by the nightly publisher.

## Local checks

Use the owner's serialized build wrapper and resource limits on the owner machine. The focused C++ tests carry the `RetractionLimiter` tag in `libslic3r` and `fff_print`. The real CLI test is registered as `cli_retraction_guard` with labels `CLI;RequiresApp;RetractionLimiter`; it requires the built application and shipped profiles. It can also run directly:

```sh
build/package/libexec/orca-slicer-env python3 tests/cli/test_retraction_guard.py \
  "$PWD/build/src/Release/orca-slicer" "$PWD/resources/profiles/BBL"
```

The historical local `tests/cli-agent` packing fixture still has a known one-plate packing limitation. The retraction checks do not establish physical heat-creep immunity. Explicit flattened machine/process recipes also need independent inspection of variant-dependent machine limits and the purge matrix after a project is reopened and re-sliced.
