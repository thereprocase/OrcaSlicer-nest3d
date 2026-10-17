# Repro development builds

`repro-dev` is this fork's integration and default branch. It includes the clean Nest3D arranger, the CLI automation/bridge features described in [agent-cli.md](agent-cli.md), and shared GUI/CLI retraction protection. Both interfaces are built from the same sources. Historical `nest3d`, rewrite and clean branches remain available as development history; the integration branch uses the tested upstream-candidate implementation already in `agent`.

GUI retraction controls are under **Printer → Extruder → Retraction** in Advanced mode, with filament controls under **Setting overrides → Retraction**. GUI defaults preserve existing profiles. Headless slicing supplies the documented local starting policy unless `--retraction-guard 0` is given. The CLI also offers automation controls without a corresponding GUI button, including bridge upload and machine-readable reports.

## Builds and downloads

Pushes and pull requests targeting `repro-dev` run **Repro build and test**. The multi-platform pipeline builds the graphical and command-line application for Linux x86_64/aarch64, Windows x64/arm64 and universal macOS, including platform test artifacts. Linux also exercises the CLI retraction policy with the built application. The existing profile slice checks and Flatpak checks remain enabled.

At **02:15 UTC daily**, the default branch builds this integration branch. Integration pushes and manual builds use the same graph. The final publisher waits for every listed platform build, unit-test, Flatpak and profile-slice gate, then publishes the artifacts of that exact run to [Repro development nightly](https://github.com/thereprocase/OrcaSlicer-nest3d/releases/tag/repro-nightly). Pull requests and dependency-only runs never publish. Superseded integration pushes are cancelled.

Each published build is first retained under `repro-build-RUN_ID-COMMIT` (12-character commit prefix), with installers, portable bundles and checksums. An interrupted draft can resume; a completed archive is left unchanged. The rolling `repro-nightly` tag then points to the newest published passing build. The README links directly to the nightly, archive and CI runs.

`SHA256SUMS` accompanies downloads; the release notes and tag identify the tested commit. macOS fork builds use the unsigned packaging path. **Republish verified Repro build** is a manual recovery operation for an existing successful integration build, not an automatic second publication pipeline. It runs the publisher from the selected workflow revision against the verified build artifacts and their original commit/run ID. Use `archive_only` to retain an older passing build without changing the rolling nightly.

The archive publisher creates or verifies the Git tag first, then creates the release with `--verify-tag`. This avoids GitHub's workflow-token restriction on release creation with an explicit older `target_commitish`. Both lightweight and annotated existing tags must resolve to the verified build SHA; mismatches stop publication.

Windows packaging uses the official NSIS 3.13 portable archive with its SHA-256 pinned from [NSIS release data](https://github.com/NSIS-Dev/release-data/blob/main/data/versions.json), avoiding the Chocolatey feed. External CLI regression fixtures are pinned in `scripts/ci/orca-test-repo.ref`. Flatpak ships and runs the standalone local API transport and client tests as well as the core suites.

Upstream issue assignment, AI deduplication, vendor merge delegation, OTA publication, WinGet publication, translation publication and the redundant status-badge workflow are archived outside the active workflows directory. Build/test coverage remains active; no upstream maintainer list or upstream release destination controls this fork.

Only the publisher job can write release contents; normal builds use read-only repository contents. External action versions used by this build/check workflow chain are pinned to full commit SHAs, recorded in [action-pins.json](action-pins.json). Inherited signing steps retain their upstream-only guards.

The upstream Nest3D PR remains separate and is not opened by these builds. No printer job, local installed GUI replacement or bridge deployment is performed by the nightly publisher.

## Local checks

Use the owner's serialized build wrapper and resource limits on the owner machine. The focused C++ tests carry the `RetractionLimiter` tag in `libslic3r` and `fff_print`. The real CLI test is registered as `cli_retraction_guard` with labels `CLI;RequiresApp;RetractionLimiter`; it requires the built application and shipped profiles. It can also run directly:

```sh
build/package/libexec/orca-slicer-env python3 tests/cli/test_retraction_guard.py \
  "$PWD/build/src/Release/orca-slicer" "$PWD/resources/profiles/BBL"
```

The historical local `tests/cli-agent` packing fixture still has a known one-plate packing limitation. The retraction checks do not establish physical heat-creep immunity. Explicit flattened machine/process recipes also need independent inspection of variant-dependent machine limits and the purge matrix after a project is reopened and re-sliced.
