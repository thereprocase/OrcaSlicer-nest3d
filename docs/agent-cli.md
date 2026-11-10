# Command line for agents and print bridges

These options let a script or an agent drive the CLI headless: arrange parts with a known spacing, check
the placement and the slice from JSON instead of the G-code, and hand single-plate jobs to a print bridge
that sends them on to a printer. They work on top of the usual CLI run:

```bash
orca-slicer --datadir DATA \
  --load-settings "machine.json;process.json" --load-filaments filament.json \
  --arrange 1 --orient 0 --slice 0 --export-3mf job.3mf --outputdir OUT \
  part-a.stl part-b.stl
```

Files the options write (`--arrange-report`, `--summary`) follow `--export-3mf`: a relative path is
relative to `--outputdir`. `result.json` is written to `--outputdir` on Linux only, as before.

## Arranging

### `--arrange-engine stock|nest3d`

Chooses the arranger `--arrange 1` runs. `stock` (the default) is the outline arranger; `nest3d` packs
the parts' real shapes and heights (see [HLSD/nest3d.md](HLSD/nest3d.md)). By-object printing always uses
the stock arranger. When the option is absent, the `ORCA_NEST3D` environment variable still selects
Nest3D: `1` for Nest3D, `stack` for Nest3D with stacking.

### `--nest3d-stack`

Lets Nest3D place short parts under the overhangs of taller ones, with 2 mm clearance below the
overhang. Implies `--arrange-engine nest3d` and is an error with `--arrange-engine stock`.

### `--nest3d-supports`

Generates supports before arranging and includes their extrusion widths in each part's occupied
space. Support settings, painted enforcers and blockers, modifiers, and rafts come from the project.
Each offered rotation is sliced separately because support patterns depend on orientation; this
takes longer than ordinary arrangement. The option is off by default, implies
`--arrange-engine nest3d`, and is an error with `--arrange-engine stock`.

The generated geometry includes model brims and material shrink compensation. Used filaments must
have matching XY and Z shrink compensation so isolated parts and the final plate use the same scale.

It can be combined with `--nest3d-stack` and a small explicit gap:

```bash
orca-slicer ... --arrange 1 --allow-rotations --nest3d-supports --nest3d-stack --arrange-gap 0.8
```

Always slice the arranged plate. Neighboring parts can affect final support generation, so the
preliminary envelopes do not replace final toolpath clearance checks or establish support removability.

### `--arrange-gap MM`

The minimum distance between two arranged parts and between a part and the bed's excluded area, for
both arrangers. Nest3D also keeps this distance from the whole `bed_exclude_area` polygon and from the
purge line or other material the machine start G-code extrudes on the plate (see
[HLSD/nest3d.md](HLSD/nest3d.md)). Without it the spacing comes from the parts' brim and support settings (1 mm per part
without support). Parts keep 2 mm from the bed edge, as with the stock spacing, or half the gap when
that is more.

```bash
orca-slicer ... --arrange 1 --arrange-engine nest3d --arrange-gap 3 nut.stl nut.stl ring.stl
```

### `--arrange-report FILE.json`

Written after arranging (and with `--arrange 0`, for the parts as loaded). Distances are measured on the
parts' footprints, the outlines of their meshes seen from above, in plate coordinates (mm).
`generated_supports` records whether support preparation was requested. Footprint distances in this
report remain model-only; they do not measure final support clearance.

```json
{
  "engine": "nest3d", "stacking": false, "generated_supports": false, "gap_mm": 3.0, "arranged": true,
  "plate_count": 1, "plates_used": 1,
  "objects": [
    {
      "name": "nut.stl", "source_file": "/parts/nut.stl", "object_index": 0, "instance_index": 0,
      "plate": 1, "x": 61.2, "y": 140.8, "rotation_deg": 45.0,
      "footprint_bbox": {"min": [13.7, 93.3], "max": [108.7, 188.3], "size": [95.0, 95.0]},
      "footprint_area_mm2": 4012.5,
      "nearest_neighbour_mm": 3.02, "nearest_neighbour": "ring-cap-side.stl",
      "nearest_exclusion_mm": 61.0, "bed_edge_mm": 2.5
    }
  ],
  "unplaced": [],
  "bed": {"printable_area": [[0, 0], [256, 0], [256, 256], [0, 256]], "exclude_area": [[[0, 0], [18, 0], [18, 28], [0, 28]]]}
}
```

`plate` is 1-based. `x`, `y` are the instance's position (its object origin) on that plate. A distance
is `null` when there is nothing to measure against, and 0 when footprints overlap, as stacked parts do.
`unplaced` lists objects that are not entirely on any plate.

## Retraction protection

Headless `--slice` runs in this local fork enable a starting retraction policy unless
`--retraction-guard 0` is given. This switch disables the CLI defaults and respects the supplied
profiles; it does not erase protection explicitly stored in a profile. GUI defaults are off.

| Flag / printer setting | CLI default | Behavior |
|---|---|---|
| `--retraction-count-max` | `3` | Soft maximum cycles in the extrusion window; zero disables this budget. |
| `--retraction-extrusion-window` | `1` mm | Fresh filament window over which cycles are counted. |
| `--retraction-minimum-extrusion` | `0.3` mm | Hard minimum fresh filament between normal retracts, including layer changes; zero disables this minimum. |

Explicit flags override individual CLI defaults. These are per-extruder printer settings, shown
in Advanced mode beside the existing retraction controls. The corresponding nullable filament
settings (`filament_retraction_count_max`, `filament_retraction_extrusion_window`, and
`filament_retraction_minimum_extrusion`) override the printer when enabled in the filament's
Setting overrides → Retraction section. The settings persist in presets and exported projects.

Each filament has its own history. Retraction/unretraction and E-axis resets do not replenish the
budget. Wipe segments count as one cycle. The soft budget preserves outer-wall starts/ends and
outside-silhouette crossings. When the budget is exhausted, wall avoidance is attempted first; a shortcut across internal holes
is preferred to leaving the outside contour. Automatic avoidance permits at most 10 mm extra travel
when no detour limit is configured. Explicit wall avoidance retains its configured distance policy. The hard minimum takes priority over wall quality,
so it can increase stringing where a crossing is unavoidable. Z-hop remains available when
retraction is skipped. Required material-change, unload and final retractions are exempt.
Protected cycles still consume the budget. This policy controls repeated filament cycling; the
starting values are not a physical heat-creep qualification.

```bash
orca-slicer ... --slice 0 --retraction-count-max 3 \
  --retraction-extrusion-window 1 --retraction-minimum-extrusion 0.3
# Reproduce the profile's existing behavior without the local CLI defaults:
orca-slicer ... --slice 0 --retraction-guard 0
```

## Exported files

### `--export-plates-separately`

With `--export-3mf` on a sliced run, also writes one `<stem>-plate<N>.gcode.3mf` per sliced plate next to
the project, for printers and bridges that take one plate per job. Inside each file the plate is plate
1: `Metadata/plate_1.gcode`, `plate_1.png`, one `<plate>` with index 1 in `slice_info.config`. Like the
GUI's "Export plate sliced file", the model itself is left out. The file list is in `result.json` as
`plate_files`.

```bash
orca-slicer ... --slice 0 --export-3mf kraken.3mf --export-plates-separately
# OUT/kraken.3mf, OUT/kraken-plate1.gcode.3mf, OUT/kraken-plate2.gcode.3mf
```

### Changes to every exported 3MF

- `printer_model_id` in `slice_info.config` comes from the vendor profile of the machine preset's
  `printer_model` (`Bambu Lab P1S` gives `C12`) when the CLI has no other source for it, as with a fresh
  `--datadir`.
- `filament_maps` in `slice_info.config` holds one entry per project filament, the filaments the
  `<filament id>` elements number.
- Tool selects in the machine start G-code (`T0`, `T[initial_extruder]`) are written at the start of
  their line. Only whitespace changes.

### `--ams-map LIST`

The intended AMS tray, 0-based, of each filament in filament order, e.g. `2` or `2,0`. It needs one
entry per filament. Recorded per plate in `Metadata/slice_info.config`:

```xml
<plate>
  <metadata key="index" value="1"/>
  <metadata key="printer_model_id" value="C12"/>
  <metadata key="ams_mapping" value="2,0"/>
  ...
```

## Checking the slice

### Plate summaries, `--summary FILE.json`

Every sliced plate in `result.json` carries a `summary`; `--summary` also writes them to a file of their
own, with `plate_files` and `bridge_uploads` when present.

```json
{
  "plates": [
    {
      "plate": 1, "estimated_time_s": 13987,
      "filaments": [{"id": 1, "type": "ASA", "color": "#F2754E", "length_m": 35.0, "weight_g": 85.87}],
      "objects": ["nut.stl", "ring-cap-side.stl"],
      "has_brim": false, "has_skirt": false, "has_support": false,
      "feature_types": ["Inner wall", "Outer wall", "Sparse infill"]
    }
  ]
}
```

`has_brim`, `has_skirt`, `has_support` and `feature_types` come from the G-code processor's statistics
of the exported G-code, so they describe the toolpaths it holds: support enabled on a part that needs
none gives `has_support: false`.

### `--first-layer-check`, `--first-layer-min-island MM2`

Adds `first_layer` to each plate summary: per printed instance its first-layer contact area, island
count, smallest island and distance to the excluded area. Islands under `--first-layer-min-island`
(default 2 mm²) are listed and flag `small_island`; a first layer closer than 2 mm to the excluded area
flags `near_exclusion_area`. With a raft, this is the part's first layer on the raft.

```json
"first_layer": {
  "min_island_mm2": 2.0, "min_exclusion_mm": 2.0,
  "objects": [
    {"name": "nut.stl", "island_count": 1, "contact_area_mm2": 3120.4, "smallest_island_mm2": 3120.4,
     "small_islands": [], "nearest_exclusion_mm": 61.0,
     "bbox": {"min": [13.7, 93.3], "max": [108.7, 188.3], "size": [95.0, 95.0]}, "flags": []}
  ]
}
```

## Presets

For standalone user profiles, nozzle families, JSON inspection, revision-checked
updates and cloud publication, see the [agent profile manager](agent-profile-manager.md).
The manager exports independent root presets so its family relationships do not
depend on Orca's user-parent loading order.

A user preset that `inherits` a system preset (`"from": "User"`, `"inherits": "Bambu Lab P1S 0.4 nozzle"`)
resolves in a fresh `--datadir` too: when the data directory has no vendor installed to resolve it
against, the parent is read from the bundled profiles and the preset's own values are laid over it.

### `--object-settings FILE.json`

Per-object setting overrides, applied before arranging and slicing. Keys are object names or input file
names; every object they match gets the settings. Values are JSON strings, numbers or booleans, in the
same form as in a preset. Only per-object (process) settings are accepted; an unknown key, a key that
matches no object or a setting that is not per object fails the run.

```json
{
  "gear.stl": {"brim_type": "no_brim"},
  "nut.stl": {"wall_loops": 5, "enable_support": true}
}
```

## Sending to a print bridge

### `--send-to-bridge URL`, `--bridge-token-file PATH`, `--bridge-ams-map LIST`

After exporting, POSTs each single-plate file (this implies `--export-plates-separately`) to `URL` as
`multipart/form-data` with the fields `file` (the `.gcode.3mf`) and `ams_mapping` (comma-separated
0-based trays, from `--bridge-ams-map`, else `--ams-map`), and the header
`Authorization: Bearer <token>`. The token is read from the file, surrounding whitespace removed, and is
never printed or logged; it is also scrubbed from any server reply kept in the results. Uploads use the
slicer's HTTP client, which does not verify TLS certificates, so a bridge with a self-signed certificate
is accepted.

```bash
orca-slicer ... --slice 0 --export-3mf job.3mf \
  --send-to-bridge https://bridge.local:8443/api/v1/printers/p1s/jobs \
  --bridge-token-file ~/.config/bridge/token --ams-map 2
```

Each upload is reported in `result.json` as `bridge_uploads`:
`[{"file": ".../job-plate1.gcode.3mf", "ok": true, "status": 201, "response": "..."}]`. A failed upload
ends the run with return code -49.

## Tests

`tests/cli-agent/test_agent_cli.py` runs the built binary over real parts and checks each option,
including the upload against a mock bridge on 127.0.0.1. The docstring lists the environment variables
that point it at a binary, a data directory, presets and parts.

```bash
python3 tests/cli-agent/test_agent_cli.py          # all checks
python3 tests/cli-agent/test_agent_cli.py -k bridge
```

The connection is verified: a bridge with a self-signed certificate is refused unless that
certificate (or its CA) is passed with `--bridge-ca-file PATH`. `--bridge-insecure` skips the checks;
anyone able to intercept the connection could then read the token.

## Calibration recipes

`--calibration` loads the shipped calibration model and invokes the same native settings recipe used
by the GUI Calibration menu. It accepts `flow-ratio` (classic pass 1), `flow-ratio-fine` (classic pass 2),
`flow-ratio-yolo`, and `flow-ratio-yolo-fine`. Supply printer, process and filament profiles as usual;
exactly one filament profile is required, and no input mesh is accepted. Exporting a project is required, and slicing is optional:

```bash
orca-slicer --datadir DATA \
  --load-settings 'machine.json;process.json' --load-filaments filament.json \
  --calibration flow-ratio --slice 0 --export-3mf flow.3mf --outputdir OUT
```

The flow dialog defaults to YOLO; choose `flow-ratio-yolo` to match that selection.
`--calibration-pattern archimedeanchords` (default) or `monotonic` reproduces the dialog's two surface choices.

The selected recipe overrides the same process/object fields as the GUI: Archimedean-chords top
surfaces, calibration-specific fill order, per-surface centering, shell and infill setup, widths,
no ironing or scarf seams, and common flow-limited top/solid speeds. Object names select the patch
modifiers. Classic modifiers multiply the current filament flow ratio; YOLO modifiers add to it.
The material temperatures, pressure advance and base flow ratio remain those of the loaded profile.
No cloud profiles or printer settings are changed.

Nozzle scaling uses the same ten-layer target as the GUI. Arrangement may be requested, but rotation,
other geometry transforms, and extra input models are rejected. CLI-only retraction guards are not
implicitly enabled on calibration jobs; explicit guard flags remain available.

The flat-bed tower recipes `temperature`, `max-volumetric-speed`, `retraction`, `vfa`,
`overhang-fan`, `part-fan`, `bridge-fan` and `small-feature-fan` use the same native tower
functions as their GUI menu entries. Those functions own model cutting, nozzle scaling,
recipe settings and conversion of user values to the slicer's runtime `Calib_Params`.
Both frontends select stock assets through `tower_calibration_asset`; the GUI retains its
scene updates and belt handling around these shared routines. A future change to a shared
routine therefore reaches both callers. This does not guarantee compatibility with an
arbitrary upstream merge: the runtime G-code tests and caller wiring still need review.

Tower jobs require `--slice`, `--export-3mf`, one filament profile and explicit
`--calibration-start`, `--calibration-end` and `--calibration-step`. For example:

```sh
orca-slicer --load-settings 'MACHINE.json;PROCESS.json' --load-filaments FILAMENT.json \
  --calibration retraction --calibration-start 0.2 --calibration-end 1.0 \
  --calibration-step 0.2 --slice 0 --export-3mf retraction.3mf --outputdir OUT
```

| Recipe | Range units and constraints |
|---|---|
| `temperature` | Descending temperatures in °C, multiples of 5 from 155 to 500; step 5 |
| `max-volumetric-speed` | Increasing flow in mm³/s; positive step |
| `retraction` | Increasing retraction length in mm; positive step |
| `vfa` | Increasing wall speed in mm/s; positive step |
| Four fan recipes | Increasing percentages from 0 to 100, multiples of 10; step 10 |

`--calibration-resize 1` (default) scales temperature, fan and VFA towers for the nozzle.
`--calibration-resize 0` retains the reference geometry and preset layer height, as the
underlying GUI tower helper does when resizing is disabled. VFA additionally accepts
`--calibration-layer-height MM` (zero defaults to nozzle diameter / 2). The VFA model's
blocks and runtime stepping retain 25 layers per speed block. Other towers retain the GUI's
existing height/ramp formulas, including their end-band behavior.

Runtime calibration modes belong to `Print`, and upstream project serialization does not
preserve them for reslicing a reopened project. The exported sliced 3MF contains the actual
calibration G-code; rerun the calibration command to regenerate it. Reopening the project and
slicing it as an ordinary project does not reproduce the calibration mode. Recipe setting changes use the existing preset override metadata so their values survive reopening.
No profile or project-format keys are added for the runtime requests.

Pressure advance, cornering, input shaping, belt counterrotation/support wedges and belt
sectioned temperature tests remain GUI-only generators. Unsupported recipe names fail without
exporting a substitute test. Hardware-driven automatic calibrations are separate from model generation.

`--calibration list` emits a JSON catalogue of the GUI recipes and whether this CLI can generate each
one. The shared dispatcher rejects a known but unimplemented generator explicitly. The current GUI
entrypoint map is:

| Recipes | GUI generator | Additional inputs / behavior beyond loading an asset |
|---|---|---|
| Four flow-ratio recipes | `adjust_settings_for_flowrate_calib` | Shared native recipe, nozzle scaling, labelled percentage/additive patch factors |
| PA line, pattern, tower | `Plater::calib_pa` | Start/end/step, extruder, numbered lines, model variant, PA-specific G-code generation |
| Temperature | `Plater::calib_temp` | Shared flat-bed tower blocks and runtime parameters; belt uses sectioned objects/events |
| Maximum volumetric speed | `Plater::calib_max_vol_speed` | Shared thick-line/spiral setup, bed-aware scaling, tower cut and volume-to-speed conversion |
| Retraction | `Plater::calib_retraction` | Shared nozzle-dependent layers, firmware-retraction disable and tower cut |
| VFA | `Plater::calib_VFA` | Shared layer-height resolution, nozzle scaling, layer-count blocks and slowdown/spiral setup |
| Cornering | `Plater::Calib_Cornering` | Range, model selection, acceleration/speed vectors, firmware-specific runtime commands |
| Input-shaping frequency/damping | `Plater::calib_input_shaping_freq` / `calib_input_shaping_damp` | Per-axis ranges, shaper, model selection, firmware-specific runtime commands |
| Four fan towers | `Plater::calib_fan_tower` | Shared tower cutting/nozzle resize, overhang classification overrides and runtime cooling parameters |

The menu dispatch lives in `MainFrame::run_calibration`; recipe geometry/configuration lives in
`Plater.cpp`, `FlowRatioCalibration.cpp` and `TowerCalibration.cpp`. Tower `Calib_Params` also drive the G-code writer and cooling buffer. Loading the model
or storing only visible settings does not reproduce those runtime modes.
