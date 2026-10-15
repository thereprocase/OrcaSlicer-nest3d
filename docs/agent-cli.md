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

### `--arrange-gap MM`

The minimum distance between two arranged parts and between a part and the bed's excluded area, for
both arrangers. Without it the spacing comes from the parts' brim and support settings (1 mm per part
without support). Parts keep 2 mm from the bed edge, as with the stock spacing, or half the gap when
that is more.

```bash
orca-slicer ... --arrange 1 --arrange-engine nest3d --arrange-gap 3 nut.stl nut.stl ring.stl
```

### `--arrange-report FILE.json`

Written after arranging (and with `--arrange 0`, for the parts as loaded). Distances are measured on the
parts' footprints, the outlines of their meshes seen from above, in plate coordinates (mm).

```json
{
  "engine": "nest3d", "stacking": false, "gap_mm": 3.0, "arranged": true,
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
ends the run with return code -25.

## Tests

`tests/cli-agent/test_agent_cli.py` runs the built binary over real parts and checks each option,
including the upload against a mock bridge on 127.0.0.1. The docstring lists the environment variables
that point it at a binary, a data directory, presets and parts.

```bash
python3 tests/cli-agent/test_agent_cli.py          # all checks
python3 tests/cli-agent/test_agent_cli.py -k bridge
```
