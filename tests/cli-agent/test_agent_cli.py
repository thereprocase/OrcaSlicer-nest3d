#!/usr/bin/env python3
"""End-to-end checks of the command-line options for agent and print-bridge use (docs/agent-cli.md).

Runs the built orca-slicer on real parts and checks what it writes: the arrange report, single-plate
.gcode.3mf files, slice_info metadata, plate summaries against the G-code, the first-layer check,
inheriting presets in an empty data directory, per-object settings and an upload to a local mock
bridge. Standard library only.

Inputs, overridable through the environment:
  ORCA_BIN        the orca-slicer binary        (build/src/Release/orca-slicer)
  ORCA_ENV        wrapper that sets up its libs (build/package/libexec/orca-slicer-env, optional)
  ORCA_DATADIR    a data directory with system profiles, copied per run
  ORCA_PRESETS    directory with flattened machine.json, process.json and filament.json for a P1S
  ORCA_PARTS      directory with nut.stl, ring-cap-side.stl and ring-gear-side-1.5mm-flange.stl

Usage: python3 tests/cli-agent/test_agent_cli.py [-k NAME] [--keep]
"""

import argparse
import http.server
import json
import os
import re
import secrets
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LAB = "/home/repro/lab/kraken-ams/print-2026-10-14"
BIN = os.environ.get("ORCA_BIN", os.path.join(REPO, "build/src/Release/orca-slicer"))
ENV = os.environ.get("ORCA_ENV", os.path.join(REPO, "build/package/libexec/orca-slicer-env"))
DATADIR = os.environ.get("ORCA_DATADIR", os.path.join(LAB, "nest3d-test/data-x"))
PRESETS = os.environ.get("ORCA_PRESETS", os.path.join(LAB, "nest3d-test"))
PARTS = os.environ.get("ORCA_PARTS", os.path.join(LAB, "asa"))

NUT = os.path.join(PARTS, "nut.stl")
CAP = os.path.join(PARTS, "ring-cap-side.stl")
GEAR = os.path.join(PARTS, "ring-gear-side-1.5mm-flange.stl")
KRAKEN_SET = [NUT] * 2 + [CAP] * 4 + [GEAR] * 4

# Distances in the report are measured on outlines simplified to 0.01 mm and rounded to 0.001 mm.
DISTANCE_TOLERANCE = 0.03


class Work:
    """A scratch directory per test with a private copy of the data directory."""

    def __init__(self, root, name):
        self.dir = os.path.join(root, name)
        os.makedirs(self.dir)
        self.out = os.path.join(self.dir, "out")
        os.makedirs(self.out)
        self._datadir = None

    @property
    def datadir(self):
        if self._datadir is None:
            self._datadir = os.path.join(self.dir, "data")
            shutil.copytree(DATADIR, self._datadir, symlinks=True)
        return self._datadir

    def path(self, name):
        return os.path.join(self.dir, name)

    def write_json(self, name, data):
        with open(self.path(name), "w") as f:
            json.dump(data, f)
        return self.path(name)


def run_slicer(work, inputs, *options, datadir=None, presets=None, filaments=None, check=True, env=None):
    machine, process = presets or (os.path.join(PRESETS, "machine.json"), os.path.join(PRESETS, "process.json"))
    filaments = filaments or [os.path.join(PRESETS, "filament.json")]
    cmd = ([ENV] if os.path.exists(ENV) else []) + [
        BIN,
        "--datadir", datadir or work.datadir,
        "--load-settings", f"{machine};{process}",
        "--load-filaments", ";".join(filaments),
        "--outputdir", work.out,
        *options,
        *inputs,
    ]
    run_env = dict(os.environ)
    run_env.pop("ORCA_NEST3D", None)
    run_env.update(env or {})
    started = time.time()
    proc = subprocess.run(cmd, capture_output=True, text=True, env=run_env)
    with open(work.path("stdout.txt"), "w") as f:
        f.write(proc.stdout)
    with open(work.path("stderr.txt"), "w") as f:
        f.write(proc.stderr)
    result_file = os.path.join(work.out, "result.json")
    result = json.load(open(result_file)) if os.path.exists(result_file) else {}
    if check and proc.returncode != 0:
        raise AssertionError(f"orca-slicer exited {proc.returncode} after {time.time() - started:.0f} s: "
                             f"{result.get('error_string')}\n{proc.stderr[-2000:]}")
    return proc, result


def read_zip(path):
    with zipfile.ZipFile(path) as z:
        return {name: z.read(name) for name in z.namelist()}


def slice_info_plates(xml):
    """Per <plate> of slice_info.config: its metadata and its <filament> and <object> elements."""
    plates = []
    for block in re.findall(r"<plate>(.*?)</plate>", xml, re.S):
        plates.append({
            "metadata": dict(re.findall(r'<metadata key="([^"]+)" value="([^"]*)"', block)),
            "filaments": re.findall(r"<filament ([^>]*)/>", block),
            "objects": re.findall(r'<object [^>]*name="([^"]*)"', block),
        })
    return plates


def gcode_features(gcode):
    return set(re.findall(r"^; FEATURE: (.+?)\s*$", gcode, re.M))


def check(condition, message):
    if not condition:
        raise AssertionError(message)


# ---------------------------------------------------------------------------------------------------------
# Arrange


_arranged = {}


def arrange_kraken_set(root, engine):
    """The arrange report of the ten-part set at a 3 mm gap, run once per engine."""
    if (root, engine) in _arranged:
        return _arranged[(root, engine)]
    work = Work(root, f"arrange-{engine}")
    run_slicer(work, KRAKEN_SET, "--arrange", "1", "--orient", "0", "--arrange-engine", engine, "--arrange-gap", "3",
               "--arrange-report", "report.json", "--slice", "0", "--export-3mf", "plate.3mf")
    _arranged[(root, engine)] = json.load(open(os.path.join(work.out, "report.json")))
    return _arranged[(root, engine)]


def check_gap(report, gap):
    for obj in report["objects"]:
        name = f'{obj["name"]} (object {obj["object_index"]})'
        if obj["nearest_neighbour_mm"] is not None:
            check(obj["nearest_neighbour_mm"] >= gap - DISTANCE_TOLERANCE,
                  f'{name} is {obj["nearest_neighbour_mm"]} mm from {obj["nearest_neighbour"]}, gap {gap}')
        if obj["nearest_exclusion_mm"] is not None:
            check(obj["nearest_exclusion_mm"] >= gap - DISTANCE_TOLERANCE,
                  f'{name} is {obj["nearest_exclusion_mm"]} mm from the excluded area, gap {gap}')
        # The stock 2 mm bed margin, or half the gap when that is more.
        check(obj["bed_edge_mm"] >= max(2., gap / 2) - DISTANCE_TOLERANCE, f'{name} is {obj["bed_edge_mm"]} mm from the bed edge')


def test_arrange_report_and_gap(root, notes):
    for engine in ("stock", "nest3d"):
        report = arrange_kraken_set(root, engine)
        check(report["engine"] == engine and report["gap_mm"] == 3, f"report header {report['engine']} {report['gap_mm']}")
        check(len(report["objects"]) + len(report["unplaced"]) == len(KRAKEN_SET), "every object is reported")
        for obj in report["objects"]:
            for key in ("name", "source_file", "plate", "x", "y", "rotation_deg", "footprint_bbox",
                        "nearest_neighbour_mm", "nearest_exclusion_mm"):
                check(key in obj, f"report entry lacks {key}")
            check(os.path.basename(obj["source_file"]) == obj["name"], f'source file {obj["source_file"]}')
            bbox = obj["footprint_bbox"]
            check(0 <= bbox["min"][0] <= bbox["max"][0] <= 256 and 0 <= bbox["min"][1] <= bbox["max"][1] <= 256,
                  f"footprint {bbox} lies on the 256 mm bed")
        check(report["plates_used"] == max(o["plate"] for o in report["objects"]), "plates_used")
        check_gap(report, 3.)
        notes.append(f"{engine}: {report['plates_used']} plate(s), {len(report['unplaced'])} unplaced, "
                     f"closest pair {min(o['nearest_neighbour_mm'] or 1e9 for o in report['objects']):.3f} mm, "
                     f"closest to the excluded area "
                     f"{min(o['nearest_exclusion_mm'] or 1e9 for o in report['objects']):.3f} mm")


def test_kraken_set_fits_one_plate_with_3mm_gap(root, notes):
    plates = {engine: arrange_kraken_set(root, f"{engine}") for engine in ("stock", "nest3d")}
    used = {engine: report["plates_used"] + (1 if report["unplaced"] else 0) for engine, report in plates.items()}
    notes.append(f"plates used with a 3 mm gap: {used}")
    check(min(used.values()) == 1, f"neither arranger fits the set on one plate: {used}")


def test_nest3d_selection_and_env_fallback(root, notes):
    work = Work(root, "engine-select")
    proc, _ = run_slicer(work, [CAP], "--arrange", "1", "--arrange-engine", "bogus", "--slice", "0", check=False)
    check(proc.returncode != 0, "an unknown engine is rejected")
    work = Work(root, "engine-env")
    run_slicer(work, [CAP, CAP], "--arrange", "1", "--arrange-report", "r.json", "--slice", "0",
               env={"ORCA_NEST3D": "stack"})
    report = json.load(open(os.path.join(work.out, "r.json")))
    check(report["engine"] == "nest3d" and report["stacking"], f"ORCA_NEST3D=stack: {report['engine']} {report['stacking']}")
    work = Work(root, "engine-stack")
    run_slicer(work, [CAP, CAP], "--arrange", "1", "--nest3d-stack", "--arrange-report", "r.json", "--slice", "0")
    report = json.load(open(os.path.join(work.out, "r.json")))
    check(report["engine"] == "nest3d" and report["stacking"], "--nest3d-stack implies Nest3D")


# ---------------------------------------------------------------------------------------------------------
# Exported files


def test_plate_files_and_slice_info(root, notes):
    work = Work(root, "plates")
    _, result = run_slicer(work, KRAKEN_SET, "--arrange", "1", "--orient", "0", "--slice", "0",
                           "--export-3mf", "kraken.3mf", "--export-plates-separately", "--ams-map", "2")
    project = read_zip(os.path.join(work.out, "kraken.3mf"))
    project_plates = slice_info_plates(project["Metadata/slice_info.config"].decode())
    check(len(project_plates) >= 2, f"the 10-part set at the default spacing needs 2 plates here, got {len(project_plates)}")
    check(len(result["plate_files"]) == len(project_plates), f"one file per plate: {result['plate_files']}")

    for n, plate_file in enumerate(result["plate_files"], start=1):
        check(os.path.basename(plate_file) == f"kraken-plate{n}.gcode.3mf", f"file name {plate_file}")
        files = read_zip(plate_file)
        gcodes = sorted(name for name in files if re.fullmatch(r"Metadata/plate_\d+\.gcode", name))
        check(gcodes == ["Metadata/plate_1.gcode"], f"{plate_file}: {gcodes}")
        plates = slice_info_plates(files["Metadata/slice_info.config"].decode())
        check(len(plates) == 1, f"{plate_file}: {len(plates)} plates in slice_info")
        meta = plates[0]["metadata"]
        check(meta["index"] == "1", f"{plate_file}: plate index {meta['index']}")
        check(meta["printer_model_id"] == "C12", f"printer_model_id {meta['printer_model_id']!r}")
        check(meta.get("ams_mapping") == "2", f"ams_mapping {meta.get('ams_mapping')!r}")
        check(len(meta["filament_maps"].split()) == 1, f"filament_maps {meta['filament_maps']!r} for one filament")
        check(sorted(plates[0]["objects"]) == sorted(project_plates[n - 1]["objects"]),
              f"{plate_file} holds plate {n}'s objects")
        check("Metadata/plate_1.png" in files, f"{plate_file} has its thumbnail as plate_1.png")
        gcode = files["Metadata/plate_1.gcode"].decode()
        check(gcode == open(os.path.join(work.out, f"plate_{n}.gcode")).read(), "the G-code of that plate")
        start = gcode[:gcode.index("; MACHINE_START_GCODE_END")] if "; MACHINE_START_GCODE_END" in gcode else gcode
        indented = re.findall(r"^[ \t]+T\d+\b.*$", start, re.M)
        check(not indented, f"indented tool selects: {indented[:3]}")
        check(re.search(r"^T\d+\s*$", start, re.M), "the start G-code selects a tool")


def test_filament_maps_one_slot_per_filament(root, notes):
    work = Work(root, "filament-maps")
    filament = os.path.join(PRESETS, "filament.json")
    second = json.load(open(filament))
    second.update(name="Kraken ASA white", filament_colour=["#FFFFFF"])
    second = work.write_json("filament-white.json", second)
    # Two distinct filaments, and one preset loaded twice, which leaves a single project filament.
    for name, filaments in (("distinct", [filament, second]), ("same", [filament, filament])):
        run_slicer(work, [CAP, CAP], "--arrange", "1", "--load-filament-ids", "1,2", "--slice", "0",
                   "--export-3mf", f"{name}.3mf", filaments=filaments)
        plates = slice_info_plates(read_zip(os.path.join(work.out, f"{name}.3mf"))["Metadata/slice_info.config"].decode())
        for plate in plates:
            ids = [int(re.search(r'id="(\d+)"', f).group(1)) for f in plate["filaments"]]
            slots = len(plate["metadata"]["filament_maps"].split())
            check(slots >= max(ids) and slots >= 2 if name == "distinct" else slots >= max(ids),
                  f"{name}: filament_maps {plate['metadata']['filament_maps']!r} for filaments {ids}")

    proc, _ = run_slicer(work, [CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "q.3mf", "--ams-map", "1,2,3",
                         filaments=[filament, filament], check=False)
    check(proc.returncode != 0, "an AMS map with a tray count other than the filament count is rejected")


# ---------------------------------------------------------------------------------------------------------
# Summary and first-layer check


def l_bracket_stl(path):
    """An L-shaped bracket 30 x 10 x 20 mm whose 20 mm arm overhangs at 17 mm: it needs support."""
    # Profile in XZ, split into rectangles that share their corners; extruded 10 mm along Y.
    rects = [((0, 0), (10, 17)), ((0, 17), (10, 20)), ((10, 17), (30, 20))]
    outline = [(0, 0), (10, 0), (10, 17), (30, 17), (30, 20), (10, 20), (0, 20), (0, 17)]  # counter-clockwise in XZ
    tris = []
    for (x0, z0), (x1, z1) in rects:
        for y in (0, 10):
            a, b, c, d = (x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)
            # (a, b, c) faces -Y: right for the y=0 cap, reversed for the y=10 one.
            quad = [(a, b, c), (a, c, d)]
            tris += quad if y == 0 else [(t[0], t[2], t[1]) for t in quad]
    for (xa, za), (xb, zb) in zip(outline, outline[1:] + outline[:1]):
        a, b, c, d = (xa, 0, za), (xb, 0, zb), (xb, 10, zb), (xa, 10, za)
        tris += [(a, d, c), (a, c, b)]
    with open(path, "wb") as f:
        f.write(b"l-bracket".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for tri in tris:
            f.write(struct.pack("<3f", 0, 0, 0))
            for v in tri:
                f.write(struct.pack("<3f", *v))
            f.write(b"\0\0")
    return path


def box_stl(path, lo, hi):
    """An axis-aligned box from corner `lo` to corner `hi`, outward-facing."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    quads = [((x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)), ((x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)),
             ((x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)), ((x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)),
             ((x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)), ((x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1))]
    with open(path, "wb") as f:
        f.write(b"box".ljust(80, b" "))
        f.write(struct.pack("<I", 2 * len(quads)))
        for a, b, c, d in quads:
            for tri in ((a, b, c), (a, c, d)):
                f.write(struct.pack("<3f", 0, 0, 0))
                for v in tri:
                    f.write(struct.pack("<3f", *v))
                f.write(b"\0\0")
    return path


def test_summary_matches_toolpaths(root, notes):
    work = Work(root, "summary-plain")
    _, result = run_slicer(work, [CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "p.3mf", "--summary", "summary.json")
    summary = json.load(open(os.path.join(work.out, "summary.json")))
    check(summary["plates"] == [p["summary"] for p in result["sliced_plates"]], "result.json carries the same summary")
    plate = summary["plates"][0]
    features = gcode_features(open(os.path.join(work.out, "plate_1.gcode")).read())
    check(plate["has_brim"] == ("Brim" in features) and not plate["has_brim"], f"no brim: {plate['has_brim']} {features}")
    check(plate["has_skirt"] == ("Skirt" in features), f"skirt {plate['has_skirt']} vs {features}")
    check(plate["has_support"] == any(f.startswith("Support") for f in features), f"support vs {features}")
    check(plate["objects"] == ["ring-cap-side.stl"], f"objects {plate['objects']}")
    check(plate["estimated_time_s"] > 0 and plate["filaments"][0]["weight_g"] > 0 and plate["filaments"][0]["length_m"] > 0,
          f"time and filament {plate}")

    # Brim on one object through --object-settings, skirt through the process, support on an overhang.
    process = json.load(open(os.path.join(PRESETS, "process.json")))
    process["skirt_loops"] = "1"
    process_file = work.write_json("process-skirt.json", process)
    settings = work.write_json("objects.json", {"ring-cap-side.stl": {"brim_type": "outer_only", "brim_width": 5},
                                                "bracket.stl": {"enable_support": True}})
    bracket = l_bracket_stl(work.path("bracket.stl"))
    work = Work(root, "summary-features")
    _, result = run_slicer(work, [CAP, bracket], "--arrange", "1", "--slice", "0", "--export-3mf", "p.3mf",
                           "--object-settings", settings, "--summary", "summary.json",
                           presets=(os.path.join(PRESETS, "machine.json"), process_file))
    plate = json.load(open(os.path.join(work.out, "summary.json")))["plates"][0]
    features = gcode_features(open(os.path.join(work.out, "plate_1.gcode")).read())
    check(plate["has_brim"] and "Brim" in features, f"brim: {plate['has_brim']} {features}")
    check(plate["has_skirt"] and "Skirt" in features, f"skirt: {plate['has_skirt']} {features}")
    check(plate["has_support"] and any(f.startswith("Support") for f in features), f"support: {plate['has_support']} {features}")
    notes.append(f"feature types with brim/skirt/support: {plate['feature_types']}")


def test_first_layer_check(root, notes):
    work = Work(root, "first-layer")
    _, result = run_slicer(work, [NUT, CAP], "--arrange", "1", "--arrange-gap", "3", "--slice", "0", "--export-3mf", "p.3mf",
                           "--first-layer-check", "--arrange-report", "report.json")
    report = {o["name"]: o for o in json.load(open(os.path.join(work.out, "report.json")))["objects"]}
    first = result["sliced_plates"][0]["summary"]["first_layer"]
    check(first["min_island_mm2"] == 2 and len(first["objects"]) == 2, f"first layer {first}")
    for obj in first["objects"]:
        check(0 < obj["smallest_island_mm2"] <= obj["contact_area_mm2"], f"areas {obj}")
        check(obj["island_count"] >= 1 and "small_island" not in obj["flags"], f"islands {obj}")
        footprint = report[obj["name"]]["footprint_bbox"]
        check(all(abs(obj["bbox"][k][i] - footprint[k][i]) < 1. for k in ("min", "max") for i in (0, 1)),
              f"first layer of {obj['name']} {obj['bbox']} lies where the report puts it {footprint}")
        check(obj["nearest_exclusion_mm"] >= report[obj["name"]]["nearest_exclusion_mm"] - DISTANCE_TOLERANCE,
              "the first layer is no closer to the excluded area than the whole part")
        check("near_exclusion_area" not in obj["flags"], "3 mm from the excluded area is not flagged")

    # Left where the file puts it, 1.5 mm right of the 18 x 28 mm excluded area.
    work = Work(root, "first-layer-near-exclusion")
    cube = box_stl(work.path("cube.stl"), (19.5, 5, 0), (29.5, 15, 10))
    _, result = run_slicer(work, [cube], "--arrange", "0", "--slice", "0", "--export-3mf", "p.3mf",
                           "--first-layer-check", "--arrange-report", "report.json")
    report = json.load(open(os.path.join(work.out, "report.json")))
    check(not report["unplaced"] and report["objects"][0]["plate"] == 1, f"an unarranged part on the bed is on plate 1: {report}")
    check(abs(report["objects"][0]["nearest_exclusion_mm"] - 1.5) < DISTANCE_TOLERANCE, f"report {report['objects'][0]}")
    obj = result["sliced_plates"][0]["summary"]["first_layer"]["objects"][0]
    check(abs(obj["nearest_exclusion_mm"] - 1.5) < DISTANCE_TOLERANCE and "near_exclusion_area" in obj["flags"],
          f"1.5 mm from the excluded area is flagged: {obj}")
    check(abs(obj["contact_area_mm2"] - 100) < 1, f"a 10 x 10 mm box stands on 100 mm2: {obj['contact_area_mm2']}")

    work = Work(root, "first-layer-threshold")
    _, result = run_slicer(work, [CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "p.3mf",
                           "--first-layer-check", "--first-layer-min-island", "100000")
    obj = result["sliced_plates"][0]["summary"]["first_layer"]["objects"][0]
    check("small_island" in obj["flags"] and obj["small_islands"], f"islands under the threshold are flagged: {obj}")


# ---------------------------------------------------------------------------------------------------------
# Presets and per-object settings


def test_inheriting_presets_in_empty_datadir(root, notes):
    work = Work(root, "inherits")
    machine = work.write_json("machine.json", {"type": "machine", "name": "Agent P1S", "from": "User",
                                               "inherits": "Bambu Lab P1S 0.4 nozzle"})
    process = work.write_json("process.json", {"type": "process", "name": "Agent 0.20", "from": "User",
                                               "inherits": "0.20mm Standard @BBL X1C", "brim_type": "no_brim",
                                               "curr_bed_type": "Textured PEI Plate",
                                               "wall_loops": "4"})
    filament = work.write_json("filament.json", {"type": "filament", "name": "Agent ASA", "from": "User",
                                                 "inherits": "Bambu ASA @BBL X1C"})
    empty = work.path("empty-datadir")
    os.makedirs(empty)
    _, result = run_slicer(work, [CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "p.3mf",
                           datadir=empty, presets=(machine, process), filaments=[filament])
    files = read_zip(os.path.join(work.out, "p.3mf"))
    settings = json.loads(files["Metadata/project_settings.config"])
    check(settings["wall_loops"] == "4" and settings["brim_type"] == "no_brim", "the user's values win")
    check(settings["printer_model"] == "Bambu Lab P1S", f"printer_model from the parent: {settings['printer_model']}")
    check(settings["filament_type"] == ["ASA"], f"filament from its parent: {settings['filament_type']}")
    check(result["wall_loops"] == 4, "sliced with the user's wall loops")
    meta = slice_info_plates(files["Metadata/slice_info.config"].decode())[0]["metadata"]
    check(meta["printer_model_id"] == "C12", f"printer_model_id {meta['printer_model_id']!r}")


def test_object_settings(root, notes):
    work = Work(root, "object-settings")
    settings = work.write_json("objects.json", {"nut.stl": {"wall_loops": 5}, "ring-cap-side.stl": {"brim_type": "outer_only"}})
    run_slicer(work, [NUT, CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "p.3mf", "--object-settings", settings)
    model_settings = read_zip(os.path.join(work.out, "p.3mf"))["Metadata/model_settings.config"].decode()
    objects = re.findall(r"<object id=\"\d+\">(.*?)</object>", model_settings, re.S)
    by_name = {re.search(r'key="name" value="([^"]*)"', o).group(1): o for o in objects}
    check('key="wall_loops" value="5"' in by_name["nut.stl"], "nut.stl got wall_loops 5")
    check('key="brim_type" value="outer_only"' in by_name["ring-cap-side.stl"], "ring-cap-side.stl got its brim")
    check('key="brim_type"' not in by_name["nut.stl"], "the nut keeps the process brim")

    for bad in ({"nosuch.stl": {"wall_loops": 2}}, {"nut.stl": {"no_such_setting": 1}}, {"nut.stl": {"printer_model": "x"}}):
        work = Work(root, f"object-settings-bad-{len(os.listdir(root))}")
        proc, _ = run_slicer(work, [NUT], "--arrange", "1", "--slice", "0", "--object-settings", work.write_json("o.json", bad),
                             check=False)
        check(proc.returncode != 0, f"{bad} is rejected")


# ---------------------------------------------------------------------------------------------------------
# Bridge upload


class MockBridge(http.server.BaseHTTPRequestHandler):
    requests = []

    def do_POST(self):
        body = self.rfile.read(int(self.headers["Content-Length"]))
        MockBridge.requests.append({"path": self.path, "headers": dict(self.headers), "body": body})
        reply = json.dumps({"job": len(MockBridge.requests)}).encode()
        self.send_response(201)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(reply)))
        self.end_headers()
        self.wfile.write(reply)

    def log_message(self, *args):
        pass


def multipart_fields(request):
    boundary = re.search(r"boundary=(.+)", request["headers"]["Content-Type"]).group(1).strip('"').encode()
    fields = {}
    for part in request["body"].split(b"--" + boundary)[1:-1]:
        head, _, content = part.partition(b"\r\n\r\n")
        name = re.search(rb'name="([^"]+)"', head).group(1).decode()
        filename = re.search(rb'filename="([^"]+)"', head)
        fields[name] = {"filename": filename.group(1).decode() if filename else None, "content": content[:-2]}
    return fields


def test_send_to_bridge_mock(root, notes):
    MockBridge.requests = []
    server = http.server.HTTPServer(("127.0.0.1", 0), MockBridge)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        work = Work(root, "bridge")
        token = "tok-" + secrets.token_hex(16)
        token_file = work.path("token")
        with open(token_file, "w") as f:
            f.write(token + "\n")
        url = f"http://127.0.0.1:{server.server_port}/api/v1/printers/p1s/jobs"
        logfile = work.path("orca.log")
        proc, result = run_slicer(work, [NUT, NUT, CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "job.3mf",
                                  "--send-to-bridge", url, "--bridge-token-file", token_file, "--bridge-ams-map", "3",
                                  "--debug", "5", "--logfile", logfile)
    finally:
        server.shutdown()

    plate_files = result["plate_files"]
    check(len(MockBridge.requests) == len(plate_files) >= 1, f"{len(MockBridge.requests)} uploads for {plate_files}")
    for request, plate_file in zip(MockBridge.requests, plate_files):
        check(request["path"] == "/api/v1/printers/p1s/jobs", f"path {request['path']}")
        check(request["headers"].get("Authorization") == f"Bearer {token}", "bearer token header")
        fields = multipart_fields(request)
        check(fields["ams_mapping"]["content"] == b"3", f"ams_mapping {fields['ams_mapping']}")
        check(fields["file"]["filename"] == os.path.basename(plate_file), f"file name {fields['file']['filename']}")
        check(fields["file"]["content"] == open(plate_file, "rb").read(), "the uploaded bytes are the plate file")
    check(all(u["ok"] and u["status"] == 201 for u in result["bridge_uploads"]), f"uploads {result['bridge_uploads']}")

    leaks = [name for name, text in (("stdout", proc.stdout), ("stderr", proc.stderr)) if token in text]
    for directory in (work.dir,):
        for dirpath, _, names in os.walk(directory):
            for name in names:
                path = os.path.join(dirpath, name)
                if path == token_file:
                    continue
                with open(path, "rb") as f:
                    if token.encode() in f.read():
                        leaks.append(path)
    check(os.path.getsize(logfile) > 0, "the debug log was written")
    check(not leaks, f"the token leaked into {leaks}")

    # A rejected upload fails the run and is reported.
    work = Work(root, "bridge-down")
    proc, result = run_slicer(work, [CAP], "--arrange", "1", "--slice", "0", "--export-3mf", "job.3mf",
                              "--send-to-bridge", "http://127.0.0.1:9/jobs", "--bridge-token-file", token_file,
                              "--ams-map", "0", check=False)
    check(proc.returncode != 0 and not result["bridge_uploads"][0]["ok"], "an unreachable bridge fails the run")
    check(token not in proc.stdout + proc.stderr + json.dumps(result), "no token on failure either")


def test_send_to_bridge_tls(root, notes):
    """The token only goes out over a verified connection: a self-signed bridge is refused unless its
    certificate is given with --bridge-ca-file (or checks are waived with --bridge-insecure)."""
    import ssl
    import subprocess as sp

    work = Work(root, "bridge-tls")
    cert, key = work.path("bridge.pem"), work.path("bridge.key")
    sp.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1", "-subj", "/CN=127.0.0.1",
            "-addext", "subjectAltName=IP:127.0.0.1", "-keyout", key, "-out", cert], check=True, capture_output=True)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    token_file = work.path("token")
    with open(token_file, "w") as f:
        f.write("tok-" + secrets.token_hex(16) + "\n")

    def attempt(name, extra):
        MockBridge.requests = []
        server = http.server.HTTPServer(("127.0.0.1", 0), MockBridge)
        server.socket = ctx.wrap_socket(server.socket, server_side=True)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            url = f"https://127.0.0.1:{server.server_port}/api/v1/printers/p1s/jobs"
            proc, result = run_slicer(Work(root, name), [CAP], "--arrange", "1", "--slice", "0", "--export-3mf",
                                      "job.3mf", "--send-to-bridge", url, "--bridge-token-file", token_file,
                                      "--ams-map", "0", *extra, check=False)
        finally:
            server.shutdown()
        return proc, result, len(MockBridge.requests)

    proc, result, seen = attempt("tls-untrusted", [])
    check(proc.returncode != 0 and seen == 0, f"a self-signed bridge is refused by default (exit {proc.returncode}, {seen} requests)")
    proc, result, seen = attempt("tls-pinned", ["--bridge-ca-file", cert])
    check(proc.returncode == 0 and seen == 1, f"--bridge-ca-file trusts the bridge (exit {proc.returncode}, {seen} requests)")
    proc, result, seen = attempt("tls-insecure", ["--bridge-insecure"])
    check(proc.returncode == 0 and seen == 1, f"--bridge-insecure skips the checks (exit {proc.returncode}, {seen} requests)")


TESTS = [
    test_arrange_report_and_gap,
    test_kraken_set_fits_one_plate_with_3mm_gap,
    test_nest3d_selection_and_env_fallback,
    test_plate_files_and_slice_info,
    test_filament_maps_one_slot_per_filament,
    test_summary_matches_toolpaths,
    test_first_layer_check,
    test_inheriting_presets_in_empty_datadir,
    test_object_settings,
    test_send_to_bridge_mock,
    test_send_to_bridge_tls,
]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-k", help="run only tests whose name contains this")
    parser.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = parser.parse_args()

    for path in (BIN, DATADIR, NUT, CAP, GEAR):
        if not os.path.exists(path):
            sys.exit(f"missing {path}; see the docstring for the environment variables")

    root = tempfile.mkdtemp(prefix="orca-agent-cli-")
    failed = 0
    for test in TESTS:
        if args.k and args.k not in test.__name__:
            continue
        notes = []
        started = time.time()
        try:
            test(root, notes)
            status = "PASS"
        except Exception as ex:
            failed += 1
            status = "FAIL"
            notes.append("".join(traceback.format_exception_only(type(ex), ex)).strip())
        print(f"{status} {test.__name__} ({time.time() - started:.0f} s)")
        for note in notes:
            print(f"     {note}")
        sys.stdout.flush()
    if args.keep:
        print(f"scratch kept in {root}")
    else:
        shutil.rmtree(root, ignore_errors=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
