#!/usr/bin/env python3
"""Check support interface filament changes and nozzle clearance in a sliced G-code file.

Reads an OrcaSlicer G-code file (plain text, absolute XYZ) and reports, for every run of support
interface layers (interface printed on consecutive print_z levels):

  * the batches it printed as ("; BATCHED_INTERFACE_START" blocks, support_interface_batch) and
    the planner's "; batched interface:" comments explaining a split or a fallback;
  * the tool changes from the run's first layer to the layer after its last, and how many of
    them load the interface filament;
  * for every extrusion beside material standing taller than the nozzle tip, the distance from
    the nozzle centre to that material's edge minus the distance the nozzle needs (d - d_req);
  * for every travel, the clearance of the nozzle (tip and cone) over printed material.

Nozzle model, the same as OrcaSlicer's GCode/NozzleClearance.hpp: a flat tip of radius
r0 = 1.25 x nozzle diameter under a 45 degree cone. Beside material `delta` taller than the tip
the nozzle centre needs d_req = r0 + delta + 0.15 mm to the material's edge. Material is traced
from the extrusions with their ;WIDTH: / ; LINE_WIDTH: tags.

Nozzle diameter, nozzle_height and the batch lead come from the file's config block unless given.
Standard library only. Usage:
    check_support_interface_batch.py plate_1.gcode [--nozzle-diameter 0.4] [--lead 0.4]
Exit status 1 if an extrusion is closer than d_req to taller material (beyond --tolerance), a
travel passes through material, or a step beside an extrusion exceeds the lead; else 0.
"""

import argparse
import math
import re
import sys

CELL = 0.2  # mm, material map resolution
STEP = 0.1  # mm, sampling step along moves
EPS = 1e-3
MARGIN = 0.15  # mm, NozzleClearance::margin

MOVE_RE = re.compile(r"^G[0-3](?:\s|$)")
WORD_RE = re.compile(r"([XYZEF])(-?\d*\.?\d+)")
TOOL_RE = re.compile(r"^T(\d+)\s*(?:;.*)?$")
CONFIG_RE = re.compile(r"^;\s*([a-z_0-9]+)\s*=\s*(.*)$")


def samples(a, b, step=STEP):
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / step)))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


class MaterialMap:
    """The tallest extrusion sample per CELL x CELL cell: (top z, line width, x, y)."""

    def __init__(self):
        self.cells = {}

    def deposit(self, a, b, z, width):
        for x, y in samples(a, b):
            key = (math.floor(x / CELL), math.floor(y / CELL))
            old = self.cells.get(key)
            if old is None or old[0] < z:
                self.cells[key] = (z, width, x, y)

    def near(self, x, y, radius):
        """Yield (top z, distance from (x, y) to the material's edge) within radius."""
        r = int(math.ceil(radius / CELL)) + 1
        cx, cy = math.floor(x / CELL), math.floor(y / CELL)
        for ix in range(cx - r, cx + r + 1):
            for iy in range(cy - r, cy + r + 1):
                cell = self.cells.get((ix, iy))
                if cell is None:
                    continue
                d = math.hypot(cell[2] - x, cell[3] - y) - 0.5 * cell[1]
                if d <= radius:
                    yield cell[0], d


class Run:
    def __init__(self, levels):
        self.levels = levels  # interface print_z values, ascending
        self.first_layer = None
        self.last_layer = None
        self.interface_tool = None
        self.batches = set()
        self.notes = []
        self.tool_changes = 0
        self.interface_loads = 0
        self.worst_margin = None  # (d - d_req, line, delta)
        self.margin_violations = []
        self.deepest_step = None  # (delta, line)
        self.min_travel = None  # (clearance, line)
        self.travels_through = []
        self.extrusions_checked = 0
        self.travels_checked = 0


def read_config(path):
    config = {}
    with open(path, "r", errors="replace") as f:
        for line in f:
            m = CONFIG_RE.match(line.strip())
            if m:
                config.setdefault(m.group(1), m.group(2))
    return config


def first_float(value, default):
    try:
        return float(str(value).split(",")[0].strip().strip('"'))
    except (TypeError, ValueError):
        return default


def parse(path):
    pos = [0.0, 0.0, 0.0]
    relative_e = False
    e_abs = 0.0
    tool = None
    layer = -1
    layer_z = {}
    feature = ""
    width = 0.42
    batch = None
    batch_count = 0
    moves = []  # (line, layer, kind, a, b, z0, z1, feature, tool, width, batch)
    tool_changes = []  # (line, layer, new tool)
    notes = []  # (layer, text)
    with open(path, "r", errors="replace") as f:
        for line_no, raw in enumerate(f, 1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith(";"):
                if line.startswith("; CHANGE_LAYER") or line.startswith(";LAYER_CHANGE"):
                    layer += 1
                elif line.startswith(";Z:") or line.startswith("; Z_HEIGHT:"):
                    layer_z[layer] = float(line.split(":", 1)[1])
                elif line.startswith("; FEATURE:") or line.startswith(";TYPE:"):
                    feature = line.split(":", 1)[1].strip()
                elif line.startswith(";WIDTH:") or line.startswith("; LINE_WIDTH:"):
                    width = float(line.split(":", 1)[1])
                elif line.startswith("; BATCHED_INTERFACE_START"):
                    batch = batch_count
                    batch_count += 1
                elif line.startswith("; BATCHED_INTERFACE_END"):
                    batch = None
                elif line.startswith("; batched interface:"):
                    notes.append((layer, line[2:]))
                continue
            code = line.split(";", 1)[0].strip()
            if code == "M82":
                relative_e = False
                continue
            if code == "M83":
                relative_e = True
                continue
            if code.startswith("G92"):
                for k, v in WORD_RE.findall(code):
                    if k == "E":
                        e_abs = float(v)
                continue
            m = TOOL_RE.match(code)
            if m:
                t = int(m.group(1))
                # T255 and T1000+ are Bambu firmware helpers, not filament slots.
                if t < 64:
                    if layer >= 0 and t != tool:
                        tool_changes.append((line_no, layer, t))
                    tool = t
                continue
            if not MOVE_RE.match(code):
                continue
            words = dict(WORD_RE.findall(code))
            new = list(pos)
            for i, k in enumerate("XYZ"):
                if k in words:
                    new[i] = float(words[k])
            de = 0.0
            if "E" in words:
                e = float(words["E"])
                de = e if relative_e else e - e_abs
                e_abs = e_abs + e if relative_e else e
            xy_moves = abs(new[0] - pos[0]) > EPS or abs(new[1] - pos[1]) > EPS
            if layer >= 0:
                kind = None
                if de > 0 and xy_moves:
                    kind = "extrude"
                elif de >= 0:
                    # Wipes (negative E with XY) retrace the last extrusion; skip them.
                    kind = "travel"
                if kind and (xy_moves or abs(new[2] - pos[2]) > EPS):
                    moves.append((line_no, layer, kind, (pos[0], pos[1]), (new[0], new[1]), pos[2], new[2], feature, tool, width, batch))
            pos = new
    return moves, tool_changes, notes, layer_z


def find_runs(moves, layer_z):
    levels = sorted({round(mv[6], 3) for mv in moves if mv[2] == "extrude" and mv[7] == "Support interface"})
    if not levels:
        return []
    # Interface on consecutive print_z levels of the file is one run.
    all_z = sorted({round(z, 3) for z in layer_z.values()} | set(levels))
    index = {z: i for i, z in enumerate(all_z)}
    runs = [[levels[0]]]
    for z in levels[1:]:
        if index[z] == index[runs[-1][-1]] + 1:
            runs[-1].append(z)
        else:
            runs.append([z])
    return [Run(r) for r in runs]


def check(path, nozzle_diameter, lead, tolerance):
    moves, tool_changes, notes, layer_z = parse(path)
    runs = find_runs(moves, layer_z)
    r0 = 1.25 * nozzle_diameter
    by_level = {}
    for run in runs:
        for z in run.levels:
            by_level[z] = run
    for mv in moves:
        if mv[2] == "extrude" and mv[7] == "Support interface":
            run = by_level[round(mv[6], 3)]
            run.first_layer = mv[1] if run.first_layer is None else min(run.first_layer, mv[1])
            run.last_layer = mv[1] if run.last_layer is None else max(run.last_layer, mv[1])
            run.interface_tool = mv[8]
            if mv[10] is not None:
                run.batches.add(mv[10])

    def run_of(layer):
        for run in runs:
            if run.first_layer <= layer <= run.last_layer + 1:
                return run
        return None

    for layer, text in notes:
        for run in runs:
            if run.first_layer - 1 <= layer <= run.last_layer + 1 and text not in run.notes:
                run.notes.append(text)
    for line_no, layer, t in tool_changes:
        run = run_of(layer)
        if run is not None:
            run.tool_changes += 1
            run.interface_loads += t == run.interface_tool

    material = MaterialMap()
    reach = r0 + max(lead, 0.4) + MARGIN + 0.5
    for line_no, layer, kind, a, b, z0, z1, feat, t, width, batch in moves:
        run = run_of(layer)
        if run is not None:
            if kind == "extrude":
                run.extrusions_checked += 1
                for x, y in samples(a, b, 2 * STEP):
                    for top, d in material.near(x, y, reach):
                        delta = top - z1
                        if delta <= 0.01:
                            continue
                        if run.deepest_step is None or delta > run.deepest_step[0]:
                            run.deepest_step = (delta, line_no)
                        margin = d - (r0 + delta + MARGIN)
                        if run.worst_margin is None or margin < run.worst_margin[0]:
                            run.worst_margin = (margin, line_no, delta)
                        if margin < -tolerance:
                            run.margin_violations.append((margin, line_no, delta))
            else:
                # The tip and its cone over the material: positive = clear. A move that changes Z and
                # XY together is checked at its lower end along its whole path.
                z = min(z0, z1)
                clearance = None
                for x, y in samples(a, b, 2 * STEP):
                    for top, d in material.near(x, y, r0 + 1.0):
                        c = z + max(0.0, d - r0) - top
                        clearance = c if clearance is None else min(clearance, c)
                if clearance is not None:
                    run.travels_checked += 1
                    if run.min_travel is None or clearance < run.min_travel[0]:
                        run.min_travel = (clearance, line_no)
                    if clearance < -EPS:
                        run.travels_through.append((clearance, line_no))
        if kind == "extrude":
            material.deposit(a, b, z1, width)
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("gcode")
    ap.add_argument("--nozzle-diameter", type=float, help="default: nozzle_diameter from the config block, else 0.4")
    ap.add_argument("--lead", type=float, help="default: support_interface_batch_lead from the config block, else 0.4")
    ap.add_argument("--tolerance", type=float, default=0.05, help="sampling slack on d - d_req (mm)")
    ap.add_argument("--worst", type=int, default=5, help="list this many worst moves per run")
    args = ap.parse_args()

    config = read_config(args.gcode)
    nozzle = args.nozzle_diameter or first_float(config.get("nozzle_diameter"), 0.4)
    lead = args.lead if args.lead is not None else first_float(config.get("support_interface_batch_lead"), 0.4)
    nozzle_height = first_float(config.get("nozzle_height"), None)

    runs = check(args.gcode, nozzle, lead, args.tolerance)
    failed = False
    print(f"{len(runs)} interface run(s) in {args.gcode}")
    print(f"nozzle {nozzle} mm: tip radius r0 {1.25 * nozzle:.2f} mm, d_req = r0 + delta + {MARGIN}; lead {lead} mm"
          + (f"; nozzle_height {nozzle_height} mm, lead limit {nozzle_height - MARGIN:.2f} mm" if nozzle_height else ""))
    for i, run in enumerate(runs, 1):
        print(f"\nrun {i}: interface Z {run.levels[0]:.3f}-{run.levels[-1]:.3f} on {len(run.levels)} level(s), "
              f"G-code layers {run.first_layer + 1}-{run.last_layer + 1} (1-based), interface tool T{run.interface_tool}")
        print(f"  batches: {len(run.batches)}" + ("" if run.batches else " (printed as usual)"))
        for text in run.notes:
            print(f"  note: {text}")
        print(f"  tool changes from the run's first layer to the layer after its last: {run.tool_changes}, "
              f"loading the interface filament: {run.interface_loads}")
        if run.worst_margin is None:
            print(f"  extrusions checked: {run.extrusions_checked}, none beside taller material")
        else:
            margin, line_no, delta = run.worst_margin
            step, step_line = run.deepest_step
            print(f"  extrusions checked: {run.extrusions_checked}; beside taller material: min d - d_req {margin:+.3f} mm "
                  f"(line {line_no}, step {delta:.2f} mm), deepest step {step:.2f} mm (line {step_line}) "
                  f"[{'OK' if step <= lead + EPS else 'DEEPER THAN LEAD'}]")
            for m, ln, d in sorted(run.margin_violations)[: args.worst]:
                print(f"    line {ln}: d - d_req {m:+.3f} mm beside a {d:.2f} mm step")
            failed |= bool(run.margin_violations) or step > lead + EPS
        if run.min_travel is not None:
            clearance, line_no = run.min_travel
            print(f"  travels checked: {run.travels_checked}, lowest clearance {clearance:+.3f} mm (line {line_no}), "
                  f"travels through material: {len(run.travels_through)}")
            for c, ln in sorted(run.travels_through)[: args.worst]:
                print(f"    line {ln}: clearance {c:+.3f} mm")
            failed |= bool(run.travels_through)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
