#!/usr/bin/env python3
"""Check the support interface filament changes and nozzle clearances in a sliced G-code file.

Reads an OrcaSlicer G-code file (plain text, absolute XYZ) and reports, for every run of
consecutive G-code layers that print support interface:

  * the tool changes inside the run, and how many of them load the interface filament;
  * for every interface extrusion, the tallest material already printed within --reach mm of
    it, relative to the nozzle (positive = the nozzle prints below a taller neighbour);
  * for every travel, its clearance: nozzle Z minus the tallest material already printed
    within --tip mm of the travel path (negative = the nozzle moves through material).

Batched support interface (support_interface_batch) prints a whole run inside one layer between
"; BATCHED_INTERFACE_START" and "; BATCHED_INTERFACE_END"; such a run shows up as one layer.

Standard library only. Usage:
    check_support_interface_batch.py plate_1.gcode [--lead 0.4] [--reach 1.0] [--tip 0.25]
Exit status 1 if an interface extrusion sits more than --lead below a neighbour or a travel's
clearance is below zero, else 0.
"""

import argparse
import math
import re
import sys

CELL = 0.2  # mm, height map resolution
STEP = 0.1  # mm, sampling step along moves
EPS = 1e-3

MOVE_RE = re.compile(r"^G[0-3](?:\s|$)")
WORD_RE = re.compile(r"([XYZEF])(-?\d*\.?\d+)")
TOOL_RE = re.compile(r"^T(\d+)\s*(?:;.*)?$")


class HeightMap:
    """Tallest top of extruded material per CELL x CELL cell."""

    def __init__(self):
        self.cells = {}

    def deposit(self, a, b, z):
        for x, y in samples(a, b):
            key = (math.floor(x / CELL), math.floor(y / CELL))
            if self.cells.get(key, -1.0) < z:
                self.cells[key] = z

    def max_near(self, a, b, radius):
        """Tallest material within `radius` of the segment a-b, or None."""
        r = int(math.ceil(radius / CELL))
        best = None
        seen = set()
        for x, y in samples(a, b):
            cx, cy = math.floor(x / CELL), math.floor(y / CELL)
            for ix in range(cx - r, cx + r + 1):
                for iy in range(cy - r, cy + r + 1):
                    if (ix, iy) in seen:
                        continue
                    seen.add((ix, iy))
                    z = self.cells.get((ix, iy))
                    if z is None:
                        continue
                    # Distance from the sample to the nearest point of the cell.
                    dx = max(ix * CELL - x, 0.0, x - (ix + 1) * CELL)
                    dy = max(iy * CELL - y, 0.0, y - (iy + 1) * CELL)
                    if dx * dx + dy * dy <= radius * radius and (best is None or z > best):
                        best = z
        return best


def samples(a, b):
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / STEP)))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


class Run:
    def __init__(self, first_layer):
        self.first_layer = first_layer
        self.last_layer = first_layer
        self.interface_z = set()
        self.max_excess = None  # (excess, line number)
        self.min_clearance = None  # (clearance, line number)
        self.travel_count = 0
        self.travels_through = []  # (clearance, line number)
        self.interface_tool = None
        self.tool_changes = 0
        self.interface_loads = 0


def parse(path, reach, tip):
    heights = HeightMap()
    pos = [0.0, 0.0, 0.0]
    relative_e = False
    e_abs = 0.0
    tool = None
    layer = -1
    feature = ""
    layers_with_interface = {}  # layer -> list of interface z

    # Pass 1 collects everything per move; runs are assembled afterwards.
    moves = []  # (line_no, layer, kind, a, b, z_from, z_to, feature, tool)
    tool_changes = []  # (line_no, layer, from_tool, to_tool)
    with open(path, "r", errors="replace") as f:
        for line_no, raw in enumerate(f, 1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith(";"):
                if line.startswith("; CHANGE_LAYER") or line.startswith(";LAYER_CHANGE"):
                    layer += 1
                elif line.startswith("; FEATURE:") or line.startswith(";TYPE:"):
                    feature = line.split(":", 1)[1].strip()
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
                if t < 64 and layer >= 0 and t != tool:
                    tool_changes.append((line_no, layer, tool, t))
                if t < 64:
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
                if de > 0 and xy_moves:
                    kind = "extrude"
                elif de >= 0 or not xy_moves:
                    # Wipes (negative E with XY) retrace the last extrusion; skip them.
                    kind = "travel"
                else:
                    kind = None
                if kind and (xy_moves or abs(new[2] - pos[2]) > EPS):
                    moves.append((line_no, layer, kind, (pos[0], pos[1]), (new[0], new[1]), pos[2], new[2], feature, tool))
            pos = new

    # Runs: consecutive G-code layers that print support interface.
    for mv in moves:
        if mv[2] == "extrude" and mv[7] == "Support interface":
            layers_with_interface.setdefault(mv[1], set()).add(round(mv[6], 3))
    runs = []
    for lay in sorted(layers_with_interface):
        if runs and lay == runs[-1].last_layer + 1:
            runs[-1].last_layer = lay
        else:
            runs.append(Run(lay))
        runs[-1].interface_z |= layers_with_interface[lay]

    def run_of(lay, allow_next=False):
        for r in runs:
            if r.first_layer <= lay <= r.last_layer + (1 if allow_next else 0):
                return r
        return None

    for mv in moves:
        line_no, lay, kind, a, b, z0, z1, feat, t = mv
        run = run_of(lay, allow_next=True)
        if kind == "extrude":
            if feat == "Support interface" and run is not None:
                run.interface_tool = t
                top = heights.max_near(a, b, reach)
                if top is not None:
                    excess = top - z1
                    if run.max_excess is None or excess > run.max_excess[0]:
                        run.max_excess = (excess, line_no)
            heights.deposit(a, b, z1)
        elif run is not None:
            # A move that changes Z and XY together is checked at its lower end along its whole path.
            z = min(z0, z1)
            top = heights.max_near(a, b, tip)
            if top is None:
                continue
            clearance = z - top
            run.travel_count += 1
            if run.min_clearance is None or clearance < run.min_clearance[0]:
                run.min_clearance = (clearance, line_no)
            if clearance < -EPS:
                run.travels_through.append((clearance, line_no))

    for line_no, lay, old, new in tool_changes:
        run = run_of(lay, allow_next=True)
        if run is None:
            continue
        run.tool_changes += 1
    for run in runs:
        for line_no, lay, old, new in tool_changes:
            if run.first_layer <= lay <= run.last_layer + 1 and new == run.interface_tool:
                run.interface_loads += 1
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("gcode")
    ap.add_argument("--lead", type=float, default=0.4, help="allowed height of a neighbour above the nozzle (mm)")
    ap.add_argument("--reach", type=float, default=1.0, help="lateral distance counted as a neighbour of an extrusion (mm)")
    ap.add_argument("--tip", type=float, default=0.25, help="lateral distance a travel must clear (mm)")
    ap.add_argument("--worst", type=int, default=5, help="list this many worst travels per run")
    args = ap.parse_args()

    runs = parse(args.gcode, args.reach, args.tip)
    failed = False
    print(f"{len(runs)} interface run(s) in {args.gcode}")
    for i, run in enumerate(runs, 1):
        zs = sorted(run.interface_z)
        print(f"\nrun {i}: G-code layers {run.first_layer + 1}-{run.last_layer + 1} (1-based), "
              f"interface Z {zs[0]:.3f}-{zs[-1]:.3f} on {len(zs)} level(s), interface tool T{run.interface_tool}")
        print(f"  tool changes in run and the layer after: {run.tool_changes}, loading the interface filament: {run.interface_loads}")
        if run.max_excess is not None:
            excess, line_no = run.max_excess
            verdict = "OK" if excess <= args.lead + EPS else "TOO DEEP"
            print(f"  tallest neighbour within {args.reach} mm of an interface extrusion: {excess:+.3f} mm vs nozzle "
                  f"(line {line_no}) [{verdict}, lead {args.lead}]")
            failed |= excess > args.lead + EPS
        if run.min_clearance is not None:
            clearance, line_no = run.min_clearance
            print(f"  travels checked: {run.travel_count}, lowest clearance {clearance:+.3f} mm (line {line_no}), "
                  f"travels through material: {len(run.travels_through)}")
            for c, ln in sorted(run.travels_through)[: args.worst]:
                print(f"    line {ln}: clearance {c:+.3f} mm")
            failed |= bool(run.travels_through)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
