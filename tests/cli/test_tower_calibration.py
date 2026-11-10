"""Slice native calibration towers and check their engine runtime modes in emitted G-code."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import tempfile
import zipfile


def outer_wall_speeds(gcode):
    feature = None
    feed = None
    speeds = set()
    for line in gcode.splitlines():
        if line.startswith('; FEATURE: '):
            feature = line[11:]
        if line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')):
            values = {key: float(value) for key, value in re.findall(r'\b([XYZEF])(-?[\d.]+)', line)}
            if 'F' in values:
                feed = values['F'] / 60
            if feature == 'Outer wall' and values.get('E', 0) > 0 and any(key in values for key in ('X', 'Y')):
                speeds.add(round(feed, 2))
    return speeds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('profiles', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        print('SKIP: built slicer unavailable')
        return 77
    ranges = {'pressure-advance-line': (0, .04, .02), 'pressure-advance-tower': (0, .04, .02),
              'retraction': (.2, .6, .2), 'max-volumetric-speed': (4, 8, 1), 'vfa': (40, 80, 20),
              'temperature': (250, 240, 5), 'overhang-fan': (20, 40, 10), 'part-fan': (20, 40, 10),
              'bridge-fan': (20, 40, 10), 'small-feature-fan': (20, 40, 10)}
    with tempfile.TemporaryDirectory(prefix='orca-cli-towers-') as directory:
        root = Path(directory)
        for recipe, (start, end, step) in ranges.items():
            output = root / recipe
            output.mkdir()
            command = [str(args.binary), '--datadir', str(output / 'data'),
                       '--load-settings', ';'.join(str(args.profiles / path) for path in (
                           'machine/Bambu Lab P1S 0.4 nozzle.json', 'process/0.20mm Standard @BBL X1C.json')),
                       '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json'),
                       '--calibration', recipe, '--calibration-start', str(start), '--calibration-end', str(end),
                       '--calibration-step', str(step), '--slice', '0', '--outputdir', str(output),
                       '--filament-flow-ratio', '.95', '--export-3mf', 'tower.3mf']
            result = subprocess.run(command, capture_output=True, text=True, timeout=180)
            assert result.returncode == 0, (recipe, result.stdout[-4000:], result.stderr[-4000:])
            with zipfile.ZipFile(output / 'tower.3mf') as archive:
                config = json.loads(archive.read('Metadata/project_settings.config'))
                gcode = archive.read('Metadata/plate_1.gcode').decode()
            assert config['resonance_avoidance'] == '0', recipe
            changed = config['different_settings_to_system']
            if recipe.startswith('pressure-advance-'):
                advances = {float(value) for value in re.findall(r'^M900 K([\d.]+)', gcode, re.M)}
                assert {0, .02, .04}.issubset(advances), (recipe, advances)
                if recipe == 'pressure-advance-line':
                    assert '; FEATURE: Top surface' in gcode
                    unlabelled = output / 'unlabelled'
                    unlabelled.mkdir()
                    without_numbers = command + ['--calibration-print-numbers', '0']
                    without_numbers[without_numbers.index('--outputdir') + 1] = str(unlabelled)
                    result = subprocess.run(without_numbers, capture_output=True, text=True, timeout=180)
                    assert result.returncode == 0, result.stdout[-2000:] + result.stderr[-2000:]
                    with zipfile.ZipFile(unlabelled / 'tower.3mf') as archive:
                        assert '; FEATURE: Top surface' not in archive.read('Metadata/plate_1.gcode').decode()
            elif recipe == 'retraction':
                ramp = [(float(z), float(length)) for z, length in re.findall(
                    r'; Calib_Retraction_tower: Z_HEIGHT: ([\d.]+), length:([\d.]+)', gcode)]
                assert len(ramp) > 5, ramp
                assert len({length for _, length in ramp}) > 1, ramp
                for z, length in ramp:
                    # The comment rounds Z, so allow either side of an exact integer boundary.
                    expected = {start + math.floor(max(0, z - .4 + delta)) * step for delta in (-1e-6, 1e-6)}
                    assert any(abs(length - value) < 1e-5 for value in expected), (z, length, expected)
                # Firmware retraction would hide the lengths from the emitted moves.
                assert config['use_firmware_retraction'] == '0'
                actual_retracts = {round(-float(value), 3) for value in re.findall(r'^G1 E(-[\d.]+)(?:\s|$)', gcode, re.M)}
                assert len(actual_retracts.intersection({round(length, 3) for _, length in ramp})) >= 2, actual_retracts
            elif recipe == 'max-volumetric-speed':
                assert float(config['filament_max_volumetric_speed'][0]) >= 200
                assert 'filament_max_volumetric_speed' in changed[1].split(';')
                speeds = outer_wall_speeds(gcode)
                assert len(speeds) >= 3, speeds
                # A fixed profile outer-wall speed would never produce this low ramp.
                bead = (.7 - .32 * (1 - math.pi / 4)) * .32 * .95
                assert abs(max(speeds) - math.floor((start + 5 * step) / bead + .5)) <= 2, speeds
            elif recipe == 'vfa':
                speeds = outer_wall_speeds(gcode)
                for value in (start, start + step, end):
                    assert any(abs(speed - value) < .1 for speed in speeds), (value, speeds)
            elif recipe == 'temperature':
                assert 'nozzle_temperature' in changed[1].split(';')
                temperatures = {int(value) for value in re.findall(r'^M104\s+S(\d+)', gcode, re.M)}
                assert {250, 245, 240}.issubset(temperatures), temperatures
            else:
                fan_values = []
                for line in gcode.splitlines():
                    if line.startswith('M106 '):
                        values = dict(re.findall(r'\b([PS])(\d+)', line))
                        if values.get('P', '1') == '1' and 'S' in values:
                            fan_values.append(int(values['S']))
                for percent in (start, start + step, end):
                    assert any(abs(value - percent * 255 / 100) <= 1 for value in fan_values), (recipe, percent, fan_values)
            readback = output / 'reopened.json'
            reopened = subprocess.run([str(args.binary), '--datadir', str(output / 'data'), '--export-settings',
                                       str(readback), str(output / 'tower.3mf')], capture_output=True, text=True, timeout=60)
            assert reopened.returncode == 0, (recipe, reopened.stdout[-2000:], reopened.stderr[-2000:])
            saved = json.loads(readback.read_text())
            for key in ('resonance_avoidance', 'spiral_mode', 'filament_max_volumetric_speed',
                        'nozzle_temperature', 'nozzle_temperature_initial_layer', 'overhang_fan_threshold'):
                assert saved[key] == config[key], (recipe, key, saved[key], config[key])
            print(f'{recipe}: native runtime calibration G-code PASS')

        for flags, message in ((['--calibration', 'retraction'], 'requires --slice'),
                               (['--calibration', 'temperature', '--slice', '0', '--calibration-start', '240',
                                 '--calibration-end', '250', '--calibration-step', '5'], 'descending range')):
            output = root / ('invalid-' + str(len(flags)))
            output.mkdir()
            invalid = command[:command.index('--calibration')] + flags + ['--outputdir', str(output), '--export-3mf', 'invalid.3mf']
            result = subprocess.run(invalid, capture_output=True, text=True, timeout=30)
            assert result.returncode != 0
            assert message in result.stdout + result.stderr, result.stdout[-2000:] + result.stderr[-2000:]
            assert not (output / 'invalid.3mf').exists()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
