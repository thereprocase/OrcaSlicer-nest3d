"""Generate stock flow calibrations through the native CLI and inspect their recipes."""
import argparse
import json
import math
import re
import statistics
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile


def check_gcode(gcode):
    """Catch per-patch MVS clipping and modifier settings that do not affect extrusion."""
    obj = feature = None
    position = {}
    feed = None
    speeds = {}
    extrusion = {}
    width = None
    for line in gcode.splitlines():
        if line.startswith('; printing object '):
            obj = line.split(' id:')[0][18:]
        elif line.startswith('; FEATURE: '):
            feature = line[11:]
        elif line.startswith('; LINE_WIDTH: '):
            width = float(line[14:])
        if line.startswith(('G0 ', 'G1 ', 'G2 ', 'G3 ')):
            values = {key: float(value) for key, value in re.findall(r'\b([XYZEF])(-?[\d.]+)', line)}
            if 'F' in values:
                feed = values['F'] / 60
            if obj and feature == 'Top surface' and values.get('E', 0) > 0 and any(k in values for k in ('X', 'Y')):
                speeds.setdefault(obj, []).append(feed)
                if line.startswith('G1 ') and all(k in position for k in ('X', 'Y')) and width is not None and abs(width - .48) < .001:
                    distance = math.hypot(values.get('X', position['X']) - position['X'], values.get('Y', position['Y']) - position['Y'])
                    if distance > .2:
                        extrusion.setdefault(obj, []).append(values['E'] / distance)
            position.update({key: value for key, value in values.items() if key in ('X', 'Y', 'Z')})
    assert len(speeds) == 9, speeds.keys()
    for name, values in speeds.items():
        assert abs(max(values) - 57) < .01, (name, min(values), max(values))
    assert len(extrusion) == 9, extrusion.keys()
    baseline = statistics.median(extrusion['flowrate_0'])
    for name, values in extrusion.items():
        expected = 1 + float(name[9:].replace('m', '-')) / 100
        actual = statistics.median(values) / baseline
        assert abs(actual - expected) < .015, (name, actual, expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('profiles', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        print('SKIP: built slicer unavailable')
        return 77
    with tempfile.TemporaryDirectory(prefix='orca-cli-calibration-') as directory:
        root = Path(directory)
        for recipe in ('flow-ratio', 'flow-ratio-fine', 'flow-ratio-yolo', 'flow-ratio-yolo-fine'):
            output = root / recipe
            output.mkdir()
            surface_pattern = "monotonic" if recipe == "flow-ratio-yolo-fine" else "archimedeanchords"
            command = [str(args.binary), '--datadir', str(output / 'data'),
                       '--load-settings', ';'.join(str(args.profiles / path) for path in (
                           'machine/Bambu Lab P1S 0.4 nozzle.json',
                           'process/0.20mm Standard @BBL X1C.json')),
                       '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json'),
                       '--filament-max-volumetric-speed', '6', '--filament-flow-ratio', '0.95',
                       '--top-surface-speed', '200', '--internal-solid-infill-speed', '200',
                       '--top-surface-pattern', 'monotonic', '--top-surface-fill-order', 'inward',
                       '--calibration', recipe, '--calibration-pattern', surface_pattern, '--outputdir', str(output), '--export-3mf', 'test.3mf']
            if recipe == 'flow-ratio':
                command[command.index('--export-3mf'):command.index('--export-3mf')] = ['--slice', '0']
            result = subprocess.run(command, capture_output=True, text=True, timeout=180)
            assert result.returncode == 0, (recipe, result.stdout[-4000:], result.stderr[-4000:])
            with zipfile.ZipFile(output / 'test.3mf') as archive:
                config = json.loads(archive.read('Metadata/project_settings.config'))
                model_settings = ET.fromstring(archive.read('Metadata/model_settings.config'))
                gcode = archive.read('Metadata/plate_1.gcode').decode() if recipe == 'flow-ratio' else None
            ratio = float(config['filament_flow_ratio'][0])
            objects = model_settings.findall('object')
            assert len(objects) > 1, recipe
            measured = []
            for obj in objects:
                settings = {item.get('key'): item.get('value') for item in obj.findall('metadata')}
                modifier = float(settings['name'][9:].replace('m', '-'))
                wanted = (ratio + modifier) / ratio if 'yolo' in recipe else 1 + modifier / 100
                assert abs(float(settings['print_flow_ratio']) - wanted) < 1e-5, settings
                assert settings['top_surface_pattern'] == surface_pattern, settings
                assert settings['top_surface_fill_order'] == 'default', settings
                assert settings['calib_flowrate_topinfill_special_order'] == '1', settings
                assert settings['center_of_surface_pattern'] == 'each_surface', settings
                assert settings['top_shell_layers'] == '5', settings
                assert settings['bottom_shell_layers'] == '2', settings
                measured.append(wanted)
            assert len(set(measured)) == len(measured), (recipe, measured)
            assert config['resonance_avoidance'] == '0'
            assert config['layer_height'] == '0.2'
            assert all(float(value) == 0 for value in config['retraction_count_max']), config['retraction_count_max']
            if gcode is not None:
                check_gcode(gcode)
            print(f'{recipe}: stock assets, distinct modifiers and shared GUI recipe PASS')

        invalid = subprocess.run([str(args.binary), '--datadir', str(root / 'invalid-data'),
                                  '--calibration', 'not-a-calibration', '--export-3mf', str(root / 'invalid.3mf')],
                                 capture_output=True, text=True, timeout=30)
        assert invalid.returncode != 0
        assert 'Unsupported --calibration recipe' in invalid.stdout + invalid.stderr
        assert not (root / 'invalid.3mf').exists()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
