"""Generate stock flow calibrations through the native CLI and inspect their recipes."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile


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
            command = [str(args.binary), '--datadir', str(output / 'data'),
                       '--load-settings', ';'.join(str(args.profiles / path) for path in (
                           'machine/Bambu Lab P1S 0.4 nozzle.json',
                           'process/0.20mm Standard @BBL X1C.json')),
                       '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json'),
                       '--top-surface-pattern', 'monotonic', '--top-surface-fill-order', 'inward',
                       '--calibration', recipe, '--outputdir', str(output), '--export-3mf', 'test.3mf']
            result = subprocess.run(command, capture_output=True, text=True, timeout=180)
            assert result.returncode == 0, (recipe, result.stdout[-4000:], result.stderr[-4000:])
            with zipfile.ZipFile(output / 'test.3mf') as archive:
                config = json.loads(archive.read('Metadata/project_settings.config'))
                model_settings = ET.fromstring(archive.read('Metadata/model_settings.config'))
            ratio = float(config['filament_flow_ratio'][0])
            objects = model_settings.findall('object')
            assert len(objects) > 1, recipe
            measured = []
            for obj in objects:
                settings = {item.get('key'): item.get('value') for item in obj.findall('metadata')}
                modifier = float(settings['name'][9:].replace('m', '-'))
                wanted = (ratio + modifier) / ratio if 'yolo' in recipe else 1 + modifier / 100
                assert abs(float(settings['print_flow_ratio']) - wanted) < 1e-5, settings
                assert settings['top_surface_pattern'] == 'archimedeanchords', settings
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
