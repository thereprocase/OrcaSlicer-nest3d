"""Check PA pattern batch layout and native custom G-code on every generated plate."""
import argparse
from pathlib import Path
import re
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
    with tempfile.TemporaryDirectory(prefix='orca-cli-pa-pattern-') as directory:
        output = Path(directory)
        command = [str(args.binary), '--datadir', str(output / 'data'),
                   '--load-settings', ';'.join(str(args.profiles / path) for path in (
                       'machine/Bambu Lab P1S 0.4 nozzle.json', 'process/0.20mm Standard @BBL X1C.json')),
                   '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json'),
                   '--calibration', 'pressure-advance-pattern', '--calibration-start', '0',
                   '--calibration-end', '.04', '--calibration-step', '.02', '--calibration-speeds', '30,60',
                   '--calibration-accelerations', '1000,2000', '--printable-area', '0x0,80x0,80x80,0x80',
                   '--bed-exclude-area', '', '--slice', '0', '--outputdir', str(output), '--export-3mf', 'pattern.3mf']
        result = subprocess.run(command, capture_output=True, text=True, timeout=300)
        assert result.returncode == 0, (result.returncode, result.stdout[-5000:], result.stderr[-5000:])
        with zipfile.ZipFile(output / 'pattern.3mf') as archive:
            settings = ET.fromstring(archive.read('Metadata/model_settings.config'))
            objects = settings.findall('object')
            assert len(objects) == 4, len(objects)
            combinations = set()
            for obj in objects:
                config = {item.get('key'): item.get('value') for item in obj.findall('metadata')}
                combinations.add((float(config['outer_wall_speed']), float(config['outer_wall_acceleration'])))
            assert combinations == {(30, 1000), (60, 1000), (30, 2000), (60, 2000)}, combinations
            plates = [name for name in archive.namelist() if re.fullmatch(r'Metadata/plate_\d+\.gcode', name)]
            assert len(plates) >= 2, plates
            blocks = []
            for name in plates:
                gcode = archive.read(name).decode()
                custom = re.findall(r'; start pressure advance pattern for layer\n(.*?); end pressure advance pattern for layer', gcode, re.S)
                assert custom, name
                for block in custom:
                    advances = {float(value) for value in re.findall(r'^M900 K([\d.]+)', block, re.M)}
                    assert {0, .02, .04}.issubset(advances), (name, advances)
                    for line in block.splitlines():
                        if line.startswith(('G0 ', 'G1 ')):
                            for axis, value in re.findall(r'\b([XY])(-?[\d.]+)', line):
                                assert -.1 <= float(value) <= 80.1, (name, axis, value)
                blocks.extend(custom)
            assert len(blocks) == 4 * len(objects), (len(blocks), len(objects))
            joined = '\n'.join(blocks)
            accelerations = {int(float(value)) for value in re.findall(r'^M204\s+[PS]([\d.]+)', joined, re.M)}
            assert {1000, 2000}.issubset(accelerations), accelerations
            feeds = {int(float(value)) for value in re.findall(r'\bF([\d.]+)', joined)}
            assert {1800, 3600}.issubset(feeds), feeds
            saved = ET.fromstring(archive.read('Metadata/custom_gcode_per_layer.xml'))
            saved_plates = saved.findall('plate')
            assert {int(plate.find('plate_info').get('id')) for plate in saved_plates} == {
                int(re.search(r'plate_(\d+)\.gcode', name).group(1)) for name in plates}
            saved_blocks = []
            for plate in saved_plates:
                layers = plate.findall('layer')
                assert len(layers) == 4, len(layers)
                for layer in layers:
                    saved_blocks.extend(re.findall(
                        r'; start pressure advance pattern for layer\n(.*?); end pressure advance pattern for layer',
                        layer.get('extra'), re.S))
            assert len(saved_blocks) == len(blocks), (len(saved_blocks), len(blocks))
        print(f'PA pattern: four speed/acceleration combinations, {len(plates)} native plates, custom G-code PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
