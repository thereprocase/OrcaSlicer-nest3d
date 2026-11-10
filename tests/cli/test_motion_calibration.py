"""Exercise input shaping and cornering calibration through each supported firmware writer."""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile
import zipfile


def values(pattern, text):
    return [float(value) for value in re.findall(pattern, text, re.M)]


def check_ramp(actual, start, end, precision):
    assert len(set(actual)) > 10, actual
    assert abs(min(actual) - start) <= precision, (min(actual), start)
    assert abs(max(actual) - end) <= precision, (max(actual), end)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('profiles', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        print('SKIP: built slicer unavailable')
        return 77
    with tempfile.TemporaryDirectory(prefix='orca-cli-motion-') as directory:
        root = Path(directory)
        for flavor in ('klipper', 'reprapfirmware', 'marlin2'):
            for recipe in ('input-shaping-frequency', 'input-shaping-damping', 'cornering'):
                output = root / f'{flavor}-{recipe}'
                output.mkdir()
                # Common stock fixture; select the actual firmware writer explicitly for each run.
                command = [str(args.binary), '--datadir', str(output / 'data'),
                           '--load-settings', ';'.join(str(args.profiles / path) for path in (
                               'machine/Bambu Lab P1S 0.4 nozzle.json', 'process/0.20mm Standard @BBL X1C.json')),
                           '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json'),
                           '--gcode-flavor', flavor, '--calibration', recipe, '--calibration-test-model', '1',
                           '--slice', '0', '--outputdir', str(output), '--export-3mf', 'motion.3mf']
                if recipe == 'input-shaping-frequency':
                    command += ['--calibration-start', '.1', '--calibration-frequencies',
                                '30,60,40,80' if flavor == 'klipper' else '30,60']
                elif recipe == 'input-shaping-damping':
                    command += ['--calibration-start', '.1', '--calibration-end', '.3',
                                '--calibration-frequencies', '40,40']
                else:
                    command += ['--calibration-start', '.01' if flavor == 'marlin2' else '1',
                                '--calibration-end', '.15' if flavor == 'marlin2' else '2']
                    if flavor == 'marlin2':
                        command += ['--machine-max-junction-deviation', '.2']
                result = subprocess.run(command, capture_output=True, text=True, timeout=180)
                assert result.returncode == 0, (flavor, recipe, result.stdout[-4000:], result.stderr[-4000:])
                with zipfile.ZipFile(output / 'motion.3mf') as archive:
                    gcode = archive.read('Metadata/plate_1.gcode').decode()
                body = gcode[gcode.index('; CHANGE_LAYER'):]
                if recipe == 'input-shaping-frequency':
                    if flavor == 'klipper':
                        check_ramp(values(r'^SET_INPUT_SHAPER.*\bSHAPER_FREQ_X=([\d.]+)', body), 30, 60, .02)
                        check_ramp(values(r'^SET_INPUT_SHAPER.*\bSHAPER_FREQ_Y=([\d.]+)', body), 40, 80, .02)
                    else:
                        check_ramp(values(r'^M593.*\bF([\d.]+)', body), 30, 60, .02)
                elif recipe == 'input-shaping-damping':
                    if flavor == 'klipper':
                        check_ramp(values(r'^SET_INPUT_SHAPER.*\bDAMPING_RATIO_X=([\d.]+)', body), .1, .3, .002)
                    else:
                        parameter = 'S' if flavor == 'reprapfirmware' else 'D'
                        check_ramp(values(r'^M593.*\b' + parameter + r'([\d.]+)', body), .1, .3, .002)
                elif flavor == 'klipper':
                    check_ramp(values(r'^SET_VELOCITY_LIMIT.*\bSQUARE_CORNER_VELOCITY=([\d.]+)', body), 1, 2, .002)
                elif flavor == 'marlin2':
                    check_ramp(values(r'^M205 J([\d.]+)', body), .01, .15, .002)
                else:
                    check_ramp(values(r'^M205 X([\d.]+)', body), 1, 2, .002)
                print(f'{flavor} {recipe}: native calibration writer ramp PASS')
        prefix = command[:command.index('--gcode-flavor')]
        for index, malformed in enumerate(('30,broken', '30,60,', '30,60,70', 'nan,60')):
            output = root / f'invalid-{index}'
            output.mkdir()
            result = subprocess.run(prefix + ['--gcode-flavor', 'klipper', '--calibration', 'input-shaping-frequency',
                                    '--calibration-frequencies', malformed, '--slice', '0', '--outputdir', str(output),
                                    '--export-3mf', 'invalid.3mf'], capture_output=True, text=True, timeout=30)
            assert result.returncode != 0, malformed
            assert not (output / 'invalid.3mf').exists(), malformed
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
