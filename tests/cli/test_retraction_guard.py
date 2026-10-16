"""Exercise CLI defaults, overrides and stored retraction controls with the real app."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('profiles', type=Path)
    args = parser.parse_args()
    if not args.binary.is_file():
        print('SKIP: built slicer unavailable')
        return 77
    with tempfile.TemporaryDirectory(prefix='orca-retraction-cli-') as directory:
        root = Path(directory)
        model = root / 'cube.stl'
        vertices = [(x, y, z) for z in (0, 1) for y in (0, 10) for x in (0, 10)]
        with model.open('w') as stream:
            stream.write('solid cube\n')
            for a, b, c, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4),
                               (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
                for triangle in ((a, b, c), (a, c, d)):
                    stream.write('facet normal 0 0 0\nouter loop\n')
                    for index in triangle:
                        stream.write('vertex %g %g %g\n' % vertices[index])
                    stream.write('endloop\nendfacet\n')
            stream.write('endsolid cube\n')

        def slice_case(name, flags, expected, project=None):
            output = root / name
            output.mkdir()
            command = [str(args.binary), '--datadir', str(output / 'data')]
            if project is None:
                command += ['--load-settings', ';'.join(str(args.profiles / path) for path in (
                    'machine/Bambu Lab P1S 0.4 nozzle.json',
                    'process/0.20mm Standard @BBL X1C.json')),
                    '--load-filaments', str(args.profiles / 'filament/Bambu PLA Basic @BBL P1S 0.4 nozzle.json')]
            command += [*flags, '--slice', '0', '--outputdir', str(output),
                        '--export-3mf', 'result.3mf', str(project or model)]
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, timeout=180)
            if result.returncode:
                raise AssertionError(f'{name}: slicer exited {result.returncode}\n{result.stdout[-8000:]}')
            archive_path = output / 'result.3mf'
            with zipfile.ZipFile(archive_path) as archive:
                config = json.loads(archive.read('Metadata/project_settings.config'))
                gcode = archive.read('Metadata/plate_1.gcode').decode()
            for key, value in zip(('retraction_count_max', 'retraction_extrusion_window',
                                   'retraction_minimum_extrusion'), expected):
                if isinstance(value, tuple):
                    assert tuple(map(float, config[key])) == value, (name, key, config[key])
                else:
                    assert config[key], (name, key, 'empty vector')
                    assert all(abs(float(item) - value) < 1e-8 for item in config[key]), (name, key, config[key])
            reopened = output / 'reopened.json'
            result = subprocess.run([str(args.binary), '--datadir', str(output / 'data'),
                '--export-settings', str(reopened), str(archive_path)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=180)
            assert result.returncode == 0, result.stdout[-8000:]
            restored = json.loads(reopened.read_text())
            for key in ('retraction_count_max', 'retraction_extrusion_window',
                        'retraction_minimum_extrusion'):
                assert restored[key] == config[key], (name, key, config[key], restored[key])
            print(f'{name}: settings and non-slicing roundtrip PASS', flush=True)
            return archive_path, gcode

        slice_case('defaults', [], (3, 1, .3))
        _, gcode = slice_case('disabled', ['--retraction-guard', '0'], (0, 1, 0))
        assert '; travel retraction skipped:' not in gcode
        slice_case('explicit', ['--retraction-count-max', '2', '--retraction-extrusion-window', '2',
                                '--retraction-minimum-extrusion', '.5'], (2, 2, .5))
        slice_case('per-variant', ['--retraction-count-max', '2,4',
            '--retraction-extrusion-window', '1,2', '--retraction-minimum-extrusion', '.3,.5'],
            ((2, 4), (1, 2), (.3, .5)))
        hard, gcode = slice_case('hard-minimum', ['--retraction-count-max', '0',
            '--retraction-minimum-extrusion', '100000', '--retract-when-changing-layer'], (0, 1, 100000))
        assert '; travel retraction skipped: minimum extrusion' in gcode
        _, gcode = slice_case('reopened', ['--retraction-guard', '0'], (0, 1, 100000), hard)
        assert '; travel retraction skipped: minimum extrusion' in gcode
    print('CLI retraction defaults, explicit flags, disabling and project reopening PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
