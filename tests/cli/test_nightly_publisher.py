"""Check release packaging without network access or release credentials."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
import zipfile

spec = importlib.util.spec_from_file_location(
    'publisher', Path(__file__).resolve().parents[2] / 'scripts/publish_repro_nightly.py')
publisher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(publisher)


class NightlyPackages(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(prefix='orca-nightly-test-')
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        self.artifacts = self.root / 'artifacts'
        self.artifacts.mkdir()

    def package(self, directory, filename, content=b'package'):
        path = self.artifacts / directory / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return path

    def platforms(self):
        for directory, filename in (
            ('OrcaSlicer_Linux_ubuntu_2404_V2.5.0-dev', 'orca.AppImage'),
            ('OrcaSlicer_Mac_universal_V2.5.0-dev', 'orca.dmg'),
            ('OrcaSlicer_Windows_V2.5.0-dev', 'orca.exe'),
            ('OrcaSlicer-Linux-flatpak_V2.5.0-dev_x86_64.flatpak', 'orca.flatpak')):
            self.package(directory, filename)

    def test_portable_resources_and_checksums_survive_packaging(self):
        self.platforms()
        portable = 'OrcaSlicer_Windows_V2.5.0-dev_portable'
        self.package(portable, 'orca-slicer.exe', b'application')
        self.package(portable, 'resources/profiles/preset.json', b'{"profile": true}')
        output = self.root / 'release'
        assets = publisher.prepare_assets(self.artifacts, output)
        self.assertEqual(len(assets), 6)
        with zipfile.ZipFile(output / 'OrcaSlicer_Windows_nightly_portable.zip') as archive:
            self.assertEqual(archive.read('resources/profiles/preset.json'), b'{"profile": true}')
        for line in (output / 'SHA256SUMS').read_text().splitlines():
            digest, name = line.split(None, 1)
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)

    def test_missing_platform_does_not_produce_a_releasable_set(self):
        self.package('OrcaSlicer_Linux', 'orca.AppImage')
        with self.assertRaisesRegex(ValueError, 'Missing platform'):
            publisher.prepare_assets(self.artifacts, self.root / 'release')

    def test_duplicate_normalized_asset_names_are_rejected(self):
        self.package('OrcaSlicer_Linux_V2.5.0-dev', 'orca.AppImage')
        self.package('OrcaSlicer_Linux_V2.6.0-dev', 'orca.AppImage')
        with self.assertRaisesRegex(ValueError, 'Duplicate release asset'):
            publisher.prepare_assets(self.artifacts, self.root / 'release')

    def test_intermediate_macos_bundles_are_excluded(self):
        self.platforms()
        self.package('OrcaSlicer_Mac_bundle_arm64_commit', 'intermediate.bin')
        assets = publisher.prepare_assets(self.artifacts, self.root / 'release')
        self.assertFalse(any('bundle' in asset.name for asset in assets))


if __name__ == '__main__':
    unittest.main()
