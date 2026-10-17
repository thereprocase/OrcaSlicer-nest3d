"""Check release packaging without network access or release credentials."""
import hashlib
import importlib.util
import json
import os
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from subprocess import CompletedProcess
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

    def test_archive_only_leaves_rolling_release_and_tag_untouched(self):
        self.platforms()
        output = self.root / 'archive'
        argv = ['publish', str(self.artifacts), '--sha', 'a' * 40,
                '--run-id', '123', '--output', str(output), '--archive-only']
        with patch.object(sys, 'argv', argv), patch.dict(os.environ, {
                'GH_REPO': publisher.REPOSITORY, 'GITHUB_REF': 'refs/heads/repro-dev'}), \
                patch.object(publisher, 'publish_build_archive') as archive, \
                patch.object(publisher.subprocess, 'run') as run:
            publisher.main()
        archive.assert_called_once()
        run.assert_not_called()

    def test_archive_upload_finishes_before_publication(self):
        with patch.object(publisher, 'ensure_archive_tag'), patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 1, '')
            tag = publisher.publish_build_archive('a' * 40, '123', [Path('app.exe')], Path('notes.md'))
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(tag, 'repro-build-123-aaaaaaaaaaaa')
        self.assertIn('--draft', commands[1])
        self.assertIn('--verify-tag', commands[1])
        self.assertNotIn('--target', commands[1])
        self.assertEqual(commands[2][:3], ['gh', 'release', 'upload'])
        self.assertIn('--draft=false', commands[3])

    def test_published_archive_is_never_overwritten(self):
        with patch.object(publisher, 'ensure_archive_tag'), patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 0, json.dumps({'isDraft': False, 'targetCommitish': 'a' * 40}))
            publisher.publish_build_archive('a' * 40, '123', [], Path('notes.md'))
        self.assertEqual(run.call_count, 1)

    def test_unfinished_archive_can_resume_upload(self):
        with patch.object(publisher, 'ensure_archive_tag'), patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 0, json.dumps({'isDraft': True, 'targetCommitish': 'a' * 40}))
            publisher.publish_build_archive('a' * 40, '123', [Path('app.exe')], Path('notes.md'))
        self.assertEqual(run.call_args_list[1].args[0][:3], ['gh', 'release', 'upload'])
        self.assertEqual(run.call_count, 3)

    def test_archive_commit_collision_stops_publication(self):
        ref = 'refs/tags/repro-build-123-' + 'a' * 12
        with patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 0, 'b' * 40 + '\t' + ref + '\n')
            with self.assertRaisesRegex(ValueError, 'different commit'):
                publisher.publish_build_archive('a' * 40, '123', [], Path('notes.md'))
        self.assertEqual(run.call_count, 1)

    def test_new_archive_tag_is_pushed_at_the_verified_commit(self):
        with patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 0, '')
            publisher.ensure_archive_tag('repro-build-123-' + 'a' * 12, 'a' * 40)
        self.assertEqual(run.call_args_list[1].args[0], [
            'git', 'push', 'origin', 'a' * 40 + ':refs/tags/repro-build-123-' + 'a' * 12])

    def test_existing_annotated_archive_tag_uses_peeled_commit(self):
        ref = 'refs/tags/repro-build-123-' + 'a' * 12
        with patch.object(publisher.subprocess, 'run') as run:
            run.return_value = CompletedProcess([], 0,
                'b' * 40 + '\t' + ref + '\n' + 'a' * 40 + '\t' + ref + '^{}\n')
            publisher.ensure_archive_tag('repro-build-123-' + 'a' * 12, 'a' * 40)
        self.assertEqual(run.call_count, 1)


if __name__ == '__main__':
    unittest.main()
