"""Check that the portable test bundle contains its standalone API dependencies."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]


class PortableTests(unittest.TestCase):
    def test_flatpak_builds_the_transport_executable(self):
        manifest = (ROOT / 'scripts/flatpak/com.orcaslicer.OrcaSlicer.yml').read_text()
        commands = manifest.split('    test-commands:', 1)[1].split('    cleanup:', 1)[0]
        cmake = (ROOT / 'tests/cli/CMakeLists.txt').read_text()
        targets = re.findall(r'add_executable\((\w+)', cmake)
        self.assertTrue(targets)
        for target in targets:
            self.assertRegex(commands, rf'\b{re.escape(target)}\b')

    def test_flatpak_ships_the_gui_client_used_by_tests(self):
        manifest = (ROOT / 'scripts/flatpak/com.orcaslicer.OrcaSlicer.yml').read_text()
        self.assertRegex(manifest, r'path: ../agent_gui.py\s+dest: scripts')
        self.assertTrue((ROOT / 'scripts/agent_gui.py').is_file())

    def test_workflow_helpers_exist_in_the_checkout(self):
        workflow = (ROOT / '.github/workflows/build_orca.yml').read_text()
        for helper in re.findall(r'run: ./([^\s]+)', workflow):
            self.assertTrue((ROOT / helper).is_file(), helper)

    def test_external_regression_suite_is_immutable(self):
        revision = (ROOT / 'scripts/ci/orca-test-repo.ref').read_text().strip()
        self.assertRegex(revision, r'^[a-f0-9]{40}$')

    def test_release_still_requires_every_platform_test(self):
        workflow = (ROOT / '.github/workflows/build_all.yml').read_text()
        publish = workflow.split('  publish_repro_nightly:', 1)[1]
        needs = publish.split('    needs:', 1)[1].split('    if:', 1)[0]
        for gate in ('check_repro_build_wiring', 'build_linux', 'build_windows',
                     'build_macos_universal', 'unit_tests_linux_x86_64',
                     'unit_tests_linux_aarch64', 'unit_tests_windows_x64',
                     'unit_tests_windows_arm64', 'unit_tests_macos_arm64',
                     'unit_tests_flatpak_x86_64', 'unit_tests_flatpak_aarch64',
                     'slice_check_linux'):
            self.assertIn('- ' + gate + '\n', needs)
        self.assertIn('success()', publish.split('    if:', 1)[1].split('    runs-on:', 1)[0])


if __name__ == '__main__':
    unittest.main()
