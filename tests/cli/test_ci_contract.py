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
        condition = publish.split('    if:', 1)[1].split('    runs-on:', 1)[0]
        self.assertIn('!failure()', condition)
        for gate in ('check_repro_build_wiring', 'build_linux', 'build_windows', 'slice_check_linux',
                     'unit_tests_linux_x86_64', 'unit_tests_windows_x64'):
            self.assertIn(f"needs.{gate}.result == 'success'", condition)

    def test_fork_builds_macos_only_on_request(self):
        # Hosted macOS runners are contended; the fork builds macOS only from a manual run with build-macos.
        workflow = (ROOT / '.github/workflows/build_all.yml').read_text()
        self.assertIn('      build-macos:\n', workflow)
        macos = workflow.split('  build_macos_arch:', 1)[1].split('    uses:', 1)[0]
        self.assertIn("(github.repository != 'thereprocase/OrcaSlicer-nest3d' || inputs.build-macos)", macos)
        publish = workflow.split('  publish_repro_nightly:', 1)[1]
        condition = publish.split('    if:', 1)[1].split('    runs-on:', 1)[0]
        self.assertIn("needs.build_macos_universal.result == 'skipped'", condition)
        self.assertIn("inputs.build-macos && '--require-mac'", publish)

    def test_linux_amd64_tests_run_on_the_build_runner(self):
        # The amd64 build leg and its unit tests each compute the LINUX_RUNNER choice;
        # if the copies drift, tests run on a different machine from the build.
        workflow = (ROOT / '.github/workflows/build_all.yml').read_text()
        choices = re.findall(r"\(vars\.LINUX_RUNNER && .*?'ubuntu-24\.04'\)", workflow)
        self.assertEqual(len(choices), 2)
        self.assertEqual(choices[0], choices[1])


if __name__ == '__main__':
    unittest.main()
