"""Profile manager behavior, with optional real Orca readback via ORCA_PROFILE_COMMAND."""

import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'agent_profiles', Path(__file__).resolve().parents[2] / 'scripts/agent_profiles.py')
manager = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(manager)


class ProfilesTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.base = {'type': 'process', 'name': 'Normal 0.4', 'inherits': '',
                     'from': 'User', 'version': '2.3.0.0', 'layer_height': '0.2',
                     'default_acceleration': ['5000', '5000'],
                     'print_extruder_id': ['1', '1'],
                     'print_extruder_variant': ['Direct Drive Standard', 'Direct Drive High Flow'],
                     'wall_loops': '2', 'sparse_infill_density': '15%'}
        self.base_file = self.root / 'base.json'
        manager.write(self.base_file, self.base)
        variant = self.base | {'name': 'Normal 0.6', 'layer_height': '0.24'}
        manager.write(self.root / 'variant.json', variant)
        self.family_file = self.root / 'family.json'
        manager.main(['family', 'create', '--name', 'Normal', '--base', str(self.base_file),
                      '--base-nozzle', '0.4', '--variant', '0.6=' + str(self.root / 'variant.json'),
                      '--out', str(self.family_file)])

    def edit(self, action, key, value=None, diameter=None):
        family = manager.family_read(self.family_file)
        args = ['family', action, str(self.family_file), '--key', key,
                '--expect', manager.digest(family)]
        if value is not None:
            args += ['--value', json.dumps(value)]
        if diameter:
            args += ['--nozzle', diameter]
        return manager.main(args)

    def test_family_preserves_original_values_and_propagates_only_shared_values(self):
        before = manager.build(manager.family_read(self.family_file))
        self.assertEqual(before[1]['config']['layer_height'], '0.24')
        self.edit('set', 'wall_loops', '3')
        result = manager.build(manager.family_read(self.family_file))
        self.assertEqual([p['config']['wall_loops'] for p in result], ['3', '3'])
        self.assertEqual(result[1]['config']['layer_height'], '0.24')
        self.assertTrue(all(p['config']['inherits'] == '' for p in result))
        self.assertEqual([p['id'] for p in before], [p['id'] for p in result])

    def test_explicit_equal_override_survives_base_change_and_reset_restores_inheritance(self):
        self.edit('set', 'wall_loops', '2', '0.6')
        self.edit('set', 'wall_loops', '4')
        result = manager.build(manager.family_read(self.family_file))
        self.assertEqual(result[1]['config']['wall_loops'], '2')
        self.edit('reset', 'wall_loops', diameter='0.6')
        result = manager.build(manager.family_read(self.family_file))
        self.assertEqual(result[1]['config']['wall_loops'], '4')

    def test_stale_family_revision_is_rejected(self):
        revision = manager.digest(manager.family_read(self.family_file))
        self.edit('set', 'wall_loops', '3')
        with self.assertRaisesRegex(ValueError, 'revision changed'):
            manager.main(['family', 'set', str(self.family_file), '--key', 'wall_loops',
                          '--value', '"4"', '--expect', revision])

    def test_parented_input_is_rejected_instead_of_silently_dropping_inherited_values(self):
        with self.assertRaisesRegex(ValueError, 'standalone'):
            manager.check_root(self.base | {'inherits': 'Missing Parent'})

    @patch.object(manager, 'verify', return_value={'status': 'verified'})
    def test_apply_detects_intervening_changes_before_any_write(self, _verify):
        plan = manager.make_plan(manager.family_read(self.family_file), self.root / 'library')
        target = self.root / 'library' / plan['entries'][1]['path']
        manager.write(target, self.base | {'wall_loops': '9'})
        with self.assertRaisesRegex(ValueError, 'changed since plan'):
            manager.apply_plan(plan, ['orca'])
        self.assertFalse((self.root / 'library' / plan['entries'][0]['path']).exists())
        self.assertEqual(manager.read(target)['wall_loops'], '9')

    @patch.object(manager, 'verify', return_value={'status': 'verified'})
    def test_rollback_restores_previous_values_and_rejects_later_edits(self, _verify):
        library = self.root / 'library'
        first = manager.make_plan(manager.family_read(self.family_file), library)
        manager.apply_plan(first, ['orca'])
        self.edit('set', 'wall_loops', '3')
        second = manager.make_plan(manager.family_read(self.family_file), library)
        applied = manager.apply_plan(second, ['orca'])
        manager.main(['profiles', 'rollback', applied['receipt'], '--orca-command', '["orca"]'])
        for entry in first['entries']:
            self.assertEqual(manager.read(library / entry['path']), entry['after'])
        with self.assertRaisesRegex(ValueError, 'changed since plan'):
            manager.main(['profiles', 'rollback', applied['receipt'], '--orca-command', '["orca"]'])

    @patch.object(manager, 'verify', side_effect=ValueError('Orca rejected preset'))
    def test_native_rejection_leaves_library_unmodified(self, _verify):
        plan = manager.make_plan(manager.family_read(self.family_file), self.root / 'library')
        with self.assertRaisesRegex(ValueError, 'Orca rejected'):
            manager.apply_plan(plan, ['orca'])
        self.assertEqual(list((self.root / 'library').rglob('*.json')), [])

    def test_path_escape_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Invalid managed'):
            manager.safe_target(self.root, '../../outside.json')

    def test_cloud_wire_preserves_numeric_and_string_vectors(self):
        config = self.base | {'compatible_printers': ['Printer A', 'Printer; B'],
                              'filament_start_gcode': ['G1 X1\n; "quoted" \\ path']}
        wire = manager.cloud_wire(config)
        self.assertEqual(wire['default_acceleration'], '5000,5000')
        for key, value in config.items():
            self.assertTrue(manager.matches(wire[key], value), key)
        self.assertTrue(manager.matches('0,0;"\\n0.2,0.4444"', ['0,0', '\n0.2,0.4444']))

    def test_build_exports_named_root_files_without_modifying_sources(self):
        destination = self.root / 'export'
        before = self.family_file.read_bytes()
        manager.main(['family', 'build', str(self.family_file), '--out', str(destination)])
        self.assertEqual(manager.read(destination / 'process/base/Normal 0.6.json')['inherits'], '')
        self.assertEqual(before, self.family_file.read_bytes())
        with self.assertRaisesRegex(ValueError, 'already exists'):
            manager.main(['family', 'build', str(self.family_file), '--out', str(destination)])

    @patch.object(manager, 'verify', return_value={})
    def test_partial_write_failure_restores_prior_library(self, _verify):
        library = self.root / 'library'
        plan = manager.make_plan(manager.family_read(self.family_file), library)
        manager.apply_plan(plan, ['orca'])
        self.edit('set', 'wall_loops', '6')
        update = manager.make_plan(manager.family_read(self.family_file), library)
        original_write = manager.write
        failed = False

        def fail_once(path, value):
            nonlocal failed
            if Path(path).name == 'Normal 0.6.json' and not failed:
                failed = True
                raise OSError('simulated disk failure')
            return original_write(path, value)

        with patch.object(manager, 'write', side_effect=fail_once), self.assertRaisesRegex(OSError, 'disk failure'):
            manager.apply_plan(update, ['orca'])
        for entry in plan['entries']:
            self.assertEqual(manager.read(library / entry['path']), entry['after'])

    def test_existing_family_lock_prevents_mutation(self):
        Path(str(self.family_file) + '.lock').mkdir()
        before = self.family_file.read_bytes()
        with self.assertRaisesRegex(ValueError, 'locked'):
            self.edit('set', 'wall_loops', '7')
        self.assertEqual(before, self.family_file.read_bytes())

    def test_wrong_account_is_rejected(self):
        import base64
        token = self.root / 'token'
        payload = base64.urlsafe_b64encode(b'{"sub":"account-a"}').decode().rstrip('=')
        token.write_text('header.' + payload + '.signature')
        with self.assertRaisesRegex(ValueError, 'different account'):
            manager.check_account(token, 'account-b')

    @patch.object(manager, 'check_account', return_value=None)
    @patch.object(manager, 'verify', return_value={})
    def test_cloud_revision_conflict_prevents_push(self, _verify, _account):
        snapshot = self.root / 'snapshot.json'
        profile_id = '00000000-0000-4000-8000-000000000001'
        item = {'id': profile_id, 'name': self.base['name'], 'updated_time': 1, 'content': self.base}
        manager.write(snapshot, {'account_id': 'test-account', 'upserts': [item]})
        with patch.object(manager, 'cloud', return_value={'upserts': [item | {'updated_time': 2}]}) as call:
            with self.assertRaisesRegex(ValueError, 'revision changed'):
                manager.main(['cloud', 'push', '--profile', str(self.base_file),
                              '--snapshot', str(snapshot), '--id', profile_id,
                              '--token-file', 'unused', '--out', str(self.root / 'receipt.json'),
                              '--account', 'test-account',
                              '--orca-command', '["orca"]'])
            self.assertEqual(call.call_count, 1)
            self.assertEqual(call.call_args.args[1], 'pull')

    @patch.object(manager, 'check_account', return_value=None)
    @patch.object(manager, 'verify', return_value={})
    def test_cloud_push_records_backup_and_requires_matching_readback(self, _verify, _account):
        profile_id = '00000000-0000-4000-8000-000000000001'
        old = {'id': profile_id, 'name': self.base['name'], 'updated_time': 1, 'content': self.base}
        snapshot = self.root / 'snapshot.json'
        manager.write(snapshot, {'account_id': 'test-account', 'upserts': [old]})
        after = old | {'updated_time': 2, 'content': manager.cloud_wire(self.base)}
        receipt = self.root / 'cloud-receipt.json'
        with patch.object(manager, 'cloud', side_effect=[{'upserts': [old]}, {'updated_time': 2},
                                                       {'upserts': [after]}]) as call:
            result = manager.main(['cloud', 'push', '--profile', str(self.base_file),
                                   '--snapshot', str(snapshot), '--id', profile_id,
                                   '--token-file', 'unused', '--out', str(receipt),
                                   '--account', 'test-account', '--orca-command', '["orca"]'])
        self.assertEqual(result['status'], 'cloud_verified')
        self.assertEqual(manager.read(receipt)['before'], old)
        self.assertEqual(call.call_args_list[1].args[2]['original_updated_time'], '1')

    @unittest.skipUnless(os.environ.get('ORCA_PROFILE_COMMAND'), 'Real Orca command not configured')
    def test_real_orca_preserves_both_variants_from_local_and_cloud_formats(self):
        command = json.loads(os.environ['ORCA_PROFILE_COMMAND'])
        for config in (self.base, manager.cloud_wire(self.base)):
            self.assertEqual(manager.verify(config, command)['status'], 'orca_readback_verified')
        with self.assertRaisesRegex(ValueError, 'changed or dropped'):
            manager.verify(self.base | {'made_up_setting': '5'}, command)


if __name__ == '__main__':
    unittest.main()
