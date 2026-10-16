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
        item = {'id': profile_id, 'name': self.base['name'], 'updated_time': 1, 'content': self.base | {'wall_loops': '1'}}
        manager.write(snapshot, {'account_id': 'test-account', 'upserts': [item]})
        with patch.object(manager, 'cloud', return_value={'upserts': [item | {'updated_time': 2}]}) as call:
            result = manager.main(['cloud', 'push', '--profile', str(self.base_file),
                              '--snapshot', str(snapshot), '--id', profile_id,
                              '--token-file', 'unused', '--out', str(self.root / 'receipt.json'),
                              '--account', 'test-account',
                              '--orca-command', '["orca"]'])
            self.assertEqual(result['status'], 'conflicted')
            self.assertEqual(call.call_count, 1)
            self.assertEqual(call.call_args.args[1], 'pull')

    @patch.object(manager, 'check_account', return_value=None)
    @patch.object(manager, 'verify', return_value={})
    def test_cloud_push_records_backup_and_requires_matching_readback(self, _verify, _account):
        profile_id = '00000000-0000-4000-8000-000000000001'
        old = {'id': profile_id, 'name': self.base['name'], 'updated_time': 1, 'content': self.base | {'wall_loops': '1'}}
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
        self.assertEqual(result['status'], 'verified')
        self.assertEqual(manager.read(receipt)['before'], old)
        self.assertEqual(call.call_args_list[1].args[2]['original_updated_time'], '1')

    def cloud_fixture(self):
        self.account = 'test-account'
        self.profile_id = '00000000-0000-4000-8000-000000000001'
        self.old = {'id': self.profile_id, 'name': self.base['name'], 'updated_time': 1,
                    'content': self.base | {'wall_loops': '1'}}
        self.after = self.old | {'updated_time': 2, 'content': manager.cloud_wire(self.base)}
        self.receipt = self.root / 'operation.json'
        operation = manager.prepare_cloud(self.base, self.old, self.profile_id, self.account, ['orca'])
        manager.save_operation(self.receipt, operation, 'pending', 'prepared')
        from unittest.mock import Mock
        self.session = Mock(account=self.account)
        return self.session

    @patch.object(manager, 'verify', return_value={})
    def test_timeout_after_commit_reconciles_without_duplicate_upload(self, _verify):
        session = self.cloud_fixture()
        session.request.side_effect = [{'upserts': [self.old]}, manager.CloudError(), {'upserts': [self.after]}]
        result = manager.advance_cloud(self.receipt, session, polls=1, interval=0)
        self.assertEqual(result['status'], 'verified')
        self.assertEqual([c.args[0] for c in session.request.call_args_list], ['pull', 'push', 'pull'])

    @patch.object(manager, 'verify', return_value={})
    def test_delayed_readback_survives_restart_and_resume_does_not_repost(self, _verify):
        session = self.cloud_fixture()
        session.request.side_effect = [{'upserts': [self.old]}, manager.CloudError(), {'upserts': [self.old]}]
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'pending')
        session.request.reset_mock(side_effect=True)
        session.request.return_value = {'upserts': [self.old]}
        self.assertEqual(manager.advance_cloud(self.receipt, session, 2, 0)['status'], 'pending')
        self.assertTrue(all(c.args[0] == 'pull' for c in session.request.call_args_list))
        session.request.return_value = {'upserts': [self.after]}
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'verified')
        self.assertEqual(manager.read(self.receipt)['attempts'], 1)

    @patch.object(manager, 'verify', return_value={})
    def test_retries_keep_original_identity_revision_and_are_bounded(self, _verify):
        session = self.cloud_fixture()
        def request(action, body=None):
            if action == 'push':
                self.assertEqual(body['id'], self.profile_id)
                self.assertEqual(body['original_updated_time'], '1')
                raise manager.CloudError(503)
            return {'upserts': [self.old]}
        session.request.side_effect = request
        for _ in range(5):
            result = manager.advance_cloud(self.receipt, session, 1, 0, retry=True)
        self.assertEqual(result['status'], 'pending')
        self.assertEqual(result['attempts'], 3)

    @patch.object(manager, 'verify', return_value={})
    def test_same_timestamp_different_content_is_a_conflict(self, _verify):
        session = self.cloud_fixture()
        session.request.return_value = {'upserts': [self.old | {'content': self.base | {'wall_loops': '9'}}]}
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'conflicted')
        session.request.assert_called_once_with('pull')

    @patch.object(manager, 'verify', return_value={})
    def test_deleted_remote_or_name_collision_never_uploads(self, _verify):
        for response in ({'upserts': [], 'deletes': ['00000000-0000-4000-8000-000000000001']},
                         {'upserts': [{'id': 'another', 'name': self.base['name']}]}):
            session = self.cloud_fixture()
            session.request.return_value = response
            self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'conflicted')
            session.request.assert_called_once_with('pull')

    @patch.object(manager, 'verify', return_value={})
    def test_auth_failure_is_durable_and_can_resume_with_new_credentials(self, _verify):
        session = self.cloud_fixture()
        session.request.side_effect = manager.CloudError(401)
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'failed')
        session.request.side_effect = None
        session.request.return_value = {'upserts': [self.after]}
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0, retry=True)['status'], 'verified')

    @patch.object(manager, 'check_account', return_value=None)
    def test_refresh_hook_runs_once_without_shell_and_retries_once(self, _account):
        import hashlib
        from types import SimpleNamespace
        token = self.root / 'token'
        token.write_text('old-token')
        session = manager.CloudSession(token, 'account', ['credential-provider', '--refresh'])
        def refresh(argv, **kwargs):
            self.assertEqual(argv, ['credential-provider', '--refresh'])
            self.assertEqual(kwargs['env']['ORCA_PROFILE_TOKEN_FILE'], str(token))
            self.assertTrue(kwargs['capture_output'])
            token.write_text('new-token')
            return SimpleNamespace(returncode=0)
        with patch.object(manager.subprocess, 'run', side_effect=refresh) as hook, patch.object(
                manager, 'cloud', side_effect=[manager.CloudError(401, hashlib.sha256(b'old-token').hexdigest()),
                                                {'upserts': []}]) as transport:
            self.assertEqual(session.request('pull'), {'upserts': []})
        self.assertEqual(hook.call_count, 1)
        self.assertEqual(transport.call_count, 2)

    @patch.object(manager, 'check_account', return_value=None)
    def test_refresh_failure_does_not_leak_output_or_loop(self, _account):
        import hashlib
        from types import SimpleNamespace
        token = self.root / 'token'
        token.write_text('secret')
        session = manager.CloudSession(token, 'account', ['provider'])
        with patch.object(manager.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stdout='secret')), patch.object(
                manager, 'cloud', side_effect=manager.CloudError(401, hashlib.sha256(b'secret').hexdigest())) as transport, self.assertRaises(manager.CloudError) as error:
            session.request('pull')
        self.assertNotIn('secret', str(error.exception))
        self.assertEqual(transport.call_count, 1)

    def test_three_way_merges_disjoint_edits_deletions_and_reports_conflicts(self):
        base = {'a': '1', 'b': '2', 'c': '3'}
        self.assertEqual(manager.three_way(base, {'a': '4', 'c': '3'}, base | {'c': '5'})['merged'],
                         {'a': '4', 'c': '5'})
        result = manager.three_way(base, base | {'a': '4'}, base | {'a': '5'})
        self.assertIsNone(result['merged'])
        self.assertEqual(result['conflicts']['a']['base']['value'], '1')

    @patch.object(manager, 'verify', return_value={})
    def test_active_gui_directory_is_blocked_before_writes(self, _verify):
        for library in (self.root / 'OrcaSlicer', self.root / 'custom-data'):
            library.mkdir()
            (library / 'OrcaSlicer.conf').write_text('{}')
            plan = manager.make_plan(manager.family_read(self.family_file), library)
            with self.assertRaisesRegex(ValueError, 'GUI data directory'):
                manager.apply_plan(plan, ['orca'])
            self.assertFalse((library / 'process').exists())

    @patch.object(manager, 'verify', return_value={})
    def test_interrupted_local_apply_can_finish_but_preserves_later_edits(self, _verify):
        library = self.root / 'library'
        plan = manager.make_plan(manager.family_read(self.family_file), library)
        original_write = manager.write
        def interrupt(path, value):
            if Path(path).name == 'Normal 0.6.json':
                raise KeyboardInterrupt()
            original_write(path, value)
        with patch.object(manager, 'write', side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
            manager.apply_plan(plan, ['orca'])
        receipt = next((library / '.agent-profile-history').glob('*.json'))
        with self.assertRaisesRegex(ValueError, 'requires recovery'):
            manager.apply_plan(plan, ['orca'])
        target = library / plan['entries'][0]['path']
        original_write(target, self.base | {'wall_loops': '9'})
        with self.assertRaisesRegex(ValueError, 'later local edit'):
            manager.recover_local(receipt, ['orca'], 'finish')
        self.assertEqual(manager.read(target)['wall_loops'], '9')
        original_write(target, plan['entries'][0]['after'])
        manager.recover_local(receipt, ['orca'], 'finish')
        self.assertEqual(manager.read(receipt)['status'], 'recovered')
        self.assertTrue((library / plan['entries'][1]['path']).exists())

    def sync_fixture(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        self.local = self.root / 'library/process/base/Normal 0.4.json'
        manager.write(self.local, self.base)
        self.binding = self.root / 'binding.json'
        self.profile_id = '00000000-0000-4000-8000-000000000001'
        self.remote = {'id': self.profile_id, 'name': self.base['name'], 'updated_time': 1, 'content': self.base}
        snapshot = self.root / 'snapshot.json'
        manager.write(snapshot, {'account_id': 'account', 'upserts': [self.remote]})
        manager.bind_sync(SimpleNamespace(snapshot=snapshot, account='account', id=self.profile_id,
                                          profile=self.local, out=str(self.binding)))
        manager.write(self.local, self.base | {'wall_loops': '4'})
        self.remote = self.remote | {'updated_time': 2, 'content': self.base | {'layer_height': '0.24'}}
        self.session = Mock(account='account')
        def transport(action, body=None):
            if action == 'push':
                self.remote = self.remote | {'updated_time': 3, 'content': body['content']}
                return {'updated_time': 3}
            return {'upserts': [self.remote]}
        self.session.request.side_effect = transport
        plan = manager.sync_plan(self.binding, self.session)
        self.plan_file = self.root / 'sync-plan.json'
        manager.write(self.plan_file, plan)
        self.sync_args = SimpleNamespace(path=self.plan_file, out=str(self.root / 'sync-operation.json'),
                                         orca_command=['orca'], polls=1, interval=0)

    @patch.object(manager, 'verify', return_value={})
    def test_sync_merges_local_and_cloud_and_advances_baseline(self, _verify):
        self.sync_fixture()
        result = manager.start_sync(self.sync_args, self.session)
        self.assertEqual(result['status'], 'verified')
        actual = manager.read(self.local)
        self.assertEqual(actual['wall_loops'], '4')
        self.assertEqual(actual['layer_height'], '0.24')
        self.assertEqual(manager.read(self.binding)['baseline'], actual)
        self.assertEqual(result['gui_loaded'], 'unknown')

    @patch.object(manager, 'verify', return_value={})
    def test_sync_recovers_crashes_at_each_local_commit_boundary(self, _verify):
        for stage in ('local', 'binding', 'receipt'):
            with self.subTest(stage=stage):
                # Independent workspace for each simulated process interruption.
                self.root = Path(self.temp.name) / stage
                self.sync_fixture()
                original_write = manager.write
                def interrupt(path, value, stage=stage, original_write=original_write):
                    crash = ((stage == 'local' and Path(path) == self.local) or
                             (stage == 'binding' and Path(path) == self.binding) or
                             (stage == 'receipt' and str(path) == self.sync_args.out and value.get('status') == 'verified'))
                    if crash:
                        raise KeyboardInterrupt()
                    original_write(path, value)
                with patch.object(manager, 'write', side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
                    manager.start_sync(self.sync_args, self.session)
                with self.assertRaisesRegex(ValueError, 'Pending sync'):
                    manager.sync_plan(self.binding, self.session)
                result = manager.advance_sync(self.sync_args.out, self.session, ['orca'], 1, 0)
                self.assertEqual(result['status'], 'verified')
                self.assertEqual(sum(c.args[0] == 'push' for c in self.session.request.call_args_list), 1)
                self.assertEqual(manager.read(self.binding)['baseline'], manager.read(self.local))

    def test_os_lock_releases_after_process_exit(self):
        import subprocess
        import sys
        lock = self.root / 'process.lock'
        script = ("import importlib.util,os; "
                  f"s=importlib.util.spec_from_file_location('m', {str(SPEC.origin)!r}); "
                  "m=importlib.util.module_from_spec(s); s.loader.exec_module(m); "
                  f"lock=m.operation_lock({str(lock)!r}); lock.__enter__(); "
                  "print('locked',flush=True); input(); os._exit(7)")
        child = subprocess.Popen([sys.executable, '-c', script], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'locked')
            with self.assertRaisesRegex(ValueError, 'locked') , manager.operation_lock(lock):
                self.fail('Concurrent manager acquired the lock')
            child.communicate('exit\n', timeout=10)
            self.assertEqual(child.returncode, 7)
            with manager.operation_lock(lock):
                pass
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()
            child.stdout.close()
            child.stdin.close()

    @patch.object(manager, 'verify', return_value={})
    def test_sync_conflict_keeps_reservation_until_uncertain_upload_is_resolved(self, _verify):
        self.sync_fixture()
        original_transport = self.session.request.side_effect
        def uncertain(action, body=None):
            if action == 'push':
                raise manager.CloudError()
            return original_transport(action, body)
        self.session.request.side_effect = uncertain
        self.assertEqual(manager.start_sync(self.sync_args, self.session)['status'], 'pending')
        manager.write(self.local, self.base | {'wall_loops': '9'})
        self.assertEqual(manager.advance_sync(self.sync_args.out, self.session, ['orca'], 1, 0)['status'], 'pending')
        with self.assertRaisesRegex(ValueError, 'Pending sync'):
            manager.sync_plan(self.binding, self.session)
        child = manager.read(self.sync_args.out + '.cloud.json')
        self.remote = self.remote | {'updated_time': 3, 'content': child['request']['content']}
        self.assertEqual(manager.advance_sync(self.sync_args.out, self.session, ['orca'], 1, 0)['status'], 'conflicted')
        self.assertEqual(manager.read(self.local)['wall_loops'], '9')

    @patch.object(manager, 'verify', return_value={})
    def test_sync_cli_plan_apply_status_and_resume(self, _verify):
        self.sync_fixture()
        common = ['--account', 'account', '--token-file', 'unused']
        plan = str(self.root / 'cli-plan.json')
        receipt = str(self.root / 'cli-operation.json')
        with patch.object(manager, 'CloudSession', return_value=self.session):
            self.assertEqual(manager.main(['sync', 'plan', str(self.binding), '--out', plan] + common)['status'], 'ready')
            self.assertEqual(manager.main(['sync', 'apply', plan, '--out', receipt, '--orca-command', '["orca"]',
                                           '--polls', '1', '--interval', '0'] + common)['status'], 'verified')
            self.assertEqual(manager.main(['sync', 'status', receipt])['status'], 'verified')
            self.assertEqual(manager.main(['sync', 'resume', receipt, '--orca-command', '["orca"]',
                                           '--polls', '1', '--interval', '0'] + common)['status'], 'verified')

    @patch.object(manager, 'verify', return_value={})
    def test_lost_credentials_do_not_release_an_uncertain_sync(self, _verify):
        session = self.cloud_fixture()
        session.request.side_effect = [{'upserts': [self.old]}, manager.CloudError(), manager.CloudError(401)]
        self.assertEqual(manager.advance_cloud(self.receipt, session, 1, 0)['status'], 'pending')
        session.request.side_effect = manager.CloudError(401)
        result = manager.advance_cloud(self.receipt, session, 1, 0)
        self.assertEqual(result['status'], 'pending')
        self.assertEqual(result['reason'], 'authentication_required')

    @patch.object(manager, 'verify', return_value={})
    def test_sync_rejects_stale_cloud_and_envelope_rename(self, _verify):
        self.sync_fixture()
        self.remote = self.remote | {'updated_time': 0}
        self.assertEqual(manager.sync_plan(self.binding, self.session)['status'], 'pending')
        self.remote = self.remote | {'updated_time': 2, 'name': 'Renamed elsewhere'}
        self.assertEqual(manager.sync_plan(self.binding, self.session)['status'], 'conflicted')

    @patch.object(manager, 'verify', return_value={})
    def test_interrupted_recovery_can_itself_be_resumed(self, _verify):
        library = self.root / 'library'
        plan = manager.make_plan(manager.family_read(self.family_file), library)
        original_write = manager.write
        def interrupt(path, value):
            if Path(path).name == 'Normal 0.6.json':
                raise KeyboardInterrupt()
            original_write(path, value)
        with patch.object(manager, 'write', side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
            manager.apply_plan(plan, ['orca'])
        receipt = next((library / '.agent-profile-history').glob('*.json'))
        with patch.object(manager, 'write', side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
            manager.recover_local(receipt, ['orca'], 'finish')
        child = next(p for p in (library / '.agent-profile-history').glob('*.json') if p != receipt)
        manager.recover_local(child, ['orca'], 'finish')
        self.assertEqual(manager.read(receipt)['status'], 'recovered')
        self.assertEqual(manager.read(child)['status'], 'recovered')
        for entry in plan['entries']:
            self.assertEqual(manager.read(library / entry['path']), entry['after'])

    @unittest.skipUnless(os.environ.get('ORCA_PROFILE_COMMAND'), 'Real Orca command not configured')
    def test_real_orca_preserves_both_variants_from_local_and_cloud_formats(self):
        command = json.loads(os.environ['ORCA_PROFILE_COMMAND'])
        for config in (self.base, manager.cloud_wire(self.base)):
            self.assertEqual(manager.verify(config, command)['status'], 'orca_readback_verified')
        with self.assertRaisesRegex(ValueError, 'changed or dropped'):
            manager.verify(self.base | {'made_up_setting': '5'}, command)


if __name__ == '__main__':
    unittest.main()
