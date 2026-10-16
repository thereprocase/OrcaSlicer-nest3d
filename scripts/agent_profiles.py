#!/usr/bin/env python3
"""Manage nozzle families and publish standalone Orca user presets. JSON on stdout."""

import argparse
import base64
import copy
import csv
import hashlib
import http.client
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import uuid
from contextlib import contextmanager
from pathlib import Path

TYPES = ('process', 'filament', 'machine')
META = {'name', 'type', 'from', 'version', 'inherits', 'setting_id', 'base_id',
        'updated_time', 'user_id', 'sync_info', 'instantiation', 'filament_id', 'description'}
IDENTITY = {'process': 'print_settings_id', 'filament': 'filament_settings_id',
            'machine': 'printer_settings_id'}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix='.profile-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(encoded(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
        if os.name != 'nt':
            directory = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


@contextmanager
def operation_lock(path):
    """Persistent lock inode; the OS releases ownership even after a crash."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_dir() or path.is_symlink():
        raise ValueError('Operation is locked or the lock path is invalid')
    with path.open('a+b') as stream:
        try:
            if os.name == 'nt':
                import msvcrt
                if stream.tell() == 0:
                    stream.write(b'\0')
                    stream.flush()
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            raise ValueError('Operation is locked by another manager process') from None
        try:
            yield
        finally:
            if os.name == 'nt':
                stream.seek(0)
                msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(stream, fcntl.LOCK_UN)


def require_managed_path(path):
    """Block GUI data stores: an on-disk lock cannot protect unsaved GUI state."""
    path = Path(path).absolute()
    if path != path.resolve():
        raise ValueError('Managed paths must not contain symlinks')
    for parent in (path, *path.parents):
        if (parent.name.lower() == 'orcaslicer' or
                (parent / 'OrcaSlicer.conf').exists() or
                (parent / 'OrcaSlicer.ini').exists()):
            raise ValueError('Direct writes to an Orca GUI data directory are unsupported; use a managed workspace and cloud sync')
    return path


def nozzle(value):
    number = float(value)
    if not 0 < number <= 10:
        raise ValueError('Nozzle diameter must be in (0, 10] mm')
    return format(number, '.6g')


def check_root(config):
    if not isinstance(config, dict) or config.get('type') not in TYPES:
        raise ValueError('Expected a process, filament or machine preset')
    if not isinstance(config.get('name'), str) or not config['name'] or config.get('inherits'):
        raise ValueError('Input must be a named standalone preset: inherits must be empty')
    if config.get('include'):
        raise ValueError('Resolve include files before importing a preset')
    if config.get('base_id'):
        raise ValueError('A standalone preset cannot have a base_id')
    return config


def settings(config):
    return {k: v for k, v in config.items() if k not in META and k not in IDENTITY.values()}


def differences(left, right):
    return {k: {'before': left.get(k), 'after': right.get(k),
                'before_present': k in left, 'after_present': k in right}
            for k in sorted(left.keys() | right.keys())
            if k not in left or k not in right or left[k] != right[k]}


def family_read(path):
    family = read(path)
    if family.get('schema') != 1 or family.get('type') not in TYPES:
        raise ValueError('Unsupported family schema or preset type')
    uuid.UUID(family['id'])
    if family['base_nozzle'] not in family['variants']:
        raise ValueError('Base nozzle must have a variant')
    for diameter in family['variants']:
        if nozzle(diameter) != diameter:
            raise ValueError('Noncanonical nozzle diameter')
    names = [v['name'] for v in family['variants'].values()]
    if len(set(names)) != len(names):
        raise ValueError('Each nozzle variant must have a distinct preset name')
    return family


def edit_family(args):
    require_managed_path(args.path)
    with operation_lock(str(args.path) + '.lock'):
        family = family_read(args.path)
        if digest(family) != args.expect:
            raise ValueError('Family revision changed; inspect and retry')
        if args.key in META or args.key in IDENTITY.values():
            raise ValueError('Identity and relationship fields cannot be setting overrides')
        target = family['base'] if args.nozzle is None else family['variants'][args.nozzle]['overrides']
        if args.action == 'set':
            target[args.key] = args.value
        else:
            if args.nozzle is None:
                raise ValueError('Reset requires --nozzle; it restores inheritance from the family base')
            target.pop(args.key, None)
        write(args.path, family)
        return family


def build(family):
    result = []
    for diameter, variant in sorted(family['variants'].items(), key=lambda x: float(x[0])):
        name = variant['name']
        config = copy.deepcopy(family['base'])
        config.update(copy.deepcopy(variant['overrides']))
        config.update(type=family['type'], name=name, inherits='', base_id='',
                      version=family['version'], instantiation='true')
        config['from'] = 'User'
        identity = IDENTITY[family['type']]
        config[identity] = [name] if family['type'] == 'filament' else name
        if family.get('filament_id'):
            config['filament_id'] = family['filament_id']
        if family['type'] == 'machine':
            config['nozzle_diameter'] = [diameter]
        check_root(config)
        result.append({'id': variant['id'], 'nozzle': diameter, 'config': config})
    return result


def canonical_value(value):
    # Orca cloud uses serialized strings while JSON presets use arrays for vectors.
    if isinstance(value, list):
        return [canonical_value(v) for v in value]
    if isinstance(value, bool):
        return '1' if value else '0'
    text = str(value)
    try:
        return float(text)
    except ValueError:
        return text


def matches(expected, actual):
    if isinstance(actual, list) and not isinstance(expected, list):
        text = str(expected)
        if ';' in text or text.startswith('"'):
            expected = []
            decoder = json.JSONDecoder(strict=False)
            while text:
                if text.startswith('"'):
                    value, end = decoder.raw_decode(text)
                else:
                    value = text.partition(';')[0]
                    end = len(value)
                expected.append(value)
                text = text[end:]
                if text:
                    if not text.startswith(';'):
                        return False
                    text = text[1:]
        else:
            expected = ([text] if len(actual) == 1 else
                        next(csv.reader([text], delimiter=','))) if text else []
    return canonical_value(expected) == canonical_value(actual)


def verify(config, command):
    check_root(config)
    if not isinstance(command, list) or not command or not all(isinstance(v, str) for v in command):
        raise ValueError('--orca-command must be a JSON argv array')
    with tempfile.TemporaryDirectory(prefix='orca-profile-') as directory:
        root = Path(directory)
        source, output = root / 'preset.json', root / 'effective.json'
        write(source, config)
        flag = '--load-filaments' if config['type'] == 'filament' else '--load-settings'
        run = subprocess.run(command + ['--datadir', str(root / 'data'), flag, str(source),
                             '--export-settings', str(output)], capture_output=True,
                             timeout=120, check=False, cwd=root)
        if run.returncode or not output.exists():
            raise ValueError(f'Orca rejected preset (exit {run.returncode}); no library changes made')
        effective = read(output)
        # Compatibility and identity are profile metadata absent from project export.
        omitted = {'compatible_printers', 'compatible_printers_condition',
                   'compatible_prints', 'compatible_prints_condition'}
        failed = [k for k, v in settings(config).items()
                  if k not in omitted and (k not in effective or not matches(v, effective[k]))]
        if failed:
            raise ValueError('Orca readback changed or dropped settings: ' + ', '.join(sorted(failed)))
        return {'status': 'orca_readback_verified', 'settings_checked': len(settings(config)) -
                len(omitted & config.keys()), 'effective_sha256': digest(effective)}


class CloudError(ValueError):
    def __init__(self, status=None, token_revision=None):
        self.status = status
        self.token_revision = token_revision
        super().__init__(f'Cloud HTTP {status}' if status else 'Cloud transport outcome is uncertain')


def cloud(token_file, action, body=None):
    token = Path(token_file).read_text().strip()
    if not token or '\n' in token:
        raise ValueError('Expected an access token file')
    request = urllib.request.Request('https://api.orcaslicer.com/api/v1/sync/' + action,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json',
                 'Accept': 'application/json', 'User-Agent': 'OrcaSlicer-AgentProfiles/1'})
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        # Never echo responses or request headers containing account data.
        raise CloudError(error.code, hashlib.sha256(token.encode()).hexdigest()) from None
    except (urllib.error.URLError, OSError, http.client.HTTPException, json.JSONDecodeError):
        raise CloudError() from None


def check_account(token_file, account):
    # This is a targeting check; the cloud service verifies the token's signature.
    try:
        token = Path(token_file).read_text().strip()
        payload = token.split('.')[1]
        subject = json.loads(base64.urlsafe_b64decode(payload + '=' * (-len(payload) % 4)))['sub']
    except (ValueError, IndexError, KeyError):
        raise ValueError('Access token has no readable account identity') from None
    if subject != account:
        raise ValueError('Access token belongs to a different account')


class CloudSession:
    def __init__(self, token_file, account, refresh_command=None):
        self.token_file = token_file
        self.account = account
        self.refresh_command = refresh_command
        if refresh_command is not None and (not isinstance(refresh_command, list) or
                not refresh_command or not all(isinstance(v, str) for v in refresh_command)):
            raise ValueError('Refresh command must be a nonempty JSON argv array')

    def request(self, action, body=None):
        check_account(self.token_file, self.account)
        try:
            return cloud(self.token_file, action, body) if body is not None else cloud(self.token_file, action)
        except CloudError as error:
            if error.status != 401 or self.refresh_command is None:
                raise
            with operation_lock(str(self.token_file) + '.refresh.lock'):
                current = Path(self.token_file).read_text().strip()
                # Another process may have refreshed while this request was in flight.
                if hashlib.sha256(current.encode()).hexdigest() == error.token_revision:
                    environment = os.environ | {'ORCA_PROFILE_TOKEN_FILE': str(Path(self.token_file).absolute()),
                                                 'ORCA_PROFILE_ACCOUNT': self.account}
                    try:
                        result = subprocess.run(self.refresh_command, env=environment,
                                                capture_output=True, timeout=45, check=False)
                    except (OSError, subprocess.SubprocessError):
                        raise CloudError(401) from None
                    if result.returncode:
                        raise CloudError(401)
                    if Path(self.token_file).read_text().strip() == current:
                        raise CloudError(401)
                check_account(self.token_file, self.account)
            # Only an explicit authentication rejection is replayed here, once.
            return cloud(self.token_file, action, body) if body is not None else cloud(self.token_file, action)


def cloud_content(item):
    value = item['content']
    return json.loads(value) if isinstance(value, str) else value


def cloud_wire(config):
    output = {}
    for key, value in config.items():
        if key in {'setting_id', 'user_id', 'updated_time', 'sync_info'}:
            continue  # Account identity and revisions come from the server envelope.
        if isinstance(value, list):
            value = ['1' if v is True else '0' if v is False else v for v in value]
            numeric = all(re.fullmatch(r'(?:nil|[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?%?)', str(v))
                          for v in value)
            if numeric:
                output[key] = ','.join(str(v) for v in value)
            else:
                # ConfigOptionStrings uses semicolons and C-style quoted strings.
                output[key] = ';'.join('"' + str(v).replace('\\', '\\\\').replace('"', '\\"')
                                      .replace('\r', '\\r').replace('\n', '\\n') + '"' for v in value)
        elif isinstance(value, bool):
            output[key] = '1' if value else '0'
        else:
            output[key] = str(value)
    return output


def remote_version(item):
    if item is None:
        return None
    return {'id': item['id'], 'name': item['name'], 'updated_time': item['updated_time'],
            'content': cloud_wire(cloud_content(item))}


def save_operation(path, operation, status, reason=None):
    operation['status'] = status
    operation['reason'] = reason
    operation['changed_at'] = time.time()
    write(path, operation)


def operation_result(path, operation):
    return {'status': operation['status'], 'reason': operation.get('reason'),
            'receipt': str(path), 'id': operation['request']['id'],
            'attempts': operation['attempts'], 'gui_loaded': 'unknown'}


def cloud_observation(operation, response):
    body = operation['request']
    items = response['upserts']
    remote = next((x for x in items if x['id'] == body['id']), None)
    if any(x['id'] != body['id'] and x['name'] == body['name'] for x in items):
        return 'conflicted', remote, 'name_collision'
    if body['id'] in response.get('deletes', []):
        return 'conflicted', remote, 'remote_deleted'
    if remote and remote['name'] == body['name'] and cloud_wire(cloud_content(remote)) == body['content']:
        return 'verified', remote, None
    before = operation['before']
    if remote_version(remote) == remote_version(before):
        return 'baseline', remote, None
    if remote is None or (before and int(remote['updated_time']) < int(before['updated_time'])):
        return 'pending', remote, 'readback_not_current'
    return 'conflicted', remote, 'remote_changed'


def advance_cloud(path, session, polls=3, interval=1.0, retry=False):
    """Reconcile first; an uncertain request is only replayed with explicit retry."""
    if not 1 <= polls <= 20 or not 0 <= interval <= 10:
        raise ValueError('Polling requires 1..20 reads and 0..10 seconds between reads')
    path = Path(path)
    with operation_lock(str(path) + '.lock'):
        operation = read(path)
        if operation.get('schema') != 2 or operation.get('kind') != 'cloud_push':
            raise ValueError('Unsupported cloud operation journal')
        if operation['account_id'] != session.account:
            raise ValueError('Cloud operation belongs to a different account')
        intent = {'before': operation['before'], 'request': operation['request']}
        if digest(intent) != operation['intent_sha256']:
            raise ValueError('Cloud operation intent changed; create a new operation')
        if retry and operation['status'] == 'failed' and operation.get('reason') in (
                'authentication_required', 'upload_rejected_401', 'upload_rejected_403'):
            save_operation(path, operation, 'pending', 'credentials_retry')
        if operation['status'] in ('conflicted', 'failed'):
            return operation_result(path, operation)
        attempted_here = False
        for poll in range(polls):
            if poll:
                time.sleep(interval)
            try:
                response = session.request('pull')
            except CloudError as error:
                auth = error.status in (401, 403)
                # Lost credentials cannot establish whether a prior request committed.
                status = 'failed' if auth and operation['attempts'] == 0 else 'pending'
                save_operation(path, operation, status, 'authentication_required' if auth else 'cloud_unavailable')
                if auth:
                    break
                continue
            status, remote, reason = cloud_observation(operation, response)
            operation['observed'] = remote
            if status == 'verified':
                operation['after'] = remote
                save_operation(path, operation, 'verified')
                break
            if status == 'conflicted':
                save_operation(path, operation, 'conflicted', reason)
                break
            may_submit = operation['attempts'] == 0 or retry
            if status == 'baseline' and may_submit and not attempted_here and operation['attempts'] < 3:
                operation['attempts'] += 1
                operation['phase'] = 'submitting'
                save_operation(path, operation, 'pending', 'upload_in_flight')
                attempted_here = True
                try:
                    acknowledgement = session.request('push', operation['request'])
                    operation['acknowledged_revision'] = acknowledgement.get('updated_time')
                    operation['phase'] = 'readback'
                    save_operation(path, operation, 'pending', 'awaiting_readback')
                except CloudError as error:
                    if error.status in (400, 401, 403, 404, 413, 422):
                        save_operation(path, operation, 'failed', f'upload_rejected_{error.status}')
                        break
                    # 409 can be our previous successful request, or another writer.
                    # A timeout/5xx can also follow a successful commit. Read before retry.
                    operation['phase'] = 'readback'
                    save_operation(path, operation, 'pending', 'upload_outcome_uncertain')
                # Always perform at least one read following the upload, even polls=1.
                try:
                    response = session.request('pull')
                    state, remote, reason = cloud_observation(operation, response)
                    operation['observed'] = remote
                    if state == 'verified':
                        operation['after'] = remote
                        save_operation(path, operation, 'verified')
                        break
                    if state == 'conflicted':
                        save_operation(path, operation, 'conflicted', reason)
                        break
                except CloudError:
                    pass
            save_operation(path, operation, 'pending', reason or 'awaiting_readback')
        return operation_result(path, operation)


def start_cloud(args, session):
    require_managed_path(args.out)
    config = check_root(read(args.profile))
    snapshot = read(args.snapshot)
    if snapshot.get('account_id') != args.account:
        raise ValueError('Cloud snapshot belongs to a different account')
    old = next((x for x in snapshot['upserts'] if x['id'] == args.id), None)
    operation = prepare_cloud(config, old, args.id, args.account, args.orca_command)
    with operation_lock(str(args.out) + '.lock'):
        if Path(args.out).exists():
            raise ValueError('Operation destination already exists; use cloud resume')
        save_operation(args.out, operation, 'pending', 'prepared')
    return advance_cloud(args.out, session, args.polls, args.interval)


def prepare_cloud(config, old, profile_id, account, command):
    validation = verify(config, command)
    body = {'id': str(uuid.UUID(profile_id)), 'name': config['name'], 'content': cloud_wire(config)}
    verify(body['content'], command)
    if old and config['type'] != cloud_content(old).get('type'):
        raise ValueError('Cannot change the preset type of a cloud ID')
    if old:
        body['original_updated_time'] = str(old['updated_time'])
    return {'schema': 2, 'kind': 'cloud_push', 'account_id': account,
                 'before': old, 'request': body, 'attempts': 0, 'phase': 'prepared',
                 'validation': validation, 'intent_sha256': digest({'before': old, 'request': body})}


def three_way(base, local, remote):
    """Merge independent edits, including deletions; conflicts never get a winner."""
    base, local, remote = cloud_wire(base), cloud_wire(local), cloud_wire(remote)
    missing = object()
    merged, conflicts = {}, {}
    for key in sorted(base.keys() | local.keys() | remote.keys()):
        b, l, r = base.get(key, missing), local.get(key, missing), remote.get(key, missing)
        if l == r or r == b:
            chosen = l
        elif l == b:
            chosen = r
        else:
            conflicts[key] = {label: {'present': value is not missing,
                                      'value': None if value is missing else value}
                              for label, value in [('base', b), ('local', l), ('remote', r)]}
            continue
        if chosen is not missing:
            merged[key] = chosen
    return {'merged': merged if not conflicts else None, 'conflicts': conflicts}


def binding_library(binding):
    local = require_managed_path(binding['local_path'])
    if local.parent.name != 'base' or local.parent.parent.name not in TYPES:
        raise ValueError('Sync requires a managed TYPE/base/NAME.json profile path')
    library = local.parents[2]
    safe_target(library, local.relative_to(library).as_posix())
    return library


def bind_sync(args):
    require_managed_path(args.out)
    snapshot = read(args.snapshot)
    if snapshot.get('account_id') != args.account:
        raise ValueError('Cloud snapshot belongs to a different account')
    remote = next((x for x in snapshot['upserts'] if x['id'] == args.id), None)
    if remote is None:
        raise ValueError('Cloud profile is absent; publish it before binding')
    binding = {'schema': 1, 'kind': 'sync_binding', 'account_id': args.account,
               'profile_id': args.id, 'local_path': str(Path(args.profile).absolute())}
    library = binding_library(binding)
    with operation_lock(str(args.out) + '.lock'), operation_lock(library / '.agent-profiles.lock'):
        check_pending_sync(library)
        if Path(args.out).exists():
            raise ValueError('Binding already exists')
        local = check_root(read(args.profile))
        if Path(args.profile).stem != local['name'] or Path(args.profile).parent.parent.name != local['type']:
            raise ValueError('Profile filename/type must match its identity')
        if cloud_wire(local) != cloud_wire(cloud_content(remote)):
            raise ValueError('Initial local/cloud versions differ; reconcile explicitly before binding')
        binding.update(baseline=cloud_wire(local), remote=remote_version(remote))
        write(args.out, binding)
    return {'status': 'bound', 'binding': args.out, 'gui_loaded': 'unknown'}


def sync_plan(binding_path, session):
    with operation_lock(str(binding_path) + '.lock'):
        binding = read(binding_path)
        if binding.get('kind') != 'sync_binding' or binding.get('schema') != 1:
            raise ValueError('Unsupported sync binding')
        if binding['account_id'] != session.account:
            raise ValueError('Binding belongs to another account')
        library = binding_library(binding)
        with operation_lock(library / '.agent-profiles.lock'):
            check_pending_sync(library)
            local = check_root(read(binding['local_path']))
        response = session.request('pull')
        remote = next((x for x in response['upserts'] if x['id'] == binding['profile_id']), None)
        plan = {'schema': 1, 'kind': 'sync_plan', 'binding_path': str(Path(binding_path).absolute()),
                'binding_revision': digest(binding), 'local_before': local,
                'remote_before': remote, 'account_id': session.account, 'binding': binding}
        if remote is None or binding['profile_id'] in response.get('deletes', []):
            return plan | {'status': 'conflicted', 'reason': 'remote_missing_or_deleted', 'merged': None}
        if int(remote['updated_time']) < int(binding['remote']['updated_time']):
            return plan | {'status': 'pending', 'reason': 'stale_cloud_read', 'merged': None}
        merge = three_way(binding['baseline'], local, cloud_content(remote))
        if cloud_content(remote).get('type') != binding['baseline']['type']:
            merge = {'merged': None, 'conflicts': {'type': 'remote preset type changed'}}
        if remote['name'] != binding['baseline']['name'] or (merge['merged'] and any(
                merge['merged'].get(key) != binding['baseline'].get(key) for key in ('name', 'type'))):
            merge = {'merged': None, 'conflicts': {'identity': 'Rename/type migration requires explicit rebind'}}
        status = 'conflicted' if merge['conflicts'] else 'ready'
        return plan | merge | {'status': status, 'reason': 'divergent_edits' if merge['conflicts'] else None}


def start_sync(args, session):
    plan = read(args.path)
    if plan.get('kind') != 'sync_plan' or plan.get('status') != 'ready':
        raise ValueError('Sync requires a ready, conflict-free plan')
    if plan['account_id'] != session.account:
        raise ValueError('Plan belongs to another account')
    require_managed_path(args.out)
    operation = {'schema': 1, 'kind': 'sync_operation', 'plan': plan, 'status': 'pending',
                 'operation_id': str(uuid.uuid4()), 'plan_sha256': digest(plan),
                 'cloud_receipt': str(Path(str(args.out) + '.cloud.json').absolute())}
    with operation_lock(str(args.out) + '.lock'):
        if Path(args.out).exists() or Path(operation['cloud_receipt']).exists():
            raise ValueError('Sync operation destination exists; resume it')
        write(args.out, operation)
    return advance_sync(args.out, session, args.orca_command, args.polls, args.interval)


def check_pending_sync(library, own=None):
    marker = Path(library) / '.agent-sync-active.json'
    if marker.exists():
        previous = read(marker)['receipt']
        if ((own is None or Path(previous).resolve() != Path(own).resolve()) and
                (not Path(previous).exists() or read(previous).get('status') not in ('verified', 'conflicted', 'failed'))):
            raise ValueError('Pending sync requires resume: ' + previous)
    for prior in (Path(library) / '.agent-profile-history').glob('*.json'):
        if read(prior).get('status') in ('prepared', 'pending', 'conflicted'):
            raise ValueError('Unfinished local operation requires recovery: ' + str(prior))


def advance_sync(path, session, command, polls=3, interval=1.0, retry=False):
    with operation_lock(str(path) + '.lock'):
        operation = read(path)
        if operation.get('kind') != 'sync_operation' or operation.get('schema') != 1:
            raise ValueError('Unsupported sync operation')
        plan = operation['plan']
        if digest(plan) != operation['plan_sha256']:
            raise ValueError('Sync plan changed; create a new operation')
        if plan['account_id'] != session.account:
            raise ValueError('Sync operation belongs to another account')
        binding = plan['binding']
        library = binding_library(binding)
        with operation_lock(plan['binding_path'] + '.lock'), operation_lock(library / '.agent-profiles.lock'):
            check_pending_sync(library, path)
            write(library / '.agent-sync-active.json', {'receipt': str(Path(path).absolute())})
            actual_binding = read(plan['binding_path'])
            local = read(binding['local_path'])
            already_completed = (actual_binding.get('last_operation') == operation['operation_id'] and
                                 actual_binding.get('baseline') == plan['merged'])
            if (digest(actual_binding) != plan['binding_revision'] and not already_completed) or (
                    local != plan['local_before'] and cloud_wire(local) != plan['merged']):
                # An earlier upload may still be in flight. Resolve its outcome without
                # replay before releasing the workspace reservation for a new plan.
                state = 'conflicted'
                if Path(operation['cloud_receipt']).exists():
                    child = read(operation['cloud_receipt'])
                    if child.get('attempts', 0) and child.get('status') == 'pending':
                        observed = advance_cloud(operation['cloud_receipt'], session, polls, interval)
                        if observed['status'] == 'pending':
                            state = 'pending'
                operation.update(status=state, reason='local_or_baseline_changed')
                write(path, operation)
                return {'status': operation['status'], 'reason': operation['reason'], 'receipt': str(path), 'gui_loaded': 'unknown'}
            if not Path(operation['cloud_receipt']).exists():
                cloud_operation = prepare_cloud(plan['merged'], plan['remote_before'], binding['profile_id'],
                                                session.account, command)
                save_operation(operation['cloud_receipt'], cloud_operation, 'pending', 'prepared')
            result = advance_cloud(operation['cloud_receipt'], session, polls, interval, retry)
            operation.update(status=result['status'], reason=result['reason'])
            if result['status'] == 'verified':
                # Recheck the file even under our lock: an editor need not obey it.
                current = read(binding['local_path'])
                if current != plan['local_before'] and cloud_wire(current) != plan['merged']:
                    operation.update(status='conflicted', reason='local_changed_after_cloud_write')
                else:
                    verify(plan['merged'], command)
                    # Journal stays pending until both local data and baseline are durable.
                    write(binding['local_path'], plan['merged'])
                    remote = read(operation['cloud_receipt'])['after']
                    updated = binding | {'baseline': plan['merged'], 'remote': remote_version(remote),
                                         'last_operation': operation['operation_id']}
                    write(plan['binding_path'], updated)
            write(path, operation)
            return {'status': operation['status'], 'reason': operation.get('reason'),
                    'receipt': str(path), 'gui_loaded': 'unknown'}


def make_plan(family, library):
    library = Path(library).absolute()
    entries = []
    for variant in build(family):
        # Orca's directory loader takes preset identity from the filename.
        relative = f"{family['type']}/base/{variant['config']['name']}.json"
        target = safe_target(library, relative)
        before = read(target) if target.exists() else None
        entries.append({'path': relative, 'before': before, 'after': variant['config'],
                        'changes': differences(before or {}, variant['config'])})
    return {'schema': 1, 'library': str(library), 'family_id': family['id'], 'entries': entries}


def safe_target(library, relative):
    match = re.fullmatch(r'(process|filament|machine)/base/([^/\\]+)\.json', relative)
    if not match:
        raise ValueError('Invalid managed profile path')
    name = match[2]
    if (re.search(r'[<>:"|?*\x00-\x1f]', name) or name.rstrip(' .') != name or
            name in ('.', '..') or re.fullmatch(r'(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', name, re.IGNORECASE)):
        raise ValueError('Profile name is not a portable filename')
    target = Path(library) / relative
    if target.resolve() != target.absolute():
        raise ValueError('Symlinks are not allowed in the managed library path')
    return target


def apply_plan(plan, command, recovering=None):
    if plan.get('schema') != 1 or not plan.get('entries'):
        raise ValueError('Empty or unsupported plan')
    library = require_managed_path(plan['library'])
    library.mkdir(parents=True, exist_ok=True)
    if (library / '.agent-profiles-lock').exists():
        raise ValueError('Legacy library lock exists; inspect the old operation before removing it')
    changed = []
    recovery_chain = []
    ancestor = Path(recovering).resolve() if recovering else None
    while ancestor is not None:
        if ancestor in recovery_chain or ancestor.parent != library / '.agent-profile-history':
            raise ValueError('Invalid local recovery ancestry')
        recovery_chain.append(ancestor)
        parent = read(ancestor).get('recovery_of')
        ancestor = Path(parent).resolve() if parent else None
    with operation_lock(library / '.agent-profiles.lock'):
        marker = library / '.agent-sync-active.json'
        if marker.exists():
            prior = Path(read(marker)['receipt'])
            if not prior.exists() or read(prior).get('status') == 'pending':
                raise ValueError('Pending sync requires resume: ' + str(prior))
        history = require_managed_path(library / '.agent-profile-history')
        for prior in history.glob('*.json'):
            if prior.resolve() in recovery_chain:
                continue
            if read(prior).get('status') in ('prepared', 'pending', 'conflicted'):
                raise ValueError('Unfinished local operation requires recovery: ' + str(prior))
        paths = [e['path'] for e in plan['entries']]
        if len(paths) != len(set(paths)):
            raise ValueError('Duplicate paths in plan')
        for entry in plan['entries']:
            target = safe_target(library, entry['path'])
            current = read(target) if target.exists() else None
            if current != entry['before']:
                raise ValueError('Library changed since plan: ' + entry['path'])
            if entry['after'] is not None:
                verify(entry['after'], command)
        transaction = str(uuid.uuid4())
        receipt = history / (transaction + '.json')
        journal = plan | {'status': 'pending', 'kind': 'local_apply', 'transaction': transaction}
        if recovering:
            journal['recovery_of'] = str(Path(recovering).resolve())
        write(receipt, journal)
        try:
            for entry in plan['entries']:
                target = safe_target(library, entry['path'])
                if (read(target) if target.exists() else None) != entry['before']:
                    raise ValueError('File changed during apply: ' + entry['path'])
                changed.append(entry)
                if entry['after'] is None:
                    target.unlink(missing_ok=True)
                else:
                    write(target, entry['after'])
            write(receipt, journal | {'status': 'applied'})
        except Exception:
            conflict = False
            for entry in reversed(changed):
                target = safe_target(library, entry['path'])
                current = read(target) if target.exists() else None
                if current not in (entry['before'], entry['after']):
                    conflict = True
                    continue
                if entry['before'] is None:
                    target.unlink(missing_ok=True)
                else:
                    write(target, entry['before'])
            write(receipt, journal | {'status': 'conflicted' if conflict else 'rolled_back'})
            raise
        for previous_path in recovery_chain:
            previous = read(previous_path)
            write(previous_path, previous | {'status': 'recovered', 'recovery_receipt': str(receipt)})
        return {'status': 'local_applied', 'receipt': str(receipt), 'cloud_synced': False}


def recover_local(path, command, direction):
    journal = read(path)
    if journal.get('status') not in ('prepared', 'pending', 'conflicted'):
        raise ValueError('Local operation is not awaiting recovery')
    library = require_managed_path(journal['library'])
    entries = []
    for entry in journal['entries']:
        target = safe_target(library, entry['path'])
        current = read(target) if target.exists() else None
        if current not in (entry['before'], entry['after']):
            raise ValueError('Recovery conflicts with a later local edit: ' + entry['path'])
        entries.append(entry | {'before': current, 'after': entry['after' if direction == 'finish' else 'before']})
    plan = {'schema': 1, 'library': str(library), 'entries': entries}
    return apply_plan(plan, command, recovering=path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='group', required=True)
    profiles = sub.add_parser('profiles').add_subparsers(dest='action', required=True)
    for action in ('list', 'show', 'compare', 'validate', 'verify', 'plan', 'apply', 'rollback', 'recover'):
        p = profiles.add_parser(action)
        p.add_argument('path')
        if action == 'compare':
            p.add_argument('other')
        if action == 'plan':
            p.add_argument('--library', required=True)
            p.add_argument('--out', required=True)
        if action in ('validate', 'verify', 'apply', 'rollback', 'recover'):
            p.add_argument('--orca-command', required=True, type=json.loads)
        if action == 'recover':
            p.add_argument('--direction', choices=('finish', 'rollback'), required=True)
    families = sub.add_parser('family').add_subparsers(dest='action', required=True)
    p = families.add_parser('create')
    p.add_argument('--name', required=True)
    p.add_argument('--base', required=True)
    p.add_argument('--base-nozzle', required=True, type=nozzle)
    p.add_argument('--variant', action='append', default=[], metavar='DIAMETER=FILE')
    p.add_argument('--out', required=True)
    for action in ('show', 'set', 'reset', 'build'):
        p = families.add_parser(action)
        p.add_argument('path')
        if action in ('set', 'reset'):
            p.add_argument('--nozzle', type=nozzle)
            p.add_argument('--key', required=True)
            p.add_argument('--expect', required=True, help='Current family revision from family show')
        if action == 'set':
            p.add_argument('--value', required=True, type=json.loads)
        if action == 'build':
            p.add_argument('--out', required=True)
    for group, actions in [('cloud', ('pull', 'push', 'resume', 'status')),
                           ('sync', ('bind', 'plan', 'apply', 'resume', 'status'))]:
        commands = sub.add_parser(group).add_subparsers(dest='action', required=True)
        for action in actions:
            p = commands.add_parser(action)
            if action in ('resume', 'status') or (group == 'sync' and action in ('plan', 'apply')):
                p.add_argument('path')
            if action == 'status':
                continue
            p.add_argument('--account', required=True, help='Expected Orca account UUID')
            if action != 'bind':
                p.add_argument('--token-file', required=True)
                p.add_argument('--refresh-command', type=json.loads)
            if action not in ('resume',):
                p.add_argument('--out', required=True)
            if action in ('push', 'bind'):
                p.add_argument('--profile', required=True)
                p.add_argument('--snapshot', required=True)
                p.add_argument('--id', required=True)
            if action in ('push', 'apply') or (group == 'sync' and action == 'resume'):
                p.add_argument('--orca-command', required=True, type=json.loads)
            if action in ('push', 'apply', 'resume'):
                p.add_argument('--polls', type=int, default=3)
                p.add_argument('--interval', type=float, default=1)
            if action == 'resume':
                p.add_argument('--retry', action='store_true', help='Allow bounded replay after baseline readback')
    args = parser.parse_args(argv)
    if args.group in ('cloud', 'sync'):
        if args.action == 'status':
            journal = read(args.path)
            return {key: journal.get(key) for key in ('kind', 'status', 'reason', 'attempts', 'changed_at')} | {'gui_loaded': 'unknown'}
        if args.action == 'bind':
            return bind_sync(args)
        session = CloudSession(args.token_file, args.account, args.refresh_command)
        if args.group == 'sync':
            if args.action == 'plan':
                result = sync_plan(args.path, session)
                if Path(args.out).exists():
                    raise ValueError('Plan destination already exists')
                write(args.out, result)
                return result
            if args.action == 'apply':
                return start_sync(args, session)
            return advance_sync(args.path, session, args.orca_command, args.polls, args.interval, args.retry)
        if args.action == 'pull':
            result = session.request('pull')
            result['account_id'] = args.account
            write(args.out, result)
            return {'status': 'cloud_pulled', 'out': args.out, 'profiles': len(result['upserts'])}
        if args.action == 'push':
            return start_cloud(args, session)
        return advance_cloud(args.path, session, args.polls, args.interval, args.retry)
    if args.group == 'family':
        if args.action == 'create':
            if Path(args.out).exists():
                raise ValueError('Family destination already exists')
            base = check_root(read(args.base))
            family = {'schema': 1, 'id': str(uuid.uuid4()), 'name': args.name,
                      'type': base['type'], 'version': base.get('version', '2.3.0.0'),
                      'base_nozzle': args.base_nozzle, 'base': settings(base), 'variants': {}}
            if base.get('filament_id'):
                family['filament_id'] = base['filament_id']
            variants = [(args.base_nozzle, base)]
            for item in args.variant:
                diameter, path = item.split('=', 1)
                variants.append((nozzle(diameter), check_root(read(path))))
            for diameter, config in variants:
                if diameter in family['variants'] or config['type'] != family['type']:
                    raise ValueError('Duplicate nozzle or mismatched preset type')
                values = settings(config)
                if family['base'].keys() - values.keys():
                    raise ValueError('Variant omits base keys; import equally resolved presets')
                if family['type'] == 'machine':
                    diameters = config.get('nozzle_diameter', [])
                    if isinstance(diameters, str):
                        diameters = diameters.split(',')
                    if len(diameters) != 1 or nozzle(diameters[0]) != diameter:
                        raise ValueError('Machine variant must have exactly the specified nozzle diameter')
                family['variants'][diameter] = {'id': str(uuid.uuid4()), 'name': config['name'],
                    'overrides': {k: v for k, v in values.items() if k not in family['base'] or v != family['base'][k]}}
            make_plan(family, Path(args.out).parent / 'validation-only')
            if len({v['name'] for v in family['variants'].values()}) != len(family['variants']):
                raise ValueError('Each nozzle variant must have a distinct preset name')
            write(args.out, family)
        else:
            family = family_read(args.path)
            if args.action in ('set', 'reset'):
                family = edit_family(args)
            elif args.action == 'build':
                destination = require_managed_path(args.out)
                if destination.exists():
                    raise ValueError('Build destination already exists; use plan/apply to update a library')
                plan = make_plan(family, destination)
                for entry in plan['entries']:
                    write(safe_target(destination, entry['path']), entry['after'])
                write(destination / 'manifest.json', {'schema': 1, 'family_id': family['id'],
                    'profiles': [{'id': v['id'], 'name': v['config']['name'], 'nozzle': v['nozzle'],
                                  'sha256': digest(v['config'])} for v in build(family)]})
                return {'status': 'built', 'out': args.out, 'profiles': len(family['variants'])}
        return {'family': family, 'revision': digest(family)}
    if args.action == 'list':
        return [{'path': str(p), 'name': read(p).get('name'), 'type': read(p).get('type')}
                for p in sorted(Path(args.path).glob('*/base/*.json'))]
    if args.action == 'show':
        config = check_root(read(args.path))
        return {'profile': config, 'revision': digest(config), 'source': 'standalone_root'}
    if args.action == 'compare':
        return differences(read(args.path), read(args.other))
    if args.action in ('validate', 'verify'):
        return verify(read(args.path), args.orca_command)
    if args.action == 'plan':
        plan = make_plan(family_read(args.path), args.library)
        write(args.out, plan)
        return plan | {'revision': digest(plan)}
    if args.action == 'recover':
        return recover_local(args.path, args.orca_command, args.direction)
    plan = read(args.path)
    if args.action == 'rollback':
        if plan.get('status') != 'applied':
            raise ValueError('Rollback requires an applied receipt')
        plan['entries'] = [dict(e, before=e['after'], after=e['before']) for e in plan['entries']]
    return apply_plan(plan, args.orca_command)


if __name__ == '__main__':
    try:
        print(json.dumps({'ok': True, 'result': main()}, allow_nan=False))
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        print(json.dumps({'ok': False, 'error': str(error)}))
        sys.exit(2)
