#!/usr/bin/env python3
"""Manage nozzle families and publish standalone Orca user presets. JSON on stdout."""

import argparse
import base64
import copy
import csv
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
import uuid
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
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


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
    lock = Path(str(args.path) + '.lock')
    try:
        lock.mkdir()
    except FileExistsError:
        raise ValueError('Family is locked by another manager operation') from None
    try:
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
    finally:
        lock.rmdir()


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


def cloud(token_file, action, body=None):
    token = Path(token_file).read_text().strip()
    if not token or '\n' in token:
        raise ValueError('Expected an access token file')
    request = urllib.request.Request('https://api.orcaslicer.com/api/v1/sync/' + action,
        data=None if body is None else json.dumps(body).encode(),
        headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json',
                 'Accept': 'application/json', 'User-Agent': 'OrcaSlicer-AgentProfiles/1'})
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        # Never echo responses or request headers containing account data.
        raise ValueError(f'Cloud HTTP {error.code}; refresh authentication or re-pull on conflict') from None


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


def apply_plan(plan, command):
    if plan.get('schema') != 1 or not plan.get('entries'):
        raise ValueError('Empty or unsupported plan')
    library = Path(plan['library'])
    library.mkdir(parents=True, exist_ok=True)
    lock = library / '.agent-profiles-lock'
    try:
        lock.mkdir()
    except FileExistsError:
        raise ValueError('Library is locked by another manager operation') from None
    changed = []
    try:
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
        receipt = library / '.agent-profile-history' / (transaction + '.json')
        write(receipt, plan | {'status': 'prepared', 'transaction': transaction})
        try:
            for entry in plan['entries']:
                target = safe_target(library, entry['path'])
                changed.append(entry)
                if entry['after'] is None:
                    target.unlink(missing_ok=True)
                else:
                    write(target, entry['after'])
            write(receipt, plan | {'status': 'applied', 'transaction': transaction})
        except Exception:
            for entry in reversed(changed):
                target = safe_target(library, entry['path'])
                if entry['before'] is None:
                    target.unlink(missing_ok=True)
                else:
                    write(target, entry['before'])
            raise
        return {'status': 'local_applied', 'receipt': str(receipt), 'cloud_synced': False}
    finally:
        lock.rmdir()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='group', required=True)
    profiles = sub.add_parser('profiles').add_subparsers(dest='action', required=True)
    for action in ('list', 'show', 'compare', 'validate', 'verify', 'plan', 'apply', 'rollback'):
        p = profiles.add_parser(action)
        p.add_argument('path')
        if action == 'compare':
            p.add_argument('other')
        if action == 'plan':
            p.add_argument('--library', required=True)
            p.add_argument('--out', required=True)
        if action in ('validate', 'verify', 'apply', 'rollback'):
            p.add_argument('--orca-command', required=True, type=json.loads)
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
    clouds = sub.add_parser('cloud').add_subparsers(dest='action', required=True)
    for action in ('pull', 'push'):
        p = clouds.add_parser(action)
        p.add_argument('--token-file', required=True)
        p.add_argument('--account', required=True, help='Expected Orca account UUID')
        p.add_argument('--out', required=True)
        if action == 'push':
            p.add_argument('--profile', required=True)
            p.add_argument('--snapshot', required=True)
            p.add_argument('--id', required=True)
            p.add_argument('--orca-command', required=True, type=json.loads)
    args = parser.parse_args(argv)
    if args.group == 'cloud':
        check_account(args.token_file, args.account)
        if args.action == 'pull':
            result = cloud(args.token_file, 'pull')
            result['account_id'] = args.account
            write(args.out, result)
            return {'status': 'cloud_pulled', 'out': args.out, 'profiles': len(result['upserts'])}
        config = check_root(read(args.profile))
        verify(config, args.orca_command)
        snapshot = read(args.snapshot)
        if snapshot.get('account_id') != args.account:
            raise ValueError('Cloud snapshot belongs to a different account')
        old = next((x for x in snapshot['upserts'] if x['id'] == args.id), None)
        body = {'id': str(uuid.UUID(args.id)), 'name': config['name'], 'content': cloud_wire(config)}
        verify(body['content'], args.orca_command)
        current = cloud(args.token_file, 'pull')['upserts']
        remote = next((x for x in current if x['id'] == args.id), None)
        if (remote is None) != (old is None) or (remote and remote['updated_time'] != old['updated_time']):
            raise ValueError('Cloud revision changed; pull and re-plan')
        if any(x['id'] != args.id and x['name'] == config['name'] for x in current):
            raise ValueError('Cloud name belongs to a different profile ID')
        if old and config['type'] != cloud_content(old).get('type'):
            raise ValueError('Cannot change the preset type of a cloud ID')
        if old:
            body['original_updated_time'] = str(old['updated_time'])
        if Path(args.out).exists():
            raise ValueError('Cloud receipt destination already exists')
        write(args.out, {'status': 'prepared', 'account_id': args.account,
                         'before': old, 'request': body})
        response = cloud(args.token_file, 'push', body)
        if not response.get('updated_time'):
            raise ValueError('Cloud did not confirm a revision')
        actual = next((x for x in cloud(args.token_file, 'pull')['upserts'] if x['id'] == args.id), None)
        if actual is None or cloud_content(actual) != body['content']:
            raise ValueError('Cloud write may have succeeded but readback did not match; pull before retrying')
        write(args.out, {'status': 'cloud_verified', 'account_id': args.account,
                         'before': old, 'after': actual})
        return {'status': 'cloud_verified', 'id': args.id, 'updated_time': actual['updated_time']}
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
                destination = Path(args.out)
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
