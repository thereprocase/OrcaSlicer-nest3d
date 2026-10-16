#!/usr/bin/env python3
"""Inspect and synchronize a running Orca GUI through its opt-in loopback API."""

import argparse
import http.client
import ipaddress
import json
import math
import os
import re
import sys
import time
from pathlib import Path
from urllib.parse import urlsplit


class ClientError(Exception):
    pass


class Parser(argparse.ArgumentParser):
    def error(self, message):
        # Do not echo untrusted arguments, which might contain credentials.
        raise ClientError('Invalid arguments; use --help for usage')


def endpoint(url):
    try:
        parsed = urlsplit(url)
        address = ipaddress.ip_address(parsed.hostname or '')
        port = parsed.port if parsed.port is not None else 13619
        if (parsed.scheme != 'http' or not address.is_loopback or parsed.username
                or parsed.password or parsed.path not in ('', '/')
                or parsed.query or parsed.fragment or not 1 <= port <= 65535):
            raise ValueError()
        return str(address), port
    except ValueError:
        raise ClientError('URL must be an HTTP loopback IP origin without credentials or a path') from None


def scrub(value, token):
    """Keep authentication data out of JSON even if an endpoint echoes it."""
    if isinstance(value, dict):
        return {key: ('[redacted]' if re.search(
            r'token|password|secret|credential|authorization|access_code|api_key', key, re.IGNORECASE)
            else scrub(item, token)) for key, item in value.items()}
    if isinstance(value, list):
        return [scrub(item, token) for item in value]
    if isinstance(value, str):
        return value.replace(token, '[redacted]') if token else value
    return value


def request(host, port, token, method, path, timeout):
    connection = http.client.HTTPConnection(host, port, timeout=timeout)
    try:
        connection.request(method, path, body=b'' if method == 'POST' else None,
                           headers={'Authorization': 'Bearer ' + token,
                                    'Accept': 'application/json'})
        response = connection.getresponse()
        # http.client never follows redirects; do not expose their Location header.
        if 300 <= response.status < 400:
            raise ClientError('API redirects are prohibited')
        if response.status >= 400:
            return response.status, {}
        data = response.read(8 * 1024 * 1024 + 1)
        if len(data) > 8 * 1024 * 1024:
            raise ClientError('API response exceeds size limit')
        try:
            result = json.loads(data)
        except (ValueError, UnicodeError):
            raise ClientError('API returned invalid JSON') from None
        if not isinstance(result, dict):
            raise ClientError('API response must be a JSON object')
        return response.status, scrub(result, token)
    except (OSError, http.client.HTTPException):
        raise ClientError('Cannot reach GUI API or request timed out') from None
    finally:
        connection.close()


def http_error(code, job_id=None):
    result = {'status': 'conflict' if code == 409 else 'error', 'http_status': code,
              'error': {401: 'GUI API authentication failed',
                        403: 'GUI API access denied',
                        409: 'GUI is busy or cannot synchronize in its current state',
                        503: 'GUI is not ready, logged out, or in stealth mode'}.get(
                            code, 'GUI API request failed')}
    if job_id:
        result['job_id'] = job_id
        result['accepted'] = True
    return result


def run(args):
    host, port = endpoint(args.url)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 600:
        raise ClientError('Timeout must be greater than zero and at most 600 seconds')
    if args.wait and args.command != 'sync':
        raise ClientError('--wait is only available for sync')
    try:
        token = (Path(args.token_file).read_text(encoding='utf-8') if args.token_file
                 else os.environ.get('ORCA_AGENT_API_TOKEN', '')).strip()
    except (OSError, UnicodeError):
        raise ClientError('Cannot read GUI API token file') from None
    if len(token) < 32 or not token.isascii() or any(not 33 <= ord(c) <= 126 for c in token):
        raise ClientError('Provide a GUI API token of at least 32 printable ASCII characters')
    deadline = time.monotonic() + args.timeout
    try:
        code, data = request(host, port, token, 'POST' if args.command == 'sync' else 'GET',
                             '/v1/presets/sync' if args.command == 'sync' else '/v1/' + args.command,
                             min(args.timeout, 10))
    except ClientError as error:
        if args.command == 'sync':
            raise ClientError(str(error) + '; sync acceptance is unknown. Inspect status before retrying') from None
        raise
    if code >= 400:
        return http_error(code)
    if args.command != 'sync':
        if code != 200:
            raise ClientError('Unexpected API response status')
        return {'status': 'ok', 'result': data}
    sync = data.get('sync', {})
    if code != 202 or not isinstance(sync, dict) or not isinstance(sync.get('job_id'), str) or not sync['job_id']:
        raise ClientError('Sync response did not identify an accepted job; inspect status before retrying')
    job_id = sync['job_id']
    if not args.wait:
        return {'status': 'accepted', 'job_id': job_id, 'result': data}
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return {'status': 'timeout', 'accepted': True, 'job_id': job_id,
                    'error': 'Wait expired; synchronization may still finish. Inspect status before retrying'}
        time.sleep(min(1, remaining))
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            continue
        try:
            code, data = request(host, port, token, 'GET', '/v1/status', min(remaining, 10))
        except ClientError as error:
            return {'status': 'error', 'accepted': True, 'job_id': job_id, 'error': str(error)}
        if code >= 400:
            return http_error(code, job_id)
        sync = data.get('sync', {})
        if code != 200 or not isinstance(sync, dict) or sync.get('job_id') != job_id:
            return {'status': 'conflict', 'accepted': True, 'job_id': job_id,
                    'error': 'GUI no longer reports this sync job; inspect status'}
        state = sync.get('state')
        if state in ('applied', 'failed', 'cancelled'):
            return {'status': 'applied' if state == 'applied' else 'error',
                    'job_id': job_id, 'result': data}
        if state not in ('accepted', 'waiting', 'pulling', 'reloading'):
            return {'status': 'error', 'accepted': True, 'job_id': job_id,
                    'error': 'GUI returned an unknown sync state'}


def main(argv=None):
    parser = Parser(description=__doc__)
    parser.add_argument('command', choices=('status', 'presets', 'sync'))
    parser.add_argument('--url', default='http://127.0.0.1:13619')
    parser.add_argument('--token-file')
    parser.add_argument('--wait', action='store_true')
    parser.add_argument('--timeout', type=float, default=60,
                        help='Request/wait budget in seconds, at most 600 (default: 60)')
    try:
        result = run(parser.parse_args(argv))
    except ClientError as error:
        result = {'status': 'error', 'error': str(error)}
    print(json.dumps(result, sort_keys=True))
    return {'ok': 0, 'accepted': 0, 'applied': 0, 'conflict': 3, 'timeout': 4}.get(result['status'], 1)


if __name__ == '__main__':
    sys.exit(main())
