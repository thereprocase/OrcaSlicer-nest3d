"""GUI client contract tests against local mock HTTP endpoints; no Orca required."""

import importlib.util
import io
import json
import os
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'agent_gui', Path(__file__).resolve().parents[2] / 'scripts/agent_gui.py')
client = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(client)
TOKEN = 'local-test-secret-' + 'x' * 32


class GuiClientTest(unittest.TestCase):
    def setUp(self):
        self.responses = []
        self.requests = []
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                owner.requests.append((self.command, self.path, self.headers.get('Authorization'), body))
                code, data, headers = owner.responses.pop(0) if owner.responses else (
                    500, {}, {})
                payload = data if isinstance(data, bytes) else json.dumps(data).encode()
                self.send_response(code)
                for key, value in headers.items():
                    self.send_header(key, value)
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            do_POST = do_GET

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_server)
        self.url = 'http://127.0.0.1:' + str(self.server.server_port)

    def stop_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def invoke(self, *args):
        output = io.StringIO()
        with patch.dict(os.environ, {'ORCA_AGENT_API_TOKEN': TOKEN}), redirect_stdout(output):
            code = client.main([*args, '--url', self.url])
        self.assertNotIn(TOKEN, output.getvalue())
        return code, json.loads(output.getvalue())

    def respond(self, code, data, **headers):
        self.responses.append((code, data, headers))

    def test_status_and_presets_authenticate_and_preserve_snapshots(self):
        for command, data in [('status', {'ready': True}),
                              ('presets', {'saved': ['Normal'], 'active': {'dirty': True}})]:
            with self.subTest(command=command):
                self.respond(200, data)
                code, result = self.invoke(command)
                self.assertEqual(code, 0)
                self.assertEqual(result, {'status': 'ok', 'result': data})
                self.assertEqual(self.requests[-1][:3], ('GET', '/v1/' + command, 'Bearer ' + TOKEN))

    def test_token_file_overrides_environment_and_secret_is_redacted(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'token'
            file_token = 'file-only-' + 'z' * 40
            path.write_text(file_token + '\n')
            self.respond(200, {'token': file_token, 'message': file_token,
                               'nested': {'access_token': 'cloud-credential'}})
            code, result = self.invoke('status', '--token-file', str(path))
        self.assertEqual(code, 0)
        self.assertNotIn(file_token, json.dumps(result))
        self.assertNotIn('cloud-credential', json.dumps(result))
        self.assertEqual(self.requests[0][2], 'Bearer ' + file_token)

    def test_sync_acceptance_is_not_application_and_post_body_is_empty(self):
        self.respond(202, {'sync': {'job_id': 'one', 'state': 'accepted'}})
        code, result = self.invoke('sync')
        self.assertEqual(code, 0)
        self.assertEqual(result['status'], 'accepted')
        self.assertEqual(self.requests[0][0:2], ('POST', '/v1/presets/sync'))
        self.assertEqual(self.requests[0][3], b'')

    def test_wait_follows_job_until_gui_applies_it(self):
        for code, state in [(202, 'accepted'), (200, 'pulling'), (200, 'reloading'), (200, 'applied')]:
            self.respond(code, {'sync': {'job_id': 'one', 'state': state}})
        with patch.object(client.time, 'sleep'):
            code, result = self.invoke('sync', '--wait')
        self.assertEqual(code, 0)
        self.assertEqual(result['status'], 'applied')
        self.assertEqual(len(self.requests), 4)

    def test_wait_does_not_claim_another_jobs_success(self):
        self.respond(202, {'sync': {'job_id': 'one', 'state': 'accepted'}})
        self.respond(200, {'sync': {'job_id': 'two', 'state': 'applied'}})
        with patch.object(client.time, 'sleep'):
            code, result = self.invoke('sync', '--wait')
        self.assertEqual(code, 3)
        self.assertEqual(result['status'], 'conflict')
        self.assertEqual(result['job_id'], 'one')

    def test_wait_timeout_retains_accepted_job_without_resubmission(self):
        self.respond(202, {'sync': {'job_id': 'one', 'state': 'accepted'}})
        with patch.object(client.time, 'monotonic', side_effect=[0, 2]):
            code, result = self.invoke('sync', '--wait', '--timeout', '1')
        self.assertEqual(code, 4)
        self.assertEqual(result['status'], 'timeout')
        self.assertTrue(result['accepted'])
        self.assertEqual(len(self.requests), 1)

    def test_failed_and_cancelled_jobs_are_not_applied(self):
        for state in ('failed', 'cancelled'):
            with self.subTest(state=state):
                self.respond(202, {'sync': {'job_id': 'one', 'state': 'accepted'}})
                self.respond(200, {'sync': {'job_id': 'one', 'state': state}})
                with patch.object(client.time, 'sleep'):
                    code, result = self.invoke('sync', '--wait')
                self.assertEqual(code, 1)
                self.assertEqual(result['status'], 'error')

    def test_lost_post_response_does_not_retry_and_reports_uncertain_acceptance(self):
        with patch.object(client, 'request', side_effect=client.ClientError('Request timed out')) as request:
            code, result = self.invoke('sync', '--wait')
        self.assertEqual(code, 1)
        self.assertIn('acceptance is unknown', result['error'])
        self.assertEqual(request.call_count, 1)

    def test_poll_transport_failure_retains_accepted_job(self):
        with patch.object(client, 'request', side_effect=[
                (202, {'sync': {'job_id': 'one', 'state': 'accepted'}}),
                client.ClientError('Request timed out')]), patch.object(client.time, 'sleep'):
            code, result = self.invoke('sync', '--wait')
        self.assertEqual(code, 1)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['job_id'], 'one')

    def test_http_conflict_and_authentication_errors_hide_response_body(self):
        for status, expected in [(409, 'conflict'), (401, 'error'), (503, 'error')]:
            with self.subTest(status=status):
                self.respond(status, {'error': 'cloud-password ' + TOKEN})
                code, result = self.invoke('sync')
                self.assertNotEqual(code, 0)
                self.assertEqual(result['status'], expected)
                self.assertNotIn('cloud-password', json.dumps(result))

    def test_redirect_is_not_followed(self):
        self.respond(302, {}, Location=self.url + '/stolen')
        code, result = self.invoke('status')
        self.assertEqual(code, 1)
        self.assertIn('redirects', result['error'])
        self.assertEqual(len(self.requests), 1)

    def test_invalid_json_and_missing_job_id_fail(self):
        self.respond(200, b'not-json')
        self.assertEqual(self.invoke('status')[0], 1)
        self.respond(202, {'sync': {'state': 'accepted'}})
        self.assertEqual(self.invoke('sync')[0], 1)

    def test_remote_origins_and_invalid_timeouts_are_rejected_before_network(self):
        for url in ('https://127.0.0.1', 'http://example.com', 'http://192.168.1.2',
                    'http://user:secret@127.0.0.1', 'http://127.0.0.1/path',
                    'http://127.0.0.1?token=secret', 'http://127.0.0.1:0'):
            with self.subTest(url=url), self.assertRaises(client.ClientError):
                client.endpoint(url)
        for timeout in ('nan', 'inf', '-1', '0', '601'):
            self.assertEqual(self.invoke('status', '--timeout', timeout)[0], 1)
        self.assertEqual(self.requests, [])


if __name__ == '__main__':
    unittest.main()
