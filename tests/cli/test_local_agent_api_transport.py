"""Exercise the real loopback transport without a GUI, account or cloud calls."""

import http.client
import json
import os
import secrets
import socket
import subprocess
import sys
import time
import unittest
from pathlib import Path

BINARY = str(Path(sys.argv.pop(1)).resolve())


class TransportTest(unittest.TestCase):
    def setUp(self):
        self.token = secrets.token_hex(32)
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            self.port = reservation.getsockname()[1]
        self.environment = os.environ | {
            'ORCA_AGENT_API_TOKEN': self.token,
            'ORCA_AGENT_API_PORT': str(self.port),
        }
        self.server = subprocess.Popen(
            [BINARY], env=self.environment, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.stop)
        self.assertEqual(self.server.stdout.readline().strip(), 'READY')

    def stop(self):
        if self.server.poll() is None:
            self.server.stdin.write('stop\n')
            self.server.stdin.flush()
            try:
                stdout, stderr = self.server.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                self.server.kill()
                self.server.communicate()
                self.fail('Transport shutdown did not finish')
            self.assertEqual(self.server.returncode, 0)
            self.assertEqual(stdout.strip(), 'STOPPED')
            self.assertEqual(stderr, '')
        for stream in (self.server.stdin, self.server.stdout, self.server.stderr):
            stream.close()

    def request(self, method='GET', target='/echo', headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        try:
            connection.request(method, target, headers=headers if headers is not None else {
                'Authorization': 'Bearer ' + self.token})
            response = connection.getresponse()
            return response.status, json.loads(response.read()), dict(response.getheaders())
        finally:
            connection.close()

    def connect(self, timeout=3):
        connection = socket.create_connection(('127.0.0.1', self.port), timeout=timeout)
        self.addCleanup(connection.close)
        return connection

    def test_invalid_configuration_does_not_start(self):
        for patch in ({'ORCA_AGENT_API_TOKEN': ''}, {'ORCA_AGENT_API_TOKEN': 'short'},
                      {'ORCA_AGENT_API_TOKEN': 'x' * 32 + '\n'},
                      {'ORCA_AGENT_API_PORT': '65536'}, {'ORCA_AGENT_API_PORT': '0'},
                      {'ORCA_AGENT_API_PORT': 'bad'}):
            result = subprocess.run([BINARY], env=self.environment | patch,
                                    capture_output=True, text=True, timeout=3, check=True)
            self.assertEqual(result.stdout.strip(), 'DISABLED')
            self.assertEqual(result.stderr, '')
        # The real listener excludes another instance on the same endpoint.
        result = subprocess.run([BINARY], env=self.environment, capture_output=True,
                                text=True, timeout=3, check=True)
        self.assertEqual(result.stdout.strip(), 'DISABLED')

    def test_authentication_and_browser_origin_are_required(self):
        self.assertEqual(self.request(headers={})[0], 401)
        self.assertEqual(self.request(headers={'Authorization': 'Bearer ' + 'z' * len(self.token)})[0], 401)
        self.assertEqual(self.request(headers={'Authorization': 'Bearer ' + self.token, 'Origin': 'null'})[0], 403)
        headers = self.request()[2]
        self.assertFalse(any(key.lower().startswith('access-control-') for key in headers))
        self.assertEqual(headers['Cache-Control'], 'no-store')
        connection = self.connect()
        connection.sendall(('GET / HTTP/1.1\r\nHost: localhost\r\n' +
                            ('Authorization: Bearer ' + self.token + '\r\n') * 2 + '\r\n').encode())
        self.assertIn(b'401', connection.recv(1024).split(b'\r\n')[0])

    def test_async_reply_and_one_response_per_request(self):
        self.assertEqual(self.request('POST')[1]['method'], 'POST')
        self.assertEqual(self.request(target='/async')[1], {'async': True})
        self.assertEqual(self.request(target='/twice')[0], 200)
        self.assertEqual(self.request(target='/throw')[0], 500)
        self.assertEqual(self.request(target='/large')[0], 500)

    def test_methods_targets_and_request_bodies_are_rejected(self):
        self.assertEqual(self.request('DELETE')[0], 405)
        self.assertEqual(self.request(target='http://example.com/')[0], 400)
        self.assertEqual(self.request(target='/' + 'x' * 2048)[0], 400)
        for header in ('Content-Length: 1', 'Transfer-Encoding: chunked'):
            connection = self.connect()
            connection.sendall(('POST /echo HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer ' +
                                self.token + '\r\n' + header + '\r\n\r\n').encode())
            data = connection.recv(1024)
            self.assertTrue(not data or b'400' in data.split(b'\r\n')[0])

    def test_malformed_and_oversized_headers_close(self):
        for payload in (b'not HTTP\r\n\r\n', b'GET / HTTP/1.1\r\nX-Large: ' + b'x' * 9000 + b'\r\n\r\n'):
            connection = self.connect()
            connection.sendall(payload)
            try:
                self.assertEqual(connection.recv(1024), b'')
            except ConnectionResetError:
                pass

    def test_deferred_reply_times_out_and_late_reply_is_safe(self):
        connection = self.connect(timeout=13)
        connection.sendall(('GET /defer HTTP/1.1\r\nHost: localhost\r\nAuthorization: Bearer ' +
                            self.token + '\r\n\r\n').encode())
        started = time.monotonic()
        self.assertEqual(connection.recv(1024), b'')
        self.assertGreater(time.monotonic() - started, 8)
        self.assertLess(time.monotonic() - started, 13)
        # Harness invokes the retained Reply after stopping the transport.
        self.stop()

    def test_session_limit_and_shutdown_with_pending_reads(self):
        connections = [self.connect() for _ in range(16)]
        time.sleep(0.1)
        excess = self.connect()
        self.assertEqual(excess.recv(1024), b'')
        self.stop()
        for connection in connections:
            self.assertEqual(connection.recv(1024), b'')


if __name__ == '__main__':
    unittest.main()
