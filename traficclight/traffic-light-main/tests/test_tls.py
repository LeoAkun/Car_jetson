import json
import os
from pathlib import Path
import socket
import ssl
import subprocess
import tempfile
import threading
import time
import unittest

from traffic_light_stream.tls import (
    TLSConfig,
    TLSConfigurationError,
    TLSHandshakeTimeout,
    TLSRoleError,
    create_client_context,
    create_server_context,
    loads_tls_config,
    tls_config_from_mapping,
    wrap_client_socket,
    wrap_server_socket,
)


ROLE_PREFIX = "urn:car-jetson:traffic-light:role:"


class CertificateFixture:
    def __init__(self, root: Path):
        self.root = root
        self._run("genpkey", "-algorithm", "EC", "-pkeyopt", "ec_paramgen_curve:P-256", "-out", "ca.key")
        self._run(
            "req", "-x509", "-new", "-key", "ca.key", "-sha256", "-days", "30",
            "-subj", "/CN=P5-P6 Test CA", "-out", "ca.crt",
        )
        self.issue("server", role="service", eku="serverAuth", server_names=True)
        self.issue("expired-server", role="service", eku="serverAuth", server_names=True, days=-1)
        self.issue("sender", role="video-sender", eku="clientAuth")
        self.issue("controller", role="controller", eku="clientAuth")
        self.issue("bad-eku-sender", role="video-sender", eku="serverAuth")

        other = root / "other"
        other.mkdir()
        self.other = CertificateFixture._secondary_ca(other)

    @classmethod
    def _secondary_ca(cls, root: Path):
        obj = object.__new__(cls)
        obj.root = root
        obj._run("genpkey", "-algorithm", "EC", "-pkeyopt", "ec_paramgen_curve:P-256", "-out", "ca.key")
        obj._run(
            "req", "-x509", "-new", "-key", "ca.key", "-sha256", "-days", "30",
            "-subj", "/CN=Other Test CA", "-out", "ca.crt",
        )
        obj.issue("sender", role="video-sender", eku="clientAuth")
        return obj

    def _run(self, *args):
        subprocess.run(
            ["openssl", *args], cwd=self.root, check=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        )

    def issue(self, name, *, role, eku, server_names=False, days=30):
        self._run("genpkey", "-algorithm", "EC", "-pkeyopt", "ec_paramgen_curve:P-256", "-out", f"{name}.key")
        os.chmod(self.root / f"{name}.key", 0o600)
        self._run("req", "-new", "-key", f"{name}.key", "-subj", f"/CN={name}", "-out", f"{name}.csr")
        sans = [f"URI:{ROLE_PREFIX}{role}"]
        if server_names:
            sans[:0] = ["DNS:traffic-light.service", "IP:127.0.0.1"]
        extension = self.root / f"{name}.ext"
        extension.write_text(
            "basicConstraints=critical,CA:FALSE\n"
            "keyUsage=critical,digitalSignature\n"
            f"extendedKeyUsage={eku}\n"
            f"subjectAltName={','.join(sans)}\n"
        )
        self._run(
            "x509", "-req", "-in", f"{name}.csr", "-CA", "ca.crt", "-CAkey", "ca.key",
            "-CAcreateserial", "-days", str(days), "-sha256", "-extfile", f"{name}.ext", "-out", f"{name}.crt",
        )

    def config(self, name, *, side, expected_role, server_name=None, timeout=0.5):
        return TLSConfig(
            ca_file=str(self.root / "ca.crt"),
            cert_file=str(self.root / f"{name}.crt"),
            key_file=str(self.root / f"{name}.key"),
            server_name=server_name if side == "client" else None,
            expected_peer_role=expected_role,
            handshake_timeout_s=timeout,
        )


class OneShotServer:
    def __init__(self, config):
        self.config = config
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.ready = False
        self.error = None
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        raw = None
        tls = None
        try:
            raw, _ = self.listener.accept()
            tls = wrap_server_socket(raw, self.config)
            raw = None
            self.ready = True
            if tls.recv(1) == b"x":
                tls.sendall(b"y")
        except BaseException as exc:
            self.error = exc
        finally:
            if tls is not None:
                tls.close()
            if raw is not None:
                raw.close()
            self.listener.close()

    def join(self):
        self.thread.join(2.0)
        if self.thread.is_alive():
            self.listener.close()
            raise AssertionError("loopback TLS server did not terminate")


@unittest.skipUnless(ssl.HAS_TLSv1_3, "OpenSSL TLS 1.3 is required")
class TLSLoopbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil_which("openssl"):
            raise unittest.SkipTest("openssl CLI is required for temporary test certificates")
        cls.temporary = tempfile.TemporaryDirectory(prefix="p5-p6-tls-")
        cls.certs = CertificateFixture(Path(cls.temporary.name))

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def server_config(self, expected="video-sender", timeout=0.5):
        return self.certs.config("server", side="server", expected_role=expected, timeout=timeout)

    def client_config(self, name="sender", *, server_name="traffic-light.service", timeout=0.5):
        return self.certs.config(
            name, side="client", expected_role="service", server_name=server_name, timeout=timeout
        )

    def connect(self, server, config=None):
        raw = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
        return wrap_client_socket(raw, config or self.client_config())

    def test_valid_mtls_is_tls13_and_only_then_ready(self):
        server = OneShotServer(self.server_config())
        client = self.connect(server)
        self.assertEqual(client.version(), "TLSv1.3")
        client.sendall(b"x")
        self.assertEqual(client.recv(1), b"y")
        client.close()
        server.join()
        self.assertTrue(server.ready)
        self.assertIsNone(server.error)

    def test_ip_san_is_validated_and_accepted(self):
        server = OneShotServer(self.server_config())
        client = self.connect(server, self.client_config(server_name="127.0.0.1"))
        client.sendall(b"x")
        self.assertEqual(client.recv(1), b"y")
        client.close()
        server.join()
        self.assertTrue(server.ready)
        self.assertIsNone(server.error)

    def test_expired_server_certificate_is_rejected(self):
        expired = self.certs.config(
            "expired-server", side="server", expected_role="video-sender", timeout=0.5
        )
        server = OneShotServer(expired)
        with self.assertRaises(ssl.SSLCertVerificationError):
            self.connect(server)
        server.join()
        self.assertFalse(server.ready)

    def test_missing_client_certificate_is_rejected(self):
        server = OneShotServer(self.server_config())
        config = self.client_config()
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.minimum_version = context.maximum_version = ssl.TLSVersion.TLSv1_3
        context.check_hostname = True
        context.verify_mode = ssl.CERT_REQUIRED
        context.load_verify_locations(cafile=config.ca_file)
        try:
            raw = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
            client = context.wrap_socket(raw, server_hostname=config.server_name)
            client.close()
        except (ssl.SSLError, OSError):
            pass
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, ssl.SSLError)

    def test_wrong_server_ca_is_rejected(self):
        server = OneShotServer(self.server_config())
        config = self.client_config()
        config = TLSConfig(
            ca_file=str(self.certs.other.root / "ca.crt"), cert_file=config.cert_file,
            key_file=config.key_file, server_name=config.server_name,
            expected_peer_role=config.expected_peer_role, handshake_timeout_s=config.handshake_timeout_s,
        )
        with self.assertRaises(ssl.SSLError):
            self.connect(server, config)
        server.join()
        self.assertFalse(server.ready)

    def test_client_signed_by_wrong_ca_is_rejected(self):
        server = OneShotServer(self.server_config())
        config = TLSConfig(
            ca_file=str(self.certs.root / "ca.crt"),
            cert_file=str(self.certs.other.root / "sender.crt"),
            key_file=str(self.certs.other.root / "sender.key"),
            server_name="traffic-light.service", expected_peer_role="service", handshake_timeout_s=0.5,
        )
        try:
            client = self.connect(server, config)
            client.close()
        except (ssl.SSLError, OSError):
            pass
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, ssl.SSLError)

    def test_server_san_mismatch_is_rejected(self):
        server = OneShotServer(self.server_config())
        with self.assertRaises(ssl.SSLCertVerificationError):
            self.connect(server, self.client_config(server_name="wrong.service"))
        server.join()
        self.assertFalse(server.ready)

    def test_application_role_mismatch_is_rejected_after_ca_validation(self):
        server = OneShotServer(self.server_config(expected="video-sender"))
        client = self.connect(server, self.client_config("controller"))
        try:
            client.sendall(b"x")
            client.recv(1)
        except OSError:
            pass
        client.close()
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, TLSRoleError)

    def test_wrong_client_eku_is_rejected(self):
        server = OneShotServer(self.server_config())
        try:
            client = self.connect(server, self.client_config("bad-eku-sender"))
            client.close()
        except ssl.SSLError:
            pass
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, ssl.SSLError)

    def test_tls12_only_peer_is_rejected(self):
        server = OneShotServer(self.server_config())
        config = self.client_config()
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.minimum_version = context.maximum_version = ssl.TLSVersion.TLSv1_2
        context.check_hostname = True
        context.verify_mode = ssl.CERT_REQUIRED
        context.load_verify_locations(cafile=config.ca_file)
        context.load_cert_chain(config.cert_file, config.key_file)
        with self.assertRaises(ssl.SSLError):
            raw = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
            context.wrap_socket(raw, server_hostname=config.server_name)
        server.join()
        self.assertFalse(server.ready)

    def test_partial_handshake_uses_one_total_deadline(self):
        server = OneShotServer(self.server_config(timeout=0.12))
        raw = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
        time.sleep(0.2)
        raw.close()
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, TLSHandshakeTimeout)

    def test_disconnect_during_handshake_is_rejected(self):
        server = OneShotServer(self.server_config(timeout=0.5))
        raw = socket.create_connection(("127.0.0.1", server.port), timeout=1.0)
        raw.close()
        server.join()
        self.assertFalse(server.ready)
        self.assertIsInstance(server.error, (ssl.SSLError, OSError))

    def test_contexts_are_strict_and_server_requires_certificate(self):
        client = create_client_context(self.client_config())
        server = create_server_context(self.server_config())
        self.assertEqual(client.minimum_version, ssl.TLSVersion.TLSv1_3)
        self.assertEqual(client.maximum_version, ssl.TLSVersion.TLSv1_3)
        self.assertTrue(client.check_hostname)
        self.assertEqual(client.verify_mode, ssl.CERT_REQUIRED)
        self.assertEqual(server.minimum_version, ssl.TLSVersion.TLSv1_3)
        self.assertEqual(server.maximum_version, ssl.TLSVersion.TLSv1_3)
        self.assertFalse(server.check_hostname)
        self.assertEqual(server.verify_mode, ssl.CERT_REQUIRED)

    def test_private_key_with_group_or_other_access_is_rejected(self):
        config = self.client_config()
        os.chmod(config.key_file, 0o644)
        try:
            with self.assertRaises(TLSConfigurationError):
                create_client_context(config)
        finally:
            os.chmod(config.key_file, 0o600)

    def test_context_creation_failure_closes_raw_socket(self):
        config = self.client_config()
        raw, peer = socket.socketpair()
        os.chmod(config.key_file, 0o700)
        try:
            with self.assertRaises(TLSConfigurationError):
                wrap_client_socket(raw, config)
            self.assertEqual(raw.fileno(), -1)
        finally:
            os.chmod(config.key_file, 0o600)
            peer.close()

    def test_direct_relative_path_config_is_rejected(self):
        config = self.client_config()
        relative = TLSConfig(
            ca_file="ca.crt", cert_file=config.cert_file, key_file=config.key_file,
            server_name=config.server_name, expected_peer_role=config.expected_peer_role,
            handshake_timeout_s=config.handshake_timeout_s,
        )
        with self.assertRaises(TLSConfigurationError):
            create_client_context(relative)

    def test_strict_tls_config_parser(self):
        config = self.client_config()
        values = {
            "ca_file": config.ca_file, "cert_file": config.cert_file, "key_file": config.key_file,
            "server_name": config.server_name, "expected_peer_role": config.expected_peer_role,
            "handshake_timeout_s": config.handshake_timeout_s,
        }
        self.assertEqual(loads_tls_config(json.dumps(values), side="client"), config)
        for changed in (
            {**values, "unknown": True},
            {key: value for key, value in values.items() if key != "key_file"},
            {**values, "ca_file": "relative/ca.crt"},
            {**values, "expected_peer_role": "admin"},
            {**values, "handshake_timeout_s": True},
            {**values, "handshake_timeout_s": float("nan")},
        ):
            with self.assertRaises(TLSConfigurationError):
                tls_config_from_mapping(changed, side="client")
        duplicate = json.dumps(values)[:-1] + ',"server_name":"traffic-light.service"}'
        with self.assertRaises(TLSConfigurationError):
            loads_tls_config(duplicate, side="client")
        with self.assertRaises(TLSConfigurationError):
            tls_config_from_mapping(values, side="server")


def shutil_which(command):
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / command
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


if __name__ == "__main__":
    unittest.main()
