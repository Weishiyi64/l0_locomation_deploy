#!/usr/bin/env python3
"""Small local check for the two-stage OTA sender handshake."""

import queue
import hashlib
import struct
import socket
import sys
import tempfile
import threading
from pathlib import Path
from unittest.mock import patch

import ota_send


def check_sender(ready: bool) -> None:
    body = b"sample OTA package"
    header = bytearray(4096)
    struct.pack_into("<IIIII4BI", header, 0, 0xBEAF5AA5, 0xA5A55A5A,
                     0x6750, len(body), 1, 0, 1, 1, 4, 0)
    header[28:60] = hashlib.sha256(body).digest()
    package = bytes(header) + body
    with tempfile.TemporaryDirectory() as directory, socket.socket() as listener:
        path = Path(directory) / "update_sign.bin"
        path.write_bytes(package)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        outcome = queue.Queue()

        def serve() -> None:
            try:
                with listener.accept()[0] as peer:
                    peer.settimeout(2)
                    magic = bytearray()
                    while len(magic) < len(ota_send.MAGIC):
                        magic += peer.recv(len(ota_send.MAGIC) - len(magic))
                    assert magic == ota_send.MAGIC
                    peer.sendall(b"OTA_ENT")
                    peer.sendall(b"ERED\n")
                    peer.settimeout(0.1)
                    try:
                        assert peer.recv(1) == b"", "package sent before OTA_READY"
                    except socket.timeout:
                        pass
                    peer.settimeout(2)
                    if ready:
                        peer.sendall(b"OTA_RE")
                        peer.sendall(b"ADY\n")
                        received = bytearray()
                        while len(received) < len(package):
                            received += peer.recv(len(package) - len(received))
                        assert received == package
                        peer.sendall(b"OTA_OK\n")
                    else:
                        peer.sendall(b"OTA_ERR:FLASH\n")
                        assert peer.recv(1) == b"", "package sent after OTA_ERR"
                outcome.put(None)
            except Exception as exc:
                outcome.put(exc)

        thread = threading.Thread(target=serve)
        thread.start()
        original_argv = sys.argv
        sys.argv = ["ota_send.py", "127.0.0.1", str(path), "--port", str(listener.getsockname()[1])]
        try:
            if ready:
                assert ota_send.main() == 0
            else:
                try:
                    ota_send.main()
                except RuntimeError as exc:
                    assert "OTA_ERR:FLASH" in str(exc)
                else:
                    raise AssertionError("sender accepted OTA_ERR")
        finally:
            sys.argv = original_argv
            thread.join(timeout=3)
        assert not thread.is_alive()
        error = outcome.get_nowait()
        if error is not None:
            raise error


def check_p4_file_selection() -> None:
    assert ota_send.DEFAULT_P4_IMAGE == (
        Path(ota_send.__file__).resolve().parents[2] / "ota/esp32p4/uart_dma_ota.bin")
    with tempfile.TemporaryDirectory() as directory:
        default = Path(directory) / "default.bin"
        override = Path(directory) / "override.bin"
        default.write_bytes(b"default image")
        override.write_bytes(b"override image")
        with patch.object(ota_send, "DEFAULT_P4_IMAGE", default), \
                patch("p4_ota.validate_image", return_value={}) as validate, \
                patch("p4_ota.upgrade", return_value=0) as upgrade, \
                patch("p4_ota.query", return_value=0) as query:
            for options, expected in [([], b"default image"),
                                      (["--file", str(override)], b"override image")]:
                with patch.object(sys, "argv", ["ota_send.py", "esp32p4", *options]):
                    assert ota_send.main() == 0
                validate.assert_called_with(expected)
                assert upgrade.call_args.args[1] == expected
            assert upgrade.call_count == 2
            default.unlink()
            validate.reset_mock()
            with patch.object(sys, "argv", ["ota_send.py", "esp32p4", "--query"]):
                assert ota_send.main() == 0
            query.assert_called_once()
            assert query.call_args.args[1] is None
            validate.assert_not_called()
            with patch.object(sys, "argv", ["ota_send.py", "esp32p4"]):
                try:
                    ota_send.main()
                except FileNotFoundError:
                    pass
                else:
                    raise AssertionError("missing default image was accepted")
            assert upgrade.call_count == 2
    print("P4 default file, override, query and missing-file checks passed")


if __name__ == "__main__":
    check_p4_file_selection()
    check_sender(True)
    check_sender(False)
