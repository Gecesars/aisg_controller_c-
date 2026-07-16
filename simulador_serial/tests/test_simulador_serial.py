from __future__ import annotations

from io import StringIO
import os
from pathlib import Path
import tempfile
import time
import unittest

from simulador_serial.monitor_serial import (
    FrameCollector,
    FrameLogger,
    analyze_frame,
    encode_frame,
    format_hex,
)
from simulador_serial.simulador_serial import RetHostRunner, SerialBridge


def receive_through_bridge(bridge: SerialBridge, descriptor: int, expected_size: int) -> bytes:
    received = bytearray()
    deadline = time.monotonic() + 2.0
    while len(received) < expected_size and time.monotonic() < deadline:
        bridge.poll_once(0.02)
        try:
            received.extend(os.read(descriptor, expected_size - len(received)))
        except BlockingIOError:
            pass
    return bytes(received)


class SerialBridgeTests(unittest.TestCase):
    def test_encaminha_e_monitora_nos_dois_sentidos(self) -> None:
        output = StringIO()
        logger = FrameLogger(stream=output)
        with tempfile.TemporaryDirectory() as directory:
            controller_link = Path(directory) / "controlador"
            ret_link = Path(directory) / "ret"
            bridge = SerialBridge(
                controller_link=controller_link,
                ret_link=ret_link,
                logger=logger,
            )
            controller_fd = -1
            ret_fd = -1
            try:
                bridge.open()
                self.assertTrue(controller_link.is_symlink())
                self.assertTrue(ret_link.is_symlink())
                controller_fd = os.open(controller_link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                ret_fd = os.open(ret_link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

                request = encode_frame(0x01, 0x10, bytes.fromhex("34 00 00"))
                os.write(controller_fd, request)
                self.assertEqual(receive_through_bridge(bridge, ret_fd, len(request)), request)

                response = encode_frame(0x01, 0x30, bytes.fromhex("34 03 00 00 64 00"))
                os.write(ret_fd, response)
                self.assertEqual(receive_through_bridge(bridge, controller_fd, len(response)), response)

                logged = output.getvalue()
                self.assertIn(f"CONTROLADOR -> RET | {format_hex(request)}", logged)
                self.assertIn(f"RET -> CONTROLADOR | {format_hex(response)}", logged)
                self.assertEqual(logged.count("FCS=OK"), 2)
            finally:
                for descriptor in (controller_fd, ret_fd):
                    if descriptor >= 0:
                        os.close(descriptor)
                bridge.close()

            self.assertFalse(os.path.lexists(controller_link))
            self.assertFalse(os.path.lexists(ret_link))

    def test_recusa_substituir_arquivo_comum(self) -> None:
        logger = FrameLogger(stream=StringIO())
        with tempfile.TemporaryDirectory() as directory:
            controller_path = Path(directory) / "controlador"
            controller_path.write_text("não apagar", encoding="utf-8")
            bridge = SerialBridge(
                controller_link=controller_path,
                ret_link=Path(directory) / "ret",
                logger=logger,
            )
            with self.assertRaises(FileExistsError):
                bridge.open()
            self.assertEqual(controller_path.read_text(encoding="utf-8"), "não apagar")

    def test_inicia_firmware_c_e_responde_snrm_e_initial_data(self) -> None:
        output = StringIO()
        logger = FrameLogger(stream=output)
        bridge = SerialBridge(controller_link=None, ret_link=None, logger=logger)
        runner = RetHostRunner(device_count=1, address=1, logger=logger)
        controller_fd = -1
        try:
            bridge.open()
            runner.ensure_built()
            runner.start(bridge.ret_port, build=False)
            controller_fd = os.open(
                bridge.controller_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            collector = FrameCollector()

            def exchange(frame: bytes) -> bytes:
                os.write(controller_fd, frame)
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    bridge.poll_once(0.01)
                    runner.check()
                    try:
                        received = os.read(controller_fd, 4096)
                    except BlockingIOError:
                        continue
                    frames = collector.feed(received)
                    if frames:
                        return frames[-1]
                self.fail("firmware RET não respondeu dentro do prazo")

            ua = analyze_frame(exchange(encode_frame(0x01, 0x93)))
            self.assertTrue(ua.valid)
            self.assertEqual(ua.address, 0x01)
            self.assertEqual(ua.control, 0x73)

            initial = analyze_frame(
                exchange(encode_frame(0x01, 0x10, bytes.fromhex("05 00 00"))))
            self.assertTrue(initial.valid)
            self.assertEqual(initial.information[0], 0x05)
            self.assertIn(b"ATC-RET-HOST", initial.information)

            logged = output.getvalue()
            self.assertIn("CONTROLADOR -> RET", logged)
            self.assertIn("RET -> CONTROLADOR", logged)
        finally:
            if controller_fd >= 0:
                os.close(controller_fd)
            runner.stop()
            bridge.close()


if __name__ == "__main__":
    unittest.main()
