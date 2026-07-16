from __future__ import annotations

from datetime import datetime, timezone
from io import StringIO
import unittest

from simulador_serial.monitor_serial import (
    FrameCollector,
    FrameLogger,
    analyze_frame,
    crc16_x25,
    encode_frame,
    format_hex,
)


class MonitorSerialTests(unittest.TestCase):
    def test_formata_exatamente_com_dois_digitos_e_espacos(self) -> None:
        sample = bytes.fromhex("7F 01 0E F4 C3 C3 7F")
        self.assertEqual(format_hex(sample), "7F 01 0E F4 C3 C3 7F")

    def test_crc_x25_vetor_padrao(self) -> None:
        self.assertEqual(crc16_x25(b"123456789"), 0x906E)

    def test_codifica_e_analisa_quadro_valido(self) -> None:
        frame = encode_frame(0x01, 0x10, bytes.fromhex("34 00 00"))
        analysis = analyze_frame(frame)
        self.assertTrue(analysis.valid)
        self.assertTrue(analysis.crc_valid)
        self.assertEqual(analysis.address, 0x01)
        self.assertEqual(analysis.control, 0x10)
        self.assertEqual(analysis.information, bytes.fromhex("34 00 00"))
        self.assertIn("PROC=GET_TILT", analysis.description)
        self.assertIn("LEN=0(OK)", analysis.description)

    def test_aplica_transparencia_e_recupera_7e_e_7d(self) -> None:
        information = bytes((0x0E, 0x02, 0x00, 0x7E, 0x7D))
        frame = encode_frame(0x7E, 0x10, information)
        self.assertIn(bytes.fromhex("7D 5E"), frame)
        self.assertIn(bytes.fromhex("7D 5D"), frame)
        analysis = analyze_frame(frame)
        self.assertTrue(analysis.valid)
        self.assertEqual(analysis.address, 0x7E)
        self.assertEqual(analysis.information, information)

    def test_coletor_remonta_fragmentos_e_quadros_com_flag_compartilhado(self) -> None:
        first = encode_frame(0x01, 0x93)
        second = encode_frame(0x02, 0x10, bytes.fromhex("05 00 00"))
        wire = first[:-1] + second
        collector = FrameCollector()
        frames = []
        for chunk in (wire[:2], wire[2:7], wire[7:10], wire[10:]):
            frames.extend(collector.feed(chunk))
        self.assertEqual(frames, [first, second])

    def test_detecta_fcs_invalido(self) -> None:
        frame = bytearray(encode_frame(0x01, 0x93))
        frame[1] ^= 0x01
        analysis = analyze_frame(bytes(frame))
        self.assertFalse(analysis.valid)
        self.assertFalse(analysis.crc_valid)

    def test_logger_mantem_dump_hexadecimal_na_linha(self) -> None:
        output = StringIO()
        logger = FrameLogger(
            stream=output,
            clock=lambda: datetime(2026, 7, 16, 12, 0, tzinfo=timezone.utc),
        )
        frame = encode_frame(0x01, 0x93)
        logger.log_frame("CONTROLADOR -> RET", frame)
        line = output.getvalue()
        self.assertIn(f"CONTROLADOR -> RET | {format_hex(frame)} | FCS=OK", line)
        self.assertIn("ADDR=0x01 SNRM", line)

    def test_logger_reproduz_formato_do_arquivo_dois_rets(self) -> None:
        output = StringIO()
        moments = iter((10.0, 10.10139, 10.15247))
        logger = FrameLogger(
            stream=output,
            details=False,
            reference_format=True,
            monotonic=lambda: next(moments),
        )
        request = encode_frame(0x01, 0x93)
        response = encode_frame(0x01, 0x73)
        logger.log_frame("CONTROLADOR -> RET", request)
        logger.log_frame("RET -> CONTROLADOR", response)
        lines = output.getvalue().splitlines()
        self.assertEqual(
            lines[0],
            f"[101.39 ms] [TX] Quadro: {format_hex(request)} | Intervalo: 0.00 ms",
        )
        self.assertEqual(
            lines[1],
            f"[152.47 ms] [RX] Quadro: {format_hex(response)} | Intervalo: 51.08 ms",
        )


if __name__ == "__main__":
    unittest.main()
