#!/usr/bin/env python3
"""Monitor e utilitários de enquadramento HDLC usados pelo AISG 2.0.

O monitor trabalha sobre os bytes que a ponte serial já encaminha. Dessa forma,
ele não abre uma terceira vez as portas e não interfere na comunicação.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import sys
import time
from typing import Callable, Iterable, TextIO


FLAG = 0x7E
ESCAPE = 0x7D
ESCAPE_XOR = 0x20
MAX_UNESCAPED_FRAME_BYTES = 268
MAX_WIRE_FRAME_BYTES = 2 * MAX_UNESCAPED_FRAME_BYTES + 2

AISG2_PROCEDURES = {
    0x04: "GET_ALARMS",
    0x05: "INITIAL_DATA",
    0x06: "CLEAR_ALARMS",
    0x0A: "SELF_TEST",
    0x0E: "SET_DEVICE_DATA",
    0x0F: "GET_DEVICE_DATA",
    0x12: "SUBSCRIBE_ALARMS",
    0x31: "CALIBRATE",
    0x33: "SET_TILT",
    0x34: "GET_TILT",
    0x70: "SET_TMA_MODE",
    0x71: "GET_TMA_MODE",
    0x72: "SET_TMA_GAIN",
    0x73: "GET_TMA_GAIN",
    0x88: "ANTENNA_COUNT",
}


def format_hex(data: bytes | bytearray | memoryview) -> str:
    """Formata octetos em maiúsculas, com dois dígitos e separados por espaço."""

    return " ".join(f"{octet:02X}" for octet in data)


def crc16_x25(data: bytes | bytearray | memoryview) -> int:
    """Calcula o FCS HDLC (CRC-16/X-25) transmitido em little-endian."""

    crc = 0xFFFF
    for octet in data:
        crc ^= octet
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return (~crc) & 0xFFFF


def _escape(data: bytes) -> bytes:
    encoded = bytearray()
    for octet in data:
        if octet in (FLAG, ESCAPE):
            encoded.extend((ESCAPE, octet ^ ESCAPE_XOR))
        else:
            encoded.append(octet)
    return bytes(encoded)


def _unescape(data: bytes) -> tuple[bytes | None, str | None]:
    decoded = bytearray()
    escaped = False
    for octet in data:
        if escaped:
            decoded.append(octet ^ ESCAPE_XOR)
            escaped = False
        elif octet == ESCAPE:
            escaped = True
        elif octet == FLAG:
            return None, "flag 7E não escapado dentro do quadro"
        else:
            decoded.append(octet)
    if escaped:
        return None, "sequência de escape incompleta"
    return bytes(decoded), None


def encode_frame(address: int, control: int, information: bytes = b"") -> bytes:
    """Cria um quadro HDLC AISG válido, útil para testes e diagnóstico."""

    if not 0 <= address <= 0xFF or not 0 <= control <= 0xFF:
        raise ValueError("endereço e controle devem caber em um octeto")
    if len(information) > 264:
        raise ValueError("o campo de informação AISG não pode exceder 264 octetos")
    payload = bytes((address, control)) + information
    fcs = crc16_x25(payload)
    payload += bytes((fcs & 0xFF, fcs >> 8))
    return bytes((FLAG,)) + _escape(payload) + bytes((FLAG,))


def _control_description(control: int) -> str:
    poll_final = (control >> 4) & 1
    if control & 0x01 == 0:
        send_sequence = (control >> 1) & 0x07
        receive_sequence = (control >> 5) & 0x07
        return f"I N(S)={send_sequence} N(R)={receive_sequence} P/F={poll_final}"
    if control & 0x03 == 0x01:
        supervisory = {
            0x01: "RR",
            0x05: "RNR",
            0x09: "REJ",
            0x0D: "SREJ",
        }.get(control & 0x0F, "S?")
        receive_sequence = (control >> 5) & 0x07
        return f"{supervisory} N(R)={receive_sequence} P/F={poll_final}"
    unnumbered = {
        0x2F: "SABM",
        0x43: "DISC",
        0x63: "UA",
        0x83: "SNRM",
        0x87: "FRMR",
        0x0F: "DM",
        0xAF: "XID",
    }.get(control & 0xEF, "U?")
    return f"{unnumbered} P/F={poll_final}"


@dataclass(frozen=True)
class FrameAnalysis:
    """Resultado da inspeção de um quadro recebido no fio."""

    valid: bool
    crc_valid: bool | None
    address: int | None
    control: int | None
    information: bytes
    received_fcs: int | None
    calculated_fcs: int | None
    description: str
    error: str | None = None


def analyze_frame(frame: bytes) -> FrameAnalysis:
    """Valida delimitadores, transparência, tamanho e FCS de um quadro AISG."""

    if len(frame) < 2 or frame[0] != FLAG or frame[-1] != FLAG:
        return FrameAnalysis(False, None, None, None, b"", None, None, "inválido",
                             "quadro deve iniciar e terminar com 7E")

    payload, error = _unescape(frame[1:-1])
    if payload is None:
        return FrameAnalysis(False, None, None, None, b"", None, None, "inválido", error)
    if len(payload) < 4:
        return FrameAnalysis(False, None, None, None, b"", None, None, "inválido",
                             "quadro menor que endereço, controle e FCS")
    if len(payload) > MAX_UNESCAPED_FRAME_BYTES:
        return FrameAnalysis(False, None, payload[0], payload[1], b"", None, None, "inválido",
                             "quadro excede 268 octetos sem escape")

    address = payload[0]
    control = payload[1]
    information = payload[2:-2]
    received_fcs = payload[-2] | payload[-1] << 8
    calculated_fcs = crc16_x25(payload[:-2])
    crc_valid = received_fcs == calculated_fcs

    fields = [_control_description(control)]
    if information:
        procedure = information[0]
        fields.append(f"PROC={AISG2_PROCEDURES.get(procedure, f'0x{procedure:02X}')}")
        if len(information) >= 3:
            declared_length = information[1] | information[2] << 8
            actual_length = len(information) - 3
            length_state = "OK" if declared_length == actual_length else f"REAL={actual_length}"
            fields.append(f"LEN={declared_length}({length_state})")

    return FrameAnalysis(
        valid=crc_valid,
        crc_valid=crc_valid,
        address=address,
        control=control,
        information=information,
        received_fcs=received_fcs,
        calculated_fcs=calculated_fcs,
        description=" ".join(fields),
        error=None if crc_valid else "FCS diferente do CRC-16/X-25 calculado",
    )


class FrameCollector:
    """Reconstrói quadros separados por 7E mesmo quando chegam fragmentados."""

    def __init__(self, maximum_wire_bytes: int = MAX_WIRE_FRAME_BYTES) -> None:
        if maximum_wire_bytes < 6:
            raise ValueError("limite de quadro muito pequeno")
        self.maximum_wire_bytes = maximum_wire_bytes
        self._frame = bytearray()
        self._in_frame = False
        self._dropping = False
        self.dropped_frames = 0

    def feed(self, data: bytes | bytearray | memoryview) -> list[bytes]:
        frames: list[bytes] = []
        for octet in data:
            if octet == FLAG:
                if self._in_frame and not self._dropping and len(self._frame) > 1:
                    self._frame.append(FLAG)
                    frames.append(bytes(self._frame))
                self._frame.clear()
                self._frame.append(FLAG)
                self._in_frame = True
                self._dropping = False
                continue

            if not self._in_frame or self._dropping:
                continue
            self._frame.append(octet)
            if len(self._frame) + 1 > self.maximum_wire_bytes:
                self._frame.clear()
                self._dropping = True
                self.dropped_frames += 1
        return frames

    def reset(self) -> None:
        self._frame.clear()
        self._in_frame = False
        self._dropping = False


class FrameLogger:
    """Exibe e persiste quadros com o dump hexadecimal sempre na mesma linha."""

    def __init__(
        self,
        *,
        stream: TextIO | None = None,
        log_path: str | Path | None = None,
        details: bool = True,
        reference_format: bool = False,
        clock: Callable[[], datetime] | None = None,
        monotonic: Callable[[], float] | None = None,
    ) -> None:
        self.stream = stream if stream is not None else sys.stdout
        self.details = details
        self.reference_format = reference_format
        self.clock = clock if clock is not None else lambda: datetime.now().astimezone()
        self.monotonic = monotonic if monotonic is not None else time.monotonic
        self._started_at = self.monotonic()
        self._last_frame_at: float | None = None
        self._log_file: TextIO | None = None
        if log_path is not None:
            path = Path(log_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = path.open("a", encoding="utf-8", buffering=1)

    def _write(self, line: str) -> None:
        print(line, file=self.stream, flush=True)
        if self._log_file is not None:
            print(line, file=self._log_file, flush=True)

    def log_frame(self, direction: str, frame: bytes) -> FrameAnalysis:
        analysis = analyze_frame(frame)
        if self.reference_format:
            now = self.monotonic()
            elapsed_ms = (now - self._started_at) * 1000.0
            interval_ms = (
                0.0 if self._last_frame_at is None
                else (now - self._last_frame_at) * 1000.0
            )
            self._last_frame_at = now
            label = (
                "TX" if direction.startswith("CONTROLADOR")
                else "RX" if direction.startswith("RET")
                else direction
            )
            line = (
                f"[{elapsed_ms:.2f} ms] [{label}] Quadro: {format_hex(frame)}"
                f" | Intervalo: {interval_ms:.2f} ms"
            )
        else:
            timestamp = self.clock().astimezone().isoformat(timespec="milliseconds")
            line = f"[{timestamp}] {direction} | {format_hex(frame)}"
        if self.details:
            fcs = "OK" if analysis.crc_valid else "ERRO" if analysis.crc_valid is False else "N/A"
            address = f"0x{analysis.address:02X}" if analysis.address is not None else "N/A"
            line += f" | FCS={fcs} ADDR={address} {analysis.description}"
            if analysis.error:
                line += f" ERRO={analysis.error}"
        self._write(line)
        return analysis

    def log_event(self, message: str) -> None:
        if self.reference_format:
            elapsed_ms = (self.monotonic() - self._started_at) * 1000.0
            self._write(f"[{elapsed_ms:.2f} ms] [INFO] {message}")
        else:
            timestamp = self.clock().astimezone().isoformat(timespec="milliseconds")
            self._write(f"[{timestamp}] # {message}")

    def close(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    def __enter__(self) -> "FrameLogger":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _parse_hex(value: str) -> bytes:
    tokens = [token for token in re.split(r"[\s,:;|-]+", value.strip()) if token]
    try:
        octets = bytes(int(token, 16) for token in tokens)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"sequência hexadecimal inválida: {value!r}") from exc
    return octets


def _input_frames(arguments: Iterable[str]) -> list[bytes]:
    joined = " ".join(arguments).strip()
    if joined:
        return [_parse_hex(joined)]
    if sys.stdin.isatty():
        return []
    return [_parse_hex(line) for line in sys.stdin if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Decodifica quadros HDLC AISG 2.0 escritos em hexadecimal.")
    parser.add_argument("hex", nargs="*", help="octetos, por exemplo: 7E 01 93 37 A4 7E")
    parser.add_argument("--sem-detalhes", action="store_true", help="mostra somente direção e octetos")
    args = parser.parse_args(argv)

    try:
        frames = _input_frames(args.hex)
    except ValueError as exc:
        parser.error(str(exc))
    if not frames:
        parser.print_help()
        return 2

    logger = FrameLogger(details=not args.sem_detalhes)
    for frame in frames:
        logger.log_frame("ENTRADA", frame)
    return 0 if all(analyze_frame(frame).valid for frame in frames) else 1


if __name__ == "__main__":
    raise SystemExit(main())
