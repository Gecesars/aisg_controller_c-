#!/usr/bin/env python3
"""Par de portas seriais virtuais com ponte e monitor AISG 2.0."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import errno
import os
from pathlib import Path
import pty
import selectors
import signal
import subprocess
import termios
import threading
import tty

try:
    from .monitor_serial import FrameCollector, FrameLogger
except ImportError:  # Permite executar diretamente: python3 simulador_serial.py
    from monitor_serial import FrameCollector, FrameLogger


@dataclass
class _Endpoint:
    name: str
    master_fd: int
    slave_fd: int
    slave_path: str
    collector: FrameCollector = field(default_factory=FrameCollector)
    pending: bytearray = field(default_factory=bytearray)


class SerialBridge:
    """Liga duas PTYs em modo null-modem e observa os quadros nos dois sentidos."""

    def __init__(
        self,
        *,
        controller_link: str | Path | None = "/tmp/aisg_controlador",
        ret_link: str | Path | None = "/tmp/aisg_ret",
        logger: FrameLogger | None = None,
    ) -> None:
        self.controller_link = Path(controller_link) if controller_link is not None else None
        self.ret_link = Path(ret_link) if ret_link is not None else None
        self.logger = logger if logger is not None else FrameLogger()
        self._owns_logger = logger is None
        self._selector = selectors.DefaultSelector()
        self._controller: _Endpoint | None = None
        self._ret: _Endpoint | None = None
        self._created_links: list[tuple[Path, str]] = []
        self._opened = False

    @staticmethod
    def _new_endpoint(name: str) -> _Endpoint:
        master_fd, slave_fd = pty.openpty()
        tty.setraw(slave_fd, termios.TCSANOW)
        attributes = termios.tcgetattr(slave_fd)
        attributes[4] = termios.B9600
        attributes[5] = termios.B9600
        attributes[2] |= termios.CLOCAL | termios.CREAD
        attributes[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
        attributes[2] |= termios.CS8
        termios.tcsetattr(slave_fd, termios.TCSANOW, attributes)
        os.set_blocking(master_fd, False)
        return _Endpoint(name, master_fd, slave_fd, os.ttyname(slave_fd))

    def _make_link(self, link: Path | None, target: str) -> str:
        if link is None:
            return target
        link.parent.mkdir(parents=True, exist_ok=True)
        if os.path.lexists(link):
            if not link.is_symlink():
                raise FileExistsError(f"recusando substituir arquivo que não é link: {link}")
            link.unlink()
        link.symlink_to(target)
        self._created_links.append((link, target))
        return str(link)

    def open(self) -> "SerialBridge":
        if self._opened:
            return self
        try:
            self._controller = self._new_endpoint("CONTROLADOR")
            self._ret = self._new_endpoint("RET")
            self._selector.register(self._controller.master_fd, selectors.EVENT_READ, self._controller)
            self._selector.register(self._ret.master_fd, selectors.EVENT_READ, self._ret)
            self._opened = True
            self._make_link(self.controller_link, self._controller.slave_path)
            self._make_link(self.ret_link, self._ret.slave_path)
        except Exception:
            self.close()
            raise
        return self

    @property
    def controller_slave_path(self) -> str:
        if self._controller is None:
            raise RuntimeError("ponte ainda não foi aberta")
        return self._controller.slave_path

    @property
    def ret_slave_path(self) -> str:
        if self._ret is None:
            raise RuntimeError("ponte ainda não foi aberta")
        return self._ret.slave_path

    @property
    def controller_port(self) -> str:
        return str(self.controller_link) if self.controller_link is not None else self.controller_slave_path

    @property
    def ret_port(self) -> str:
        return str(self.ret_link) if self.ret_link is not None else self.ret_slave_path

    def _peer(self, endpoint: _Endpoint) -> _Endpoint:
        assert self._controller is not None and self._ret is not None
        return self._ret if endpoint is self._controller else self._controller

    def _direction(self, endpoint: _Endpoint) -> str:
        return "CONTROLADOR -> RET" if endpoint is self._controller else "RET -> CONTROLADOR"

    def _events_for(self, endpoint: _Endpoint) -> int:
        events = selectors.EVENT_READ
        if endpoint.pending:
            events |= selectors.EVENT_WRITE
        return events

    def _update_events(self, endpoint: _Endpoint) -> None:
        self._selector.modify(endpoint.master_fd, self._events_for(endpoint), endpoint)

    def _read(self, endpoint: _Endpoint) -> None:
        peer = self._peer(endpoint)
        while True:
            try:
                data = os.read(endpoint.master_fd, 4096)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno == errno.EIO:
                    break
                raise
            if not data:
                break
            peer.pending.extend(data)
            for frame in endpoint.collector.feed(data):
                self.logger.log_frame(self._direction(endpoint), frame)
        self._update_events(peer)

    def _write(self, endpoint: _Endpoint) -> None:
        while endpoint.pending:
            try:
                written = os.write(endpoint.master_fd, endpoint.pending)
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno == errno.EIO:
                    break
                raise
            if written <= 0:
                break
            del endpoint.pending[:written]
        self._update_events(endpoint)

    def poll_once(self, timeout: float | None = 0.1) -> int:
        if not self._opened:
            raise RuntimeError("ponte ainda não foi aberta")
        events = self._selector.select(timeout)
        for key, mask in events:
            endpoint: _Endpoint = key.data
            if mask & selectors.EVENT_READ:
                self._read(endpoint)
            if mask & selectors.EVENT_WRITE:
                self._write(endpoint)
        return len(events)

    def run(self, stop_event: threading.Event | None = None) -> None:
        if not self._opened:
            self.open()
        stop = stop_event if stop_event is not None else threading.Event()
        while not stop.is_set():
            self.poll_once(0.25)

    def close(self) -> None:
        for link, target in reversed(self._created_links):
            try:
                if link.is_symlink() and os.readlink(link) == target:
                    link.unlink()
            except FileNotFoundError:
                pass
        self._created_links.clear()

        for endpoint in (self._controller, self._ret):
            if endpoint is None:
                continue
            try:
                self._selector.unregister(endpoint.master_fd)
            except (KeyError, ValueError):
                pass
            for descriptor in (endpoint.master_fd, endpoint.slave_fd):
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        self._controller = None
        self._ret = None
        self._opened = False
        if self._owns_logger:
            self.logger.close()

    def __enter__(self) -> "SerialBridge":
        return self.open()

    def __exit__(self, *_: object) -> None:
        self.close()


class RetHostRunner:
    """Compila e executa o mesmo núcleo C do firmware com plataforma POSIX."""

    def __init__(
        self,
        *,
        source_directory: str | Path | None = None,
        build_directory: str | Path | None = None,
        executable: str | Path | None = None,
        device_count: int = 2,
        address: int | None = None,
        logger: FrameLogger | None = None,
    ) -> None:
        repository = Path(__file__).resolve().parent.parent
        self.source_directory = (
            Path(source_directory) if source_directory else repository / "RET"
        )
        self.build_directory = (
            Path(build_directory) if build_directory else self.source_directory / "build-host"
        )
        self.executable = (
            Path(executable) if executable else self.build_directory / "ret_host"
        )
        if not 1 <= device_count <= 8:
            raise ValueError("a quantidade de RETs deve estar entre 1 e 8")
        if address is not None and not 1 <= address <= 254:
            raise ValueError("o endereço RET deve estar entre 1 e 254")
        if address is not None and address + device_count - 1 > 254:
            raise ValueError("a faixa de endereços RET excede 254")
        self.device_count = device_count
        self.address = address
        self.logger = logger
        self.process: subprocess.Popen[bytes] | None = None

    def _event(self, message: str) -> None:
        if self.logger is not None:
            self.logger.log_event(message)

    @staticmethod
    def _run_build(command: list[str], working_directory: Path) -> None:
        result = subprocess.run(
            command,
            cwd=working_directory,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise RuntimeError(f"falha ao preparar RET host: {result.stdout.strip()}")

    def ensure_built(self) -> None:
        self._event("verificando executável C do firmware RET")
        self._run_build(
            [
                "cmake", "-S", str(self.source_directory),
                "-B", str(self.build_directory),
                "-DRET_BUILD_HOST=ON", "-DRET_BUILD_TESTS=OFF",
                "-DRET_BUILD_STM32=OFF",
            ],
            self.source_directory.parent,
        )
        self._run_build(
            [
                "cmake", "--build", str(self.build_directory),
                "--target", "ret_host", "--parallel",
            ],
            self.source_directory.parent,
        )
        if not self.executable.is_file():
            raise RuntimeError(f"executável RET não foi produzido: {self.executable}")

    def start(self, port: str, *, build: bool = True) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        if build:
            self.ensure_built()
        command = [
            str(self.executable), "--port", port,
            "--devices", str(self.device_count),
        ]
        if self.address is not None:
            command.extend(("--address", str(self.address)))
        self.process = subprocess.Popen(command)
        try:
            status = self.process.wait(timeout=0.05)
        except subprocess.TimeoutExpired:
            status = None
        if status is not None:
            raise RuntimeError(
                f"firmware RET host não permaneceu ativo; código {status}")
        state = (
            "NoAddress, aguardando descoberta XID"
            if self.address is None
            else f"endereços 0x{self.address:02X}..0x{self.address + self.device_count - 1:02X}"
        )
        self._event(
            f"{self.device_count} RETs do firmware iniciados automaticamente ({state})")

    def check(self) -> None:
        if self.process is None:
            return
        status = self.process.poll()
        if status is not None:
            raise RuntimeError(
                f"firmware RET host encerrou inesperadamente com código {status}")

    def stop(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        self.process = None


def main(argv: list[str] | None = None) -> int:
    default_log = Path(__file__).resolve().with_name("frames_aisg2.log")
    parser = argparse.ArgumentParser(
        description="Cria duas portas seriais virtuais e monitora quadros AISG 2.0.")
    parser.add_argument("--porta-controlador", default="/tmp/aisg_controlador",
                        help="link da porta usada pelo controlador")
    parser.add_argument("--porta-ret", default="/tmp/aisg_ret",
                        help="link da porta usada pelo RET")
    parser.add_argument("--sem-links", action="store_true",
                        help="usa diretamente os nomes /dev/pts/N")
    parser.add_argument("--log", default=str(default_log), help="arquivo de log persistente")
    parser.add_argument("--sem-arquivo-log", action="store_true",
                        help="não grava arquivo, somente exibe no terminal")
    parser.add_argument("--detalhes", action="store_true",
                        help="acrescenta CRC, endereço e decodificação do controle")
    parser.add_argument("--sem-detalhes", action="store_false", dest="detalhes",
                        help=argparse.SUPPRESS)
    parser.set_defaults(detalhes=False)
    parser.add_argument("--sem-ret-automatico", action="store_true",
                        help="não compila nem inicia o firmware RET host")
    parser.add_argument("--quantidade-rets", type=int, default=2,
                        help="quantidade de RETs automáticos (padrão: 2)")
    parser.add_argument("--endereco-ret", type=int,
                        help="pré-atribui o primeiro endereço; por padrão usa descoberta XID")
    parser.add_argument("--executavel-ret",
                        help="usa um ret_host já compilado neste caminho")
    args = parser.parse_args(argv)

    logger = FrameLogger(
        log_path=None if args.sem_arquivo_log else args.log,
        details=args.detalhes,
        reference_format=True,
    )
    bridge = SerialBridge(
        controller_link=None if args.sem_links else args.porta_controlador,
        ret_link=None if args.sem_links else args.porta_ret,
        logger=logger,
    )
    ret_runner = None if args.sem_ret_automatico else RetHostRunner(
        executable=args.executavel_ret,
        device_count=args.quantidade_rets,
        address=args.endereco_ret,
        logger=logger,
    )
    stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        bridge.open()
        logger.log_event(
            f"porta CONTROLADOR: {bridge.controller_port} -> {bridge.controller_slave_path}")
        logger.log_event(f"porta RET: {bridge.ret_port} -> {bridge.ret_slave_path}")
        if ret_runner is not None:
            ret_runner.start(bridge.ret_port, build=args.executavel_ret is None)
        logger.log_event("ponte 9600 8N1 ativa; pressione Ctrl+C para encerrar")
        while not stop.is_set():
            bridge.poll_once(0.1)
            if ret_runner is not None and not stop.is_set():
                ret_runner.check()
        logger.log_event("encerrando ponte serial")
    finally:
        if ret_runner is not None:
            ret_runner.stop()
        bridge.close()
        logger.close()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
