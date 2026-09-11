"""Bounded serial I/O, immutable telemetry snapshots and explicit failures."""

import logging
import secrets
import threading
import time

import serial

from .protocol import UINT32_MAX, Framer, Telemetry, decode, encode, newer

logger = logging.getLogger(__name__)


class LinkFault(RuntimeError):
    pass


class SerialLink:
    def __init__(
        self,
        port,
        baudrate=115200,
        read_timeout_s=0.02,
        watchdog_reply_timeout_s=0.25,
        write_timeout_s=0.1,
        *,
        transport=None,
        session=None,
    ):
        if min(read_timeout_s, watchdog_reply_timeout_s, write_timeout_s) <= 0:
            raise ValueError("UART timeouts must be positive")
        self._ser = (
            transport
            if transport is not None
            else serial.Serial(
                port, baudrate, timeout=read_timeout_s, write_timeout=write_timeout_s
            )
        )
        self.session = session or secrets.randbelow(UINT32_MAX) + 1
        self.watchdog_reply_timeout_s = watchdog_reply_timeout_s
        self._sequence = 0
        self._telemetry = None
        self._last_reply_ts = None
        self._error = None
        self._closed = False
        self._rx_lock = threading.Lock()
        self._tx_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True, name="eva505-uart"
        )
        self._reader_thread.start()

    @property
    def telemetry(self):
        with self._rx_lock:
            return self._telemetry

    @property
    def last_status(self):
        snapshot = self.telemetry
        return "FAULT" if self._error or (snapshot and snapshot.fault) else "OK"

    def send(self, message: str):
        if not message or message != message.strip():
            raise ValueError("Empty/padded command")
        with self._tx_lock:
            if self._closed:
                raise LinkFault("UART already closed")
            self._sequence = (self._sequence + 1) & UINT32_MAX
            line = encode(f"C1:{self.session}:{self._sequence}:{message}")
            try:
                if self._ser.write(line) != len(line):
                    raise LinkFault("Partial UART write")
            except (OSError, serial.SerialException, LinkFault) as exc:
                self._error = LinkFault(f"UART write failed: {exc}")
                raise self._error from exc
        return self._sequence

    def handshake_ok(self, timeout_s=1.0):
        self.send("HELLO")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.is_alive():
                return True
            if self._error:
                raise self._error
            self._stop_event.wait(0.01)
        return False

    def is_alive(self):
        with self._rx_lock:
            return (
                not self._closed
                and self._error is None
                and self._telemetry is not None
                and self._telemetry.fault == 0
                and self._last_reply_ts is not None
                and time.monotonic() - self._last_reply_ts < self.watchdog_reply_timeout_s
            )

    def _accept_line(self, raw):
        try:
            snapshot = Telemetry.parse(decode(raw))
        except (ValueError, UnicodeError):
            return  # Noise must not refresh the watchdog.
        if snapshot.session != self.session:
            return
        with self._rx_lock:
            previous = self._telemetry
            if previous is not None:
                if snapshot.boot_id != previous.boot_id:
                    self._error = LinkFault("ESP32 restarted during this session")
                    return
                if not newer(snapshot.counter, previous.counter):
                    return
            self._telemetry = snapshot
            self._last_reply_ts = time.monotonic()

    def _read_loop(self):
        framer = Framer()
        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(min(max(self._ser.in_waiting, 1), 256))
                for line in framer.feed(chunk):
                    self._accept_line(line)
            except (OSError, serial.SerialException) as exc:
                if not self._stop_event.is_set():
                    self._error = LinkFault(f"UART read failed: {exc}")
                    logger.error("%s", self._error)
                return

    def close(self):
        if self._closed:
            return
        try:
            self.send("STOP")
        except (LinkFault, OSError):
            logger.warning("STOP could not be sent; ESP32 command lease will expire")
        finally:
            self._closed = True
            self._stop_event.set()
            try:
                self._ser.cancel_read()
            except (AttributeError, OSError):
                pass
            self._reader_thread.join(timeout=1.0)
            self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
