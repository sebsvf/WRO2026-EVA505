import logging
import threading
import time

import serial

logger = logging.getLogger("uart")


class SerialLink:
    def __init__(self, port: str, baudrate: int = 115200,
                 read_timeout_s: float = 0.05,
                 watchdog_reply_timeout_s: float = 0.5):
        self.port = port
        self.baudrate = baudrate
        self.watchdog_reply_timeout_s = watchdog_reply_timeout_s

        self._ser = serial.Serial(port, baudrate, timeout=read_timeout_s)

        self.last_encoder_ticks = 0
        self.last_status = None
        self._last_reply_ts = 0.0
        self._handshake_ok = False

        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def handshake_ok(self) -> bool:
        """
        Sends PING and returns True once at least one STATUS:OK has
        been observed. Used by fsm.py's INITIALIZATION state.
        """
        self.send("PING")
        time.sleep(0.05)
        return self._handshake_ok

    def send(self, message: str):
        line = (message.strip() + "\n").encode("ascii")
        try:
            self._ser.write(line)
        except serial.SerialException:
            logger.exception("UART write failed")

    def is_alive(self) -> bool:
        """False if no reply has been seen within watchdog_reply_timeout_s."""
        return (time.monotonic() - self._last_reply_ts) < self.watchdog_reply_timeout_s

    def close(self):
        self._stop_event.set()
        self._reader_thread.join(timeout=1.0)
        self._ser.close()

    # ---- background reader ---------------------------------------------

    def _read_loop(self):
        while not self._stop_event.is_set():
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                logger.exception("UART read failed")
                time.sleep(0.1)
                continue

            if not raw:
                continue  # read timeout, no data -- normal

            self._last_reply_ts = time.monotonic()
            line = raw.decode("ascii", errors="ignore").strip()
            self._parse_line(line)

    def _parse_line(self, line: str):
        if line in ("STATUS:OK", "PONG"):
            self._handshake_ok = True
            self.last_status = "OK"
        elif line == "STATUS:FAULT":
            self.last_status = "FAULT"
            logger.error("ESP32 reported STATUS:FAULT")
        elif line.startswith("ENC:"):
            try:
                self.last_encoder_ticks = int(line.split(":", 1)[1])
            except ValueError:
                logger.warning("Malformed ENC line: %s", line)
        elif line.startswith("RPM:"):
            pass  # available for logging/telemetry if needed
        else:
            logger.debug("Unrecognized UART line: %s", line)
