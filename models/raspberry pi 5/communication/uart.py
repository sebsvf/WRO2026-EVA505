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

        self._ser = serial.Serial(
            port,
            baudrate,
            timeout=read_timeout_s
        )

        self.last_status = None
        self._last_reply_ts = 0.0
        self._handshake_ok = False

        self._encoder_ticks = 0
        self._encoder_lock = threading.Lock()

        self._stop_event = threading.Event()

        self._reader_thread = threading.Thread(
            target=self._read_loop,
            daemon=True
        )

        self._reader_thread.start()


    def handshake_ok(self) -> bool:
        self.send("PING")

        start = time.monotonic()

        while time.monotonic() - start < 1.0:
            if self._handshake_ok:
                return True

            time.sleep(0.01)

        return False


    def send(self, message: str):
        line = (message.strip() + "\n").encode("ascii")

        try:
            self._ser.write(line)

        except serial.SerialException:
            logger.exception("UART write failed")


    def get_encoder_ticks(self) -> int:
        with self._encoder_lock:
            return self._encoder_ticks


    def is_alive(self) -> bool:
        return (
            time.monotonic() - self._last_reply_ts
            < self.watchdog_reply_timeout_s
        )


    def close(self):
        self._stop_event.set()

        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

        if self._ser.is_open:
            self._ser.close()


    # ---------------- UART READER ----------------

    def _read_loop(self):

        while not self._stop_event.is_set():

            try:
                raw = self._ser.readline()

            except serial.SerialException:
                logger.exception("UART read failed")
                time.sleep(0.1)
                continue


            if not raw:
                continue


            self._last_reply_ts = time.monotonic()

            line = raw.decode(
                "ascii",
                errors="ignore"
            ).strip()

            self._parse_line(line)



    def _parse_line(self, line: str):

        if line in ("STATUS:OK", "PONG"):

            self._handshake_ok = True
            self.last_status = "OK"


        elif line == "STATUS:FAULT":

            self.last_status = "FAULT"
            logger.error(
                "ESP32 reported STATUS:FAULT"
            )


        elif line.startswith("ENC:"):

            try:
                ticks = int(
                    line.split(":", 1)[1]
                )

                with self._encoder_lock:
                    self._encoder_ticks = ticks


            except ValueError:
                logger.warning(
                    "Malformed ENC line: %s",
                    line
                )


        elif line.startswith("RPM:"):
            pass


        else:
            logger.debug(
                "Unrecognized UART line: %s",
                line
            )