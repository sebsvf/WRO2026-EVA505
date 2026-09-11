

import binascii
import math
from dataclasses import dataclass

MAX_LINE = 128
UINT32_MAX = 2**32 - 1


def newer(value: int, previous: int) -> bool:
    return 0 < ((value - previous) & UINT32_MAX) < 2**31


def encode(payload: str) -> bytes:
    raw = payload.encode("ascii")
    if not raw or any(c < 32 or c > 126 for c in raw) or b"*" in raw:
        raise ValueError("Payload must be printable ASCII without '*'")
    if len(raw) + 5 >= MAX_LINE:
        raise ValueError("UART frame too long")
    return raw + f"*{binascii.crc_hqx(raw, 0xFFFF):04X}\n".encode("ascii")


def decode(line: bytes) -> str:
    raw = line.rstrip(b"\r\n")
    payload, separator, checksum = raw.rpartition(b"*")
    if not separator or len(checksum) != 4 or len(raw) >= MAX_LINE:
        raise ValueError("Malformed UART frame")
    if any(c not in b"0123456789ABCDEF" for c in checksum):
        raise ValueError("Malformed CRC")
    if binascii.crc_hqx(payload, 0xFFFF) != int(checksum, 16):
        raise ValueError("CRC mismatch")
    if not payload or any(c < 32 or c > 126 for c in payload):
        raise ValueError("Invalid payload")
    return payload.decode("ascii")


class Framer:
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False

    def feed(self, data: bytes) -> list[bytes]:
        lines = []
        for c in data:
            if c == 10:
                if not self.discard and self.buffer:
                    lines.append(bytes(self.buffer))
                self.buffer.clear()
                self.discard = False
            elif c != 13:
                if c < 32 or c > 126 or len(self.buffer) >= MAX_LINE - 1:
                    self.discard = True
                if not self.discard:
                    self.buffer.append(c)
        return lines


def uint32(text: str) -> int:
    if not text.isascii() or not text.isdecimal() or not 0 <= int(text) <= UINT32_MAX:
        raise ValueError("Invalid uint32")
    return int(text)


@dataclass(frozen=True)
class Telemetry:
    boot_id: int
    session: int
    counter: int
    ack: int
    power: float  # Applied duty fraction, NOT measured speed.
    angle: float  # Commanded servo angle, NOT measured position.
    armed: bool
    started: bool
    fault: int

    @classmethod
    def parse(cls, payload: str):
        fields = payload.split(":")
        if len(fields) != 10 or fields[0] != "T1":
            raise ValueError("Unknown telemetry format")
        ids = [uint32(value) for value in fields[1:5]]
        power, angle = map(float, fields[5:7])
        if not all(math.isfinite(v) for v in (power, angle)):
            raise ValueError("Nonfinite telemetry")
        if not -1 <= power <= 1 or not 0 <= angle <= 180:
            raise ValueError("Out-of-range telemetry")
        if fields[7] not in ("0", "1") or fields[8] not in ("0", "1"):
            raise ValueError("Invalid boolean telemetry")
        fault = uint32(fields[9])
        if fault > 2:
            raise ValueError("Unknown firmware fault")
        return cls(*ids, power, angle, fields[7] == "1", fields[8] == "1", fault)
