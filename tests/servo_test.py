#!/usr/bin/env python3
"""
Sweeps the steering servo through its calibrated range via the ESP32,
for a mechanical/calibration sanity check (Engineering Diary Table 2:
center=90deg, verify no mechanical interference across the full range).

"""

import time

from raspberry_pi.communication.uart import SerialLink
from raspberry_pi.control import commands

PORT = "/dev/serial0"   # TODO: confirm against config.yaml
BAUD = 115200


def main():
    link = SerialLink(PORT, BAUD)
    print("Waiting for ESP32 handshake...")
    if not link.handshake_ok():
        print("WARNING: no handshake received, continuing anyway")

    sequence_deg = [90, 60, 90, 120, 90]
    for angle in sequence_deg:
        print(f"-> SET_STEERING:{angle}")
        link.send(commands.set_steering(angle))
        time.sleep(1.0)

    link.close()
    print("Servo test complete.")


if __name__ == "__main__":
    main()
