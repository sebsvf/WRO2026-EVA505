#!/usr/bin/env python3
"""
Drives the motor at a few open-loop-target speeds via the ESP32's
closed-loop velocity controller and prints back the encoder-derived
telemetry, to sanity-check the drivetrain and the UART link before
trusting the FSM with it.

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

    test_speeds_mm_s = [0, 200, 400, 200, 0]
    for speed in test_speeds_mm_s:
        print(f"-> SET_SPEED:{speed}")
        link.send(commands.set_speed(speed))
        time.sleep(2.0)
        print(f"   last encoder ticks reported: {link.last_encoder_ticks}")

    link.send(commands.stop())
    link.close()
    print("Motor test complete.")


if __name__ == "__main__":
    main()
