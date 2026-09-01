#!/usr/bin/env python3
"""
Passive encoder read-out: push the vehicle by hand and confirm the
tick count changes in the expected direction/magnitude, before relying
on it for closed-loop velocity control or lap counting.

"""

import time

from raspberry_pi.communication.uart import SerialLink

PORT = "/dev/serial0"   # TODO: confirm against config.yaml
BAUD = 115200


def main():
    link = SerialLink(PORT, BAUD)
    print("Reading encoder ticks for 10 seconds -- move the wheel by hand.")
    for _ in range(20):
        print(f"ticks = {link.last_encoder_ticks}")
        time.sleep(0.5)
    link.close()


if __name__ == "__main__":
    main()
