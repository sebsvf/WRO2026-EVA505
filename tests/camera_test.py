#!/usr/bin/env python3
"""
Captures a handful of frames, reports actual resolution/FPS achieved,
and saves one sample frame to disk for a visual sanity check.

"""

import time

import cv2

from raspberry_pi.camera.camera import Camera


def main():
    cam = Camera(resolution=(640, 480), target_fps=25)
    cam.start()

    n_frames = 30
    t0 = time.monotonic()
    last_frame = None
    for _ in range(n_frames):
        last_frame = cam.capture_frame()
    elapsed = time.monotonic() - t0

    achieved_fps = n_frames / elapsed
    print(f"Captured {n_frames} frames in {elapsed:.2f}s "
          f"({achieved_fps:.1f} fps achieved)")
    print(f"Frame shape: {last_frame.shape}")
    print(f"Camera healthy: {cam.is_healthy()}")

    out_path = "camera_test_sample.jpg"
    cv2.imwrite(out_path, last_frame)
    print(f"Sample frame saved to {out_path}")

    cam.stop()


if __name__ == "__main__":
    main()
