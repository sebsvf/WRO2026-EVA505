import time
import logging

import numpy as np

try:
    from picamera2 import Picamera2
    _PICAMERA_AVAILABLE = True
except ImportError:
    # Allows the module to be imported (and unit-tested) on a dev machine
    # that does not have picamera2 installed, e.g. a laptop.
    _PICAMERA_AVAILABLE = False

logger = logging.getLogger("camera")


class CameraFault(Exception):
    """Raised when the camera stops delivering fresh frames."""


class Camera:
    def __init__(self, resolution=(640, 480), target_fps=25,
                 stale_frame_timeout_s=1.0):
        self.resolution = tuple(resolution)
        self.target_fps = target_fps
        self.stale_frame_timeout_s = stale_frame_timeout_s

        self._picam2 = None
        self._last_frame_ts = None
        self._simulated = not _PICAMERA_AVAILABLE

        if self._simulated:
            logger.warning(
                "picamera2 not available -- Camera is running in SIMULATED "
                "mode and will yield synthetic blank frames. This is only "
                "useful for running the FSM/vision code off-target."
            )

    def start(self):
        """Open and configure the camera. Call once before capture_frame()."""
        if self._simulated:
            self._last_frame_ts = time.monotonic()
            return

        self._picam2 = Picamera2()
        config = self._picam2.create_video_configuration(
            main={"size": self.resolution, "format": "RGB888"},
            controls={"FrameRate": self.target_fps},
        )
        self._picam2.configure(config)
        self._picam2.start()
        # Give the sensor a moment to settle (auto-exposure convergence
        # before calibration.py locks it down).
        time.sleep(0.5)
        self._last_frame_ts = time.monotonic()
        logger.info("Camera started at %sx%s @ %s fps target",
                    *self.resolution, self.target_fps)

    def set_manual_controls(self, exposure_us=None, awb_gains=None):
        """
        Lock exposure/white balance. Used by calibration.py once the
        camera has converged on good auto values on-site, so the HSV
        thresholds stay valid for the rest of the run.
        """
        if self._simulated or self._picam2 is None:
            return
        controls = {"AeEnable": False, "AwbEnable": False}
        if exposure_us is not None:
            controls["ExposureTime"] = int(exposure_us)
        if awb_gains is not None:
            controls["ColourGains"] = tuple(awb_gains)
        self._picam2.set_controls(controls)
        logger.info("Camera manual controls locked: %s", controls)

    def capture_frame(self) -> np.ndarray:
        """
        Return the latest frame as a BGR NumPy array (OpenCV convention).
        Raises CameraFault if no frame could be obtained within the
        configured staleness timeout.
        """
        now = time.monotonic()

        if self._simulated:
            self._last_frame_ts = now
            return np.zeros((self.resolution[1], self.resolution[0], 3),
                             dtype=np.uint8)

        try:
            frame_rgb = self._picam2.capture_array()
        except Exception as exc:  # pragma: no cover - hardware dependent
            if now - self._last_frame_ts > self.stale_frame_timeout_s:
                raise CameraFault(f"No frame for over "
                                   f"{self.stale_frame_timeout_s}s: {exc}")
            raise

        self._last_frame_ts = now
        # picamera2 delivers RGB; OpenCV pipeline downstream expects BGR.
        return frame_rgb[:, :, ::-1]

    def is_healthy(self) -> bool:
        if self._last_frame_ts is None:
            return False
        return (time.monotonic() - self._last_frame_ts) < self.stale_frame_timeout_s

    def stop(self):
        if not self._simulated and self._picam2 is not None:
            self._picam2.stop()
