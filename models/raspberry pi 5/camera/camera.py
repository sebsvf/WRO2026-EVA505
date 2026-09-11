import logging
import math
import time

import numpy as np

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None


logger = logging.getLogger("camera")


class CameraFault(Exception):
    """Error de inicio, captura o comunicación con la cámara."""


class Camera:
    def __init__(
        self,
        resolution=(640, 480),
        target_fps=25,
        stale_frame_timeout_s=1.0,
        simulated=False,
    ):
        self.resolution = tuple(resolution)
        self.target_fps = float(target_fps)
        self.stale_frame_timeout_s = float(stale_frame_timeout_s)

        if (
            len(self.resolution) != 2
            or any(
                not isinstance(v, int) or v <= 0
                for v in self.resolution
            )
        ):
            raise ValueError("resolution debe contener dos enteros positivos")

        for value in (self.target_fps, self.stale_frame_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("FPS y timeout deben ser positivos y finitos")

        self._simulated = bool(simulated)
        self._picam2 = None
        self._started = False
        self._faulted = False
        self._last_frame_ts = None
        self.analogue_gain = None

    @property
    def simulated(self):
        return self._simulated

    def _require_started(self):
        if not self._started:
            raise CameraFault("Primero debes llamar a camera.start()")
        if self._faulted:
            raise CameraFault(
                "La cámara falló; llama a stop() y start() para reiniciarla"
            )

    def start(self):
        if self._started:
            self._require_started()
            return

        self._faulted = False
        self._last_frame_ts = None
        self.analogue_gain = None

        if self._simulated:
            self._started = True
            logger.warning("Cámara simulada: se entregarán imágenes negras")
            return

        if Picamera2 is None:
            raise CameraFault(
                "Picamera2 no está instalado. "
                "Para pruebas usa Camera(simulated=True)"
            )

        cam = None
        try:
            cam = Picamera2()
            config = cam.create_video_configuration(
                main={
                    "size": self.resolution,
                    # Picamera2 RGB888 produce un array BGR para OpenCV.
                    "format": "RGB888",
                },
                controls={"FrameRate": self.target_fps},
                # No reutilizar el último frame guardado.
                queue=False,
            )
            cam.configure(config)
            cam.start()
        except Exception as exc:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    logger.exception("No se pudo liberar la cámara")
            raise CameraFault(f"No se pudo iniciar la cámara: {exc}") from exc

        self._picam2 = cam
        self._started = True
        logger.info(
            "Cámara iniciada: %sx%s, objetivo %.1f FPS",
            *self.resolution,
            self.target_fps,
        )

    def _capture(self, metadata=False):
        """Espera limitada; después de un fallo exige reinicio."""
        self._require_started()

        try:
            if metadata:
                job = self._picam2.capture_metadata(wait=False)
            else:
                job = self._picam2.capture_array("main", wait=False)

            return job.get_result(timeout=self.stale_frame_timeout_s)

        except Exception as exc:
            # No acumular nuevos trabajos si uno quedó pendiente.
            self._faulted = True
            self._last_frame_ts = None
            raise CameraFault(
                f"Falló la captura o venció el timeout "
                f"de {self.stale_frame_timeout_s}s: {exc}"
            ) from exc

    def capture_metadata(self):
        self._require_started()

        if self._simulated:
            return {}

        return self._capture(metadata=True)

    def set_auto_controls(self):
        """Activa AE/AWB antes de una nueva calibración."""
        self._require_started()

        if not self._simulated:
            self._picam2.set_controls({
                "AeEnable": True,
                "AwbEnable": True,
            })

        self.analogue_gain = None

    def set_manual_controls(
        self,
        exposure_us=None,
        awb_gains=None,
        analogue_gain=None,
    ):

        self._require_started()

        if self._simulated:
            return

        controls = {}

        if exposure_us is not None or analogue_gain is not None:
            if exposure_us is None or analogue_gain is None:
                raise ValueError(
                    "Proporciona exposure_us y analogue_gain juntos"
                )

            exposure = float(exposure_us)
            gain = float(analogue_gain)

            if (
                not math.isfinite(exposure)
                or exposure < 1
                or not math.isfinite(gain)
                or gain <= 0
            ):
                raise ValueError("Exposición o ganancia inválidas")

            controls.update({
                "AeEnable": False,
                "ExposureTime": int(exposure),
                "AnalogueGain": gain,
            })

        if awb_gains is not None:
            gains = tuple(float(v) for v in awb_gains)
            if len(gains) != 2 or any(
                not math.isfinite(v) or v <= 0 for v in gains
            ):
                raise ValueError("awb_gains debe contener dos valores positivos")

            controls.update({
                "AwbEnable": False,
                "ColourGains": gains,
            })

        if controls:
            self._picam2.set_controls(controls)
            if analogue_gain is not None:
                self.analogue_gain = float(analogue_gain)

            logger.info("Controles manuales solicitados: %s", controls)

    def capture_frame(self) -> np.ndarray:
        """Devuelve una imagen BGR; CameraFault si falla la captura."""
        self._require_started()

        if self._simulated:
            width, height = self.resolution
            frame = np.zeros((height, width, 3), dtype=np.uint8)
        else:
            frame = self._capture()

        if (
            not isinstance(frame, np.ndarray)
            or frame.ndim != 3
            or frame.shape[2] != 3
            or frame.size == 0
        ):
            self._faulted = True
            self._last_frame_ts = None
            raise CameraFault("La cámara entregó un frame inválido")

        self._last_frame_ts = time.monotonic()
        return frame

    def is_healthy(self) -> bool:
        return (
            self._started
            and not self._faulted
            and self._last_frame_ts is not None
            and time.monotonic() - self._last_frame_ts
            < self.stale_frame_timeout_s
        )

    def stop(self):
        cam = self._picam2
        self._picam2 = None
        self._started = False
        self._last_frame_ts = None

        if cam is not None:
            try:
                cam.stop()
            finally:
                cam.close()