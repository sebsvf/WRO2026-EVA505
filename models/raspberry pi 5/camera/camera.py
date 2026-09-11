import argparse
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

        if len(self.resolution) != 2 or any(
            isinstance(v, bool) or not isinstance(v, int) or v <= 0
            for v in self.resolution
        ):
            raise ValueError(
                "resolution debe contener dos enteros positivos"
            )

        for value in (self.target_fps, self.stale_frame_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    "FPS y timeout deben ser positivos y finitos"
                )

        self._simulated = bool(simulated)
        self._picam2 = None
        self._started = False
        self._faulted = False
        self._last_frame_ts = None
        self._last_sensor_timestamp = None
        self.analogue_gain = None

    @property
    def simulated(self):
        return self._simulated

    def _require_started(self):
        if not self._started:
            raise CameraFault(
                "Primero debes llamar a camera.start()"
            )

        if self._faulted:
            raise CameraFault(
                "La cámara falló; llama a stop() y start() "
                "para reiniciarla"
            )

    def _mark_fault(self):
        self._faulted = True
        self._last_frame_ts = None

    def start(self):
        if self._started:
            self._require_started()
            return

        self._faulted = False
        self._last_frame_ts = None
        self._last_sensor_timestamp = None
        self.analogue_gain = None

        if self._simulated:
            self._started = True
            logger.warning(
                "Modo simulado: se entregarán imágenes negras. "
                "No se utilizará una cámara física."
            )
            return

        if Picamera2 is None:
            raise CameraFault(
                "Picamera2 no está disponible en este Python. "
                "En Windows utiliza Camera(simulated=True). "
                "Para captura real ejecuta el programa en la Raspberry "
                "con Picamera2 instalado."
            )

        cam = None

        try:
            cam = Picamera2()

            config = cam.create_video_configuration(
                main={
                    "size": self.resolution,
                    # En Picamera2, RGB888 entrega bytes BGR.
                    "format": "RGB888",
                },
                controls={
                    "FrameRate": self.target_fps,
                },
                queue=False,
            )

            cam.configure(config)
            cam.start()

        except Exception as exc:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    logger.exception(
                        "No se pudo liberar la cámara tras el fallo"
                    )

            self._mark_fault()
            raise CameraFault(
                f"No se pudo iniciar la cámara: {exc}"
            ) from exc

        self._picam2 = cam
        self._started = True

        logger.info(
            "Cámara iniciada: %sx%s, objetivo %.1f FPS",
            *self.resolution,
            self.target_fps,
        )

    def capture_metadata(self):
        """Lee metadatos con espera limitada."""
        self._require_started()

        if self._simulated:
            return {}

        try:
            job = self._picam2.capture_metadata(wait=False)
            return job.get_result(
                timeout=self.stale_frame_timeout_s
            )

        except Exception as exc:
            self._mark_fault()
            raise CameraFault(
                f"No se pudieron obtener metadatos: {exc}"
            ) from exc

    def _apply_controls(self, controls):
        self._require_started()

        if self._simulated:
            return

        try:
            self._picam2.set_controls(controls)

        except Exception as exc:
            self._mark_fault()
            raise CameraFault(
                f"No se pudieron aplicar los controles: {exc}"
            ) from exc

    def set_auto_controls(self):
        """Activa exposición y balance de blancos automáticos."""
        self._apply_controls({
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
        """
        Solicita controles manuales.

        Exposición y ganancia analógica deben proporcionarse juntas.
        El balance de blancos puede ajustarse independientemente.
        """
        self._require_started()
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
                raise ValueError(
                    "Exposición o ganancia inválidas"
                )

            controls.update({
                "AeEnable": False,
                "ExposureTime": int(exposure),
                "AnalogueGain": gain,
            })

        if awb_gains is not None:
            gains = tuple(float(v) for v in awb_gains)

            if len(gains) != 2 or any(
                not math.isfinite(v) or v <= 0
                for v in gains
            ):
                raise ValueError(
                    "awb_gains debe contener dos valores positivos"
                )

            controls.update({
                "AwbEnable": False,
                "ColourGains": gains,
            })

        if not controls:
            return

        self._apply_controls(controls)

        if self._simulated:
            logger.info(
                "Controles validados, pero no aplicados: modo simulado"
            )
            return

        if analogue_gain is not None:
            self.analogue_gain = float(analogue_gain)

        logger.info(
            "Controles manuales solicitados: %s",
            controls,
        )

    def capture_frame(self) -> np.ndarray:
        """
        Devuelve un frame BGR uint8.

        En modo real comprueba que el SensorTimestamp avance.
        En modo simulado devuelve una imagen negra.
        """
        self._require_started()

        try:
            if self._simulated:
                width, height = self.resolution
                frame = np.zeros(
                    (height, width, 3),
                    dtype=np.uint8,
                )

            else:
                job = self._picam2.capture_request(wait=False)
                request = job.get_result(
                    timeout=self.stale_frame_timeout_s
                )

                try:
                    metadata = request.get_metadata()
                    stamp = metadata.get("SensorTimestamp")

                    if (
                        isinstance(stamp, bool)
                        or not isinstance(stamp, (int, np.integer))
                        or stamp <= 0
                    ):
                        raise CameraFault(
                            "Fotograma sin SensorTimestamp válido"
                        )

                    if (
                        self._last_sensor_timestamp is not None
                        and stamp <= self._last_sensor_timestamp
                    ):
                        raise CameraFault(
                            "Fotograma repetido o fuera de orden"
                        )

                    # Copiar antes de liberar el buffer de la cámara.
                    frame = request.make_array("main").copy()

                finally:
                    request.release()

            if (
                not isinstance(frame, np.ndarray)
                or frame.dtype != np.uint8
                or frame.ndim != 3
                or frame.shape[2] != 3
                or frame.size == 0
            ):
                raise CameraFault(
                    "La cámara entregó un frame inválido"
                )

            if not self._simulated:
                self._last_sensor_timestamp = int(stamp)

            self._last_frame_ts = time.monotonic()
            return frame

        except Exception as exc:
            # Exigir reinicio evita acumular capturas tras un timeout.
            self._mark_fault()
            raise CameraFault(
                f"Captura inválida o timeout: {exc}"
            ) from exc

    def is_healthy(self) -> bool:
        """Indica si se obtuvo un frame recientemente y sin fallos."""
        return (
            self._started
            and not self._faulted
            and self._last_frame_ts is not None
            and (
                time.monotonic() - self._last_frame_ts
                < self.stale_frame_timeout_s
            )
        )

    def stop(self):
        """Detiene y libera la cámara."""
        cam = self._picam2

        self._picam2 = None
        self._started = False
        self._last_frame_ts = None
        self._last_sensor_timestamp = None
        self.analogue_gain = None

        if cam is not None:
            try:
                cam.stop()
            finally:
                cam.close()


def main():
    """
    Prueba independiente.

    Windows:
        python camera.py

    Raspberry con cámara física:
        python camera.py --real
    """
    parser = argparse.ArgumentParser(
        description="Prueba de captura simulada o real"
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="Usar la cámara física mediante Picamera2",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    camera = Camera(simulated=not args.real)

    try:
        camera.start()

        for index in range(3):
            frame = camera.capture_frame()

            print(
                f"Frame {index + 1}: "
                f"dimensiones={frame.shape}, "
                f"tipo={frame.dtype}, "
                f"saludable={camera.is_healthy()}"
            )

            # Solo pausa la prueba; no forma parte del controlador.
            time.sleep(0.1)

        if camera.simulated:
            print(
                "Prueba simulada completada. "
                "Las imágenes son negras; no se probó una cámara física."
            )
        else:
            print("Prueba de captura real completada.")

        return 0

    except CameraFault as exc:
        logger.error("%s", exc)
        return 1

    finally:
        camera.stop()


if __name__ == "__main__":
    raise SystemExit(main())