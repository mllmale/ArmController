"""
VisionSystem com leitura de câmera em thread separada.

O loop principal nunca espera um frame — ele pega o último disponível
e continua. O ColorReco processa numa thread própria a ~15 fps,
sem interferir no ciclo de controle dos servos (30 fps).
"""
import math
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from Model.ColorReco import ColorReco
from utils.limits import get_limits
from config import (FRAME_W, FRAME_H, CAM_FX, CAM_FY,
                    CAM_HEIGHT_M, CAM_OFFSET_X, TABLE_HEIGHT_M, JOINT_LIMITS)

logger = logging.getLogger("VisionSystem")

COLOR_BGR: dict = {
    "Amarelo":  [0,   255, 255],
    "Azul":     [255, 0,   0  ],
    "Verde":    [0,   255, 0  ],
    "Vermelho": None,
}


@dataclass
class ObjectTarget:
    color_name: str
    pixel_cx: int
    pixel_cy: int
    world_x: float
    world_y: float
    world_z: float
    approach_angle_deg: float
    contour: np.ndarray = field(default=None, repr=False)


class VisionSystem:
    """
    Câmera e ColorReco rodam numa thread de baixa prioridade (~15 fps).
    O loop principal consome o último frame anotado sem bloquear.
    """

    def __init__(self, camera_index: int, process_fps: float = 15.0) -> None:
        self.color_reco = ColorReco()
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self._cx = FRAME_W / 2.0
        self._cy = FRAME_H / 2.0

        self._lock            = threading.Lock()
        self._raw_frame       = None   # frame cru mais recente
        self._annotated_frame = None   # frame com caixas coloridas
        self._stop            = threading.Event()
        self._interval        = 1.0 / process_fps

        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="VisionCapture"
        )
        self._thread.start()
        logger.info(f"Câmera {camera_index} iniciada — processando a {process_fps:.0f} fps")

    # ── API pública ─────────────────────────────────────────────────────────

    def read_frame(self) -> Optional[np.ndarray]:
        """Frame cru — não bloqueante."""
        with self._lock:
            return self._raw_frame.copy() if self._raw_frame is not None else None

    def read_annotated(self) -> Optional[np.ndarray]:
        """Frame com caixas coloridas do ColorReco — não bloqueante."""
        with self._lock:
            return self._annotated_frame.copy() if self._annotated_frame is not None else None

    def detect(self, frame: np.ndarray, target_color: str) -> Optional[ObjectTarget]:
        """Detecta objeto da cor alvo e retorna posição 3D + ângulo do pulso."""
        _, centers = self.color_reco.process_image(frame.copy())
        hits = [(cx, cy) for name, cx, cy in centers if name == target_color]
        if not hits:
            return None

        px, py = min(hits, key=lambda p: math.hypot(p[0] - self._cx, p[1] - self._cy))
        wx, wy, wz = self._pixel_to_world(px, py)
        angle = self._best_wrist_angle(frame, px, py, target_color)
        return ObjectTarget(target_color, px, py, wx, wy, wz, angle)

    def release(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.cap.release()

    # ── Thread de captura ────────────────────────────────────────────────────

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()

            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            annotated, _ = self.color_reco.process_image(frame.copy())

            with self._lock:
                self._raw_frame       = frame
                self._annotated_frame = annotated

            # Limita ao process_fps sem travar
            elapsed = time.monotonic() - t0
            sleep_t = self._interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── Cálculos ─────────────────────────────────────────────────────────────

    def _pixel_to_world(self, px: int, py: int) -> Tuple[float, float, float]:
        x = (px - self._cx) * (CAM_HEIGHT_M / CAM_FX)
        y = (py - self._cy) * (CAM_HEIGHT_M / CAM_FY)
        return y + CAM_OFFSET_X, x, float(TABLE_HEIGHT_M)

    def _best_wrist_angle(self, frame: np.ndarray, px: int, py: int,
                          target_color: str) -> float:
        roi_size = 80
        roi = frame[max(0, py-roi_size):min(FRAME_H, py+roi_size),
                    max(0, px-roi_size):min(FRAME_W, px+roi_size)]
        if roi.size == 0:
            return 0.0

        mask = self._color_mask(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), target_color)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200:
            return 0.0

        (_, _), (w_box, h_box), rect_angle = cv2.minAreaRect(largest)
        object_angle = rect_angle + 90.0 if w_box < h_box else rect_angle

        lo, hi = JOINT_LIMITS["pulso1"]
        wrist_angle = max(lo, min(hi, object_angle))
        logger.debug(f"Orientação objeto: {object_angle:.1f}° → pulso1: {wrist_angle:.1f}°")
        return wrist_angle

    def _color_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        if color == "Vermelho":
            m1 = cv2.inRange(hsv, np.array([0,   100, 100]), np.array([10,  255, 255]))
            m2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([179, 255, 255]))
            return m1 | m2
        lo, hi = get_limits(COLOR_BGR[color])
        return cv2.inRange(hsv, lo, hi)