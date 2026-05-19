"""
hud.py — Overlay de depuração desenhado sobre o frame da câmera.
"""
import math
from typing import Dict, Optional

import cv2
import numpy as np

from config import FRAME_H
from Controller.grab_fsm import GrabState
from Controller.vision import ObjectTarget

COLOR_BGR_OVERLAY = {
    "Vermelho": (0, 0, 255), "Azul": (255, 0, 0),
    "Verde": (0, 200, 0),    "Amarelo": (0, 220, 220),
}


def draw(
    frame: np.ndarray,
    state: GrabState,
    joints: Dict[str, float],
    target_color: str,
    current_target: Optional[ObjectTarget],
    esp32_link: bool = False,
) -> np.ndarray:
    out = frame.copy()
    j = joints

    # Estado da FSM
    state_clr = (0, 255, 0) if state == GrabState.IDLE else (0, 200, 255)
    cv2.putText(out, f"Estado: {state.name}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_clr, 2)

    # Cor alvo
    tc = COLOR_BGR_OVERLAY.get(target_color, (200, 200, 200))
    cv2.putText(out, f"Alvo: {target_color}  [O=mudar | R1=capturar]",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tc, 2)

    # Link WiFi
    link_txt = "ESP32: OK" if esp32_link else "ESP32: SEM SINAL"
    link_clr = (0, 255, 80) if esp32_link else (0, 60, 255)
    cv2.putText(out, link_txt, (10, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, link_clr, 2)

    # Valores das juntas — agora com pulso1 e pulso2
    for i, line in enumerate([
        f"Base:{j['base']:+6.1f}  Ombro:{j['ombro']:+6.1f}",
        f"Cotov:{j['cotovelo']:+6.1f}  P1:{j['pulso1']:+6.1f}  P2:{j['pulso2']:+6.1f}",
        f"Garra:{j['garra']:5.1f}%",
    ]):
        cv2.putText(out, line, (10, FRAME_H - 70 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1)

    # Marcação do objeto detectado
    if current_target and state != GrabState.IDLE:
        cx, cy = current_target.pixel_cx, current_target.pixel_cy
        cv2.circle(out, (cx, cy), 14, (0, 255, 255), 2)
        cv2.drawMarker(out, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)

        ang = math.radians(current_target.approach_angle_deg)
        ex = int(cx + 45 * math.cos(ang))
        ey = int(cy + 45 * math.sin(ang))
        cv2.arrowedLine(out, (cx, cy), (ex, ey), (0, 200, 255), 2, tipLength=0.3)
        cv2.putText(out, f"pulso1: {current_target.approach_angle_deg:.1f}deg",
                    (cx + 15, cy - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    return out