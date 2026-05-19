"""
Mapeamento PS4 → juntas:
┌─────────────────────────────────────────────────────────────┐
│  Analógico E  LX → BASE          (rotação horizontal)       │
│  Analógico E  LY → OMBRO         (elevação do braço)        │
│  Analógico D  RY → COTOVELO      (flexão do antebraço)      │
│  Analógico D  RX → PULSO2        (inclinação lateral)       │
│  D-Pad  ↑↓      → PULSO1         (flexão vertical)          │
│  D-Pad  ←→      → BASE fino      (ajuste fino)              │
│  L2 / R2        → Fecha / Abre GARRA                        │
├─────────────────────────────────────────────────────────────┤
│  ✕  → HOME                                                  │
│  △  → Salva posição                                         │
│  □  → Restaura posição salva                                │
│  OPTIONS → Reinicia juntas                                  │
├─────────────────────────────────────────────────────────────┤
│  R1      → Iniciar / Cancelar captura autônoma              │
│  ○       → Ciclar cor alvo                                  │
│  L1      → Mostrar/ocultar câmera                           │
└─────────────────────────────────────────────────────────────┘
"""
import sys
import time
import logging
from typing import Dict, Any, Optional

import cv2

from game_pad.joystick import RemoteControl
from config import (
    JOINT_LIMITS, JOINT_SPEEDS, HOME_POSITION,
    DEADZONE, FINE_TUNE_SPEED, CAMERA_INDEX, TARGET_COLORS,
    ESP32_IP, ESP32_PORT, LOCAL_PORT, WIFI_TIMEOUT_S, WIFI_RETRIES,
)
from Controller.vision import VisionSystem
from Controller.grab_fsm import GrabFSM, GrabState
from Controller.ServoCmd import ServoCmd
from Controller import hud

logger = logging.getLogger("RobotController")
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")


class RobotArmController(RemoteControl):
    def __init__(self, loop_rate_hz: float = 30.0) -> None:
        super().__init__()
        self.dt  = 1.0 / loop_rate_hz
        self._hz = loop_rate_hz

        self.joints: Dict[str, float]       = dict(HOME_POSITION)
        self.saved_position: Optional[Dict] = None
        self._prev_buttons: Dict[str, int]  = {}
        self._show_camera                   = True

        self._color_idx = 0

        # Câmera — tenta índices 0, 2, 1 automaticamente
        self._vision = self._init_camera()
        self._fsm    = GrabFSM(self._vision, TARGET_COLORS[0])

        self._servo = ServoCmd(
            esp32_ip    = ESP32_IP,
            esp32_port  = ESP32_PORT,
            local_port  = LOCAL_PORT,
            timeout_s   = WIFI_TIMEOUT_S,
            max_retries = WIFI_RETRIES,
        )

        logger.info(f"Pronto. Juntas: {list(self.joints.keys())}")
        logger.info("Controle: LX=base | LY=ombro | RY=cotovelo | RX=pulso2 | DPad↑↓=pulso1 | L2/R2=garra")

    # ── Inicialização da câmera ─────────────────────────────────────────────

    def _init_camera(self) -> Optional[VisionSystem]:
        for idx in [CAMERA_INDEX, 0, 2, 1]:
            try:
                vs = VisionSystem(idx)
                # Aguarda a thread de captura processar o primeiro frame
                for _ in range(20):
                    time.sleep(0.05)
                    if vs.read_frame() is not None:
                        logger.info(f"Câmera encontrada no índice {idx}.")
                        return vs
                vs.release()
            except Exception:
                continue
        logger.warning("Nenhuma câmera encontrada — modo sem visão.")
        return None

    # ── Loop principal ──────────────────────────────────────────────────────

    def run(self) -> None:
        logger.info("Loop iniciado. Ctrl+C para sair.")
        try:
            while self._step():
                time.sleep(self.dt)
        except KeyboardInterrupt:
            logger.info("Encerrado pelo usuário.")
        finally:
            self._servo.close()
            if self._vision:
                self._vision.release()
            cv2.destroyAllWindows()
            st = self._servo.stats()
            logger.info(
                f"WiFi → TX={st['tx']} | ACK_OK={st['ack_ok']} | ACK_ERR={st['ack_err']}"
            )

    def _step(self) -> bool:
        data    = self.get_values()
        buttons = data.get("buttons", {})

        self._handle_mode_buttons(buttons)

        # read_annotated() já desenha as caixas de cor no frame
        frame = self._vision.read_annotated() if self._vision else None

        if self._fsm is None or self._fsm.state == GrabState.IDLE:
            if "conectado" in data.get("status", ""):
                self._manual(data)
        else:
            if frame is not None:
                self._fsm.tick(self.joints, frame, self.dt)

        self._clamp_all()
        self._prev_buttons = dict(buttons)

        # Envia ao ESP32 (não-bloqueante)
        self._servo.send(self.joints)

        # Câmera / HUD
        if frame is not None and self._show_camera:
            if self._fsm is not None:
                overlay = hud.draw(
                    frame, self._fsm.state, self.joints,
                    self._fsm.target_color, self._fsm._target,
                    esp32_link=self._servo.is_alive(),
                )
            else:
                overlay = frame  # sem FSM: mostra frame com anotações de cor
            cv2.imshow("Visao do Robo", overlay)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return False

        return True

    # ── Controle manual ─────────────────────────────────────────────────────

    def _manual(self, data: Dict[str, Any]) -> None:
        lx, ly = [_dz(v) for v in data["l_stick"]]
        rx, ry = [_dz(v) for v in data["r_stick"]]
        l2, r2 = data["triggers"]["l2"], data["triggers"]["r2"]
        dx, dy = data["dpad"]
        btn    = data["buttons"]

        # Analógicos
        self.joints["base"]     += lx * JOINT_SPEEDS["base"]     * self.dt
        self.joints["ombro"]    -= ly * JOINT_SPEEDS["ombro"]     * self.dt
        self.joints["cotovelo"] -= ry * JOINT_SPEEDS["cotovelo"]  * self.dt
        self.joints["pulso2"]   += rx * JOINT_SPEEDS["pulso2"]    * self.dt

        # D-Pad: pulso1 (flexão vertical) e ajuste fino de base
        self.joints["pulso1"]   += dy * FINE_TUNE_SPEED           * self.dt
        self.joints["base"]     += dx * FINE_TUNE_SPEED           * self.dt

        # Gatilhos: garra
        self.joints["garra"]    += (_trig(r2) - _trig(l2)) * JOINT_SPEEDS["garra"] * self.dt

        # Botões de ação
        if self._pressed(btn, "X"):
            self.joints = dict(HOME_POSITION)
            logger.info("HOME")

        if self._pressed(btn, "TRIANGULO"):
            self.saved_position = dict(self.joints)
            logger.info("Posição salva.")

        if self._pressed(btn, "QUADRADO") and self.saved_position:
            self.joints = dict(self.saved_position)
            logger.info("Posição restaurada.")

        if self._pressed(btn, "OPTIONS"):
            self.joints = dict(HOME_POSITION)
            logger.info("Reiniciado.")

    # ── Botões de modo ──────────────────────────────────────────────────────

    def _handle_mode_buttons(self, btn: Dict[str, int]) -> None:
        if self._pressed(btn, "R1") and self._fsm:
            if self._fsm.state == GrabState.IDLE:
                self._fsm.start()
                logger.info("Captura autônoma iniciada.")
            else:
                self._fsm.cancel()
                logger.info("Captura cancelada.")

        if self._pressed(btn, "CIRCULO"):
            self._color_idx = (self._color_idx + 1) % len(TARGET_COLORS)
            if self._fsm:
                self._fsm.target_color = TARGET_COLORS[self._color_idx]
            logger.info(f"Cor alvo: {TARGET_COLORS[self._color_idx]}")

        if self._pressed(btn, "L1"):
            self._show_camera = not self._show_camera
            if not self._show_camera:
                cv2.destroyWindow("Visao do Robo")

    # ── Utilitários ─────────────────────────────────────────────────────────

    def _pressed(self, btn: Dict[str, int], key: str) -> bool:
        return btn.get(key, 0) == 1 and self._prev_buttons.get(key, 0) == 0

    def _clamp_all(self) -> None:
        for joint, (lo, hi) in JOINT_LIMITS.items():
            if joint in self.joints:
                self.joints[joint] = max(lo, min(hi, self.joints[joint]))


def _dz(v: float) -> float:
    return 0.0 if abs(v) < DEADZONE else v


def _trig(v: float) -> float:
    """Gatilho PS4: repouso=-1.0, pressionado=+1.0 → converte para 0.0~1.0"""
    return (v + 1.0) / 2.0


if __name__ == "__main__":
    RobotArmController().run()