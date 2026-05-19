"""
Estados:
  IDLE → DETECTING → CALCULATING → MOVING → CLOSING → RETURNING → IDLE
"""
import logging
from enum import Enum, auto
from typing import Dict, Optional

import numpy as np

from config import JOINT_LIMITS, JOINT_SPEEDS, JOINT_TOL_DEG, HOME_POSITION, PID_GAINS
from Controller.kinematics import solve_ik
from Controller.vision import VisionSystem, ObjectTarget
from utils.pid import PIDController

logger = logging.getLogger("RobotController")


class GrabState(Enum):
    IDLE        = auto()
    DETECTING   = auto()
    CALCULATING = auto()
    MOVING      = auto()
    CLOSING     = auto()
    RETURNING   = auto()


class GrabFSM:
    """
    Máquina de estados que controla o ciclo completo de captura autônoma.
    Recebe/devolve o dict de juntas a cada tick — não acessa hardware diretamente.
    """

    def __init__(self, vision: VisionSystem, target_color: str) -> None:
        self.vision        = vision
        self.target_color  = target_color
        self.state         = GrabState.IDLE
        self._target: Optional[ObjectTarget] = None
        self._setpoints: Dict[str, float]    = {}
        self._pids: Dict[str, PIDController] = {
            j: PIDController(*gains, min_out=-JOINT_SPEEDS[j], max_out=+JOINT_SPEEDS[j])
            for j, gains in PID_GAINS.items()
        }


    def start(self) -> None:
        if self.state == GrabState.IDLE:
            self._transition(GrabState.DETECTING)

    def cancel(self) -> None:
        logger.info("Captura cancelada.")
        self.state = GrabState.IDLE

    def tick(self, joints: Dict[str, float], frame: Optional[np.ndarray],
             dt: float) -> Dict[str, float]:
        """
        Executa um ciclo da FSM e retorna o dict de juntas atualizado.
        Chamado a cada iteração do loop principal.
        """
        if self.state == GrabState.DETECTING:
            self._detecting(joints, frame)

        elif self.state == GrabState.CALCULATING:
            self._calculating(joints)

        elif self.state == GrabState.MOVING:
            self._moving(joints, dt)

        elif self.state == GrabState.CLOSING:
            self._closing(joints, dt)

        elif self.state == GrabState.RETURNING:
            self._returning(joints, dt)

        return joints


    def _detecting(self, joints: Dict, frame: Optional[np.ndarray]) -> None:
        if frame is None:
            return
        target = self.vision.detect(frame, self.target_color)
        if target:
            logger.info(
                f"'{target.color_name}' em pixel=({target.pixel_cx},{target.pixel_cy}) | "
                f"mundo=({target.world_x:.3f},{target.world_y:.3f},{target.world_z:.3f})m | "
                f"pulso_alvo={target.approach_angle_deg:.1f}°"
            )
            self._target = target
            self._transition(GrabState.CALCULATING)

    def _calculating(self, joints: Dict) -> None:
        ik = solve_ik(self._target.world_x, self._target.world_y,
                      self._target.world_z, self._target.approach_angle_deg)
        if ik is None:
            logger.error("IK falhou — abortando.")
            self.state = GrabState.IDLE
            return
        self._setpoints = {**ik, "garra": 80.0}
        for pid in self._pids.values():
            pid.reset()
        self._transition(GrabState.MOVING)

    def _moving(self, joints: Dict, dt: float) -> None:
        all_arrived = True
        for joint, sp in self._setpoints.items():
            if joint == "garra":
                joints["garra"] = sp
                continue
            error = sp - joints[joint]
            if abs(error) > JOINT_TOL_DEG:
                all_arrived = False
                joints[joint] += self._pids[joint].compute(sp, joints[joint], dt) * dt
            joints[joint] = _clamp(joints[joint], *JOINT_LIMITS[joint])
        if all_arrived:
            logger.info("Pré-grasp atingido. Fechando garra...")
            self._transition(GrabState.CLOSING)

    def _closing(self, joints: Dict, dt: float) -> None:
        joints["garra"] -= JOINT_SPEEDS["garra"] * dt
        joints["garra"] = max(JOINT_LIMITS["garra"][0], joints["garra"])
        if joints["garra"] <= 5.0:
            logger.info("Garra fechada. Retornando ao HOME.")
            self._transition(GrabState.RETURNING)

    def _returning(self, joints: Dict, dt: float) -> None:
        all_arrived = True
        for joint, target_val in HOME_POSITION.items():
            if joint == "garra":
                continue
            error = target_val - joints[joint]
            if abs(error) > JOINT_TOL_DEG:
                all_arrived = False
                joints[joint] += self._pids[joint].compute(target_val, joints[joint], dt) * dt
            joints[joint] = _clamp(joints[joint], *JOINT_LIMITS[joint])
        if all_arrived:
            logger.info("HOME atingido. Missão concluída!")
            self._transition(GrabState.IDLE)

    def _transition(self, new_state: GrabState) -> None:
        logger.info(f"[FSM] {self.state.name} → {new_state.name}")
        self.state = new_state


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))