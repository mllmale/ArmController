import math
import logging
from typing import Dict, Optional

from config import L1, L2, L3, JOINT_LIMITS

logger = logging.getLogger("RobotController")


def solve_ik(world_x: float, world_y: float, world_z: float,
             approach_angle_deg: float) -> Optional[Dict[str, float]]:
    """
    Cinemática inversa para braço planar de 3 links (L1, L2, L3).

    Referencial: base do robô na origem, X = frente, Y = lateral, Z = cima.

    Passos:
      1. base     = atan2(Y, X)
      2. Desconta L3 na direção do ângulo de aproximação → ponto intermediário P2.
      3. Ombro + Cotovelo via lei dos cossenos para P2 (links L1, L2).
         Usa configuração cotovelo-para-cima (mais segura sobre a mesa).
      4. pulso1   = approach_angle − (ombro + cotovelo)  [mantém orientação]
      5. pulso2   = 0.0  (inclinação lateral — controlada manualmente pelo RX)

    Retorna dict com ângulos em graus, ou None se fora do espaço de trabalho.
    """
    base_deg = math.degrees(math.atan2(world_y, world_x))

    app_rad = math.radians(approach_angle_deg)
    r    = math.hypot(world_x, world_y) - L3 * math.cos(app_rad)
    h    = world_z                       - L3 * math.sin(app_rad)
    dist = math.hypot(r, h)

    if dist > (L1 + L2) * 0.98:
        logger.warning(f"IK: alvo fora do alcance (dist={dist:.3f}m, max={(L1+L2):.3f}m)")
        return None
    if dist < abs(L1 - L2) * 1.02:
        logger.warning(f"IK: alvo muito próximo (dist={dist:.3f}m)")
        return None

    cos_e = (dist**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_e = max(-1.0, min(1.0, cos_e))
    elbow_rad    = -math.acos(cos_e)
    cotovelo_deg = math.degrees(elbow_rad)

    alpha     = math.atan2(h, r)
    beta      = math.atan2(L2 * math.sin(-elbow_rad), L1 + L2 * math.cos(-elbow_rad))
    ombro_deg = math.degrees(alpha + beta)

    # pulso1 mantém a orientação do efetuador
    # pulso2 fica em 0 — a IK não controla inclinação lateral
    pulso1_deg = approach_angle_deg - (ombro_deg + cotovelo_deg)
    pulso2_deg = 0.0

    result = {
        "base":     base_deg,
        "ombro":    ombro_deg,
        "cotovelo": cotovelo_deg,
        "pulso1":   pulso1_deg,
        "pulso2":   pulso2_deg,
    }

    for joint, angle in result.items():
        lo, hi = JOINT_LIMITS[joint]
        if not (lo <= angle <= hi):
            logger.warning(f"IK: '{joint}'={angle:.1f}° fora de [{lo}, {hi}]")
            return None

    logger.info(f"IK resolvida: { {k: f'{v:.1f}°' for k, v in result.items()} }")
    return result