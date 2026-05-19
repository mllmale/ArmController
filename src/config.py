from typing import Dict, Tuple

# Comprimentos dos segmentos (metros)
L1 = 0.15   # base → ombro
L2 = 0.15   # ombro → cotovelo
L3 = 0.10   # cotovelo → pulso

# ── Mapeamento de juntas (ordem = índice do servo no ESP32) ─────────────────
# pino 13 → servo 0 → base
# pino 14 → servo 1 → ombro
# pino 16 → servo 2 → cotovelo
# pino 17 → servo 3 → pulso1  (flexão vertical)
# pino 18 → servo 4 → pulso2  (inclinação lateral)
# pino 19 → servo 5 → garra

JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "base":     (-180.0, 180.0),  # rotação horizontal da base
    "ombro":    ( -90.0,  90.0),  # elevação do braço
    "cotovelo": (-135.0, 135.0),  # flexão do antebraço
    "pulso1":   ( -90.0,  90.0),  # flexão vertical do pulso
    "pulso2":   ( -45.0,  45.0),  # inclinação lateral do pulso
    "garra":    (   0.0, 100.0),  # percentual de abertura
}

JOINT_SPEEDS: Dict[str, float] = {
    "base":     60.0,   # graus/s
    "ombro":    45.0,
    "cotovelo": 45.0,
    "pulso1":   70.0,
    "pulso2":   70.0,
    "garra":    80.0,
}

PID_GAINS: Dict[str, Tuple[float, float, float]] = {
    "base":     (0.8, 0.01, 0.05),
    "ombro":    (0.7, 0.01, 0.04),
    "cotovelo": (0.7, 0.01, 0.04),
    "pulso1":   (1.0, 0.02, 0.03),
    "pulso2":   (1.0, 0.02, 0.03),
}

HOME_POSITION: Dict[str, float] = {
    "base":     0.0,
    "ombro":    0.0,
    "cotovelo": 0.0,
    "pulso1":   0.0,
    "pulso2":   0.0,
    "garra":   80.0,
}

# ── Câmera ──────────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0       # tente 0, 2 ou -1 se não abrir
FRAME_W       = 640
FRAME_H       = 480
CAM_FX        = 600.0
CAM_FY        = 600.0
CAM_HEIGHT_M  = 0.40
CAM_OFFSET_X  = 0.25

# ── Controle ────────────────────────────────────────────────────────────────
TABLE_HEIGHT_M  = -0.05
JOINT_TOL_DEG   = 1.5
DEADZONE        = 0.08
FINE_TUNE_SPEED = 15.0
TARGET_COLORS   = ["Vermelho", "Azul", "Verde", "Amarelo"]

# ── ESP32 / WiFi ─────────────────────────────────────────────────────────────
ESP32_IP       = "192.168.4.1"
ESP32_PORT     = 4210
LOCAL_PORT     = 4211
WIFI_TIMEOUT_S = 0.15
WIFI_RETRIES   = 2