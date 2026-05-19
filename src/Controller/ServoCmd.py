"""
ServoCmd — Comunicação WiFi UDP com o ESP32, envio assíncrono.

Mapeamento pino → servo → junta:
  pino 13 → servo 0 → base
  pino 14 → servo 1 → ombro
  pino 16 → servo 2 → cotovelo
  pino 17 → servo 3 → pulso1  (flexão vertical)
  pino 18 → servo 4 → pulso2  (inclinação lateral)
  pino 19 → servo 5 → garra   (0%=fechada, 100%=aberta)

Protocolo UDP:
  PC → ESP32 : 7 bytes  [0xAA] [s0 s1 s2 s3 s4 s5]  (ângulos 0-180)
  ESP32 → PC : 4 bytes  [0xAA] [status] [idx] [xor]
"""

import socket
import threading
import logging
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

from config import JOINT_LIMITS

logger = logging.getLogger("ServoCmd")

START_BYTE = 0xAA
ACK_LEN    = 4
ACK_OK     = 0x00
ACK_RANGE  = 0x01
ACK_BUSY   = 0x02

# Ordem exata = índice do servo no ESP32
JOINT_ORDER = ["base", "ombro", "cotovelo", "pulso1", "pulso2", "garra"]


def _logical_to_servo(joint: str, value: float) -> int:
    """Converte ângulo lógico → byte 0-180 para o servo."""
    if joint == "garra":
        # percentual 0-100 → 0-180°
        servo = int(round((value / 100.0) * 180.0))
    else:
        lo, hi = JOINT_LIMITS[joint]
        servo = int(round(((value - lo) / (hi - lo)) * 180.0))
    return max(0, min(180, servo))


def _build_packet(angles: List[int]) -> bytes:
    return bytes([START_BYTE] + angles[:6])


def _parse_ack(raw: bytes) -> Optional[dict]:
    if len(raw) < ACK_LEN or raw[0] != START_BYTE:
        return None
    status, idx, cs = raw[1], raw[2], raw[3]
    if cs != (raw[0] ^ raw[1] ^ raw[2]):
        logger.warning("ACK com checksum inválido.")
        return None
    return {"status": status, "servo_idx": idx}


class ServoCmd:
    """
    Envia comandos ao ESP32 via UDP em thread separada.
    send() é não-bloqueante — o loop principal nunca trava esperando ACK.
    """

    def __init__(
        self,
        esp32_ip: str    = "192.168.4.1",
        esp32_port: int  = 4210,
        local_port: int  = 4211,
        timeout_s: float = 0.15,
        max_retries: int = 2,
    ) -> None:
        self._ip      = esp32_ip
        self._port    = esp32_port
        self._timeout = timeout_s
        self._retries = max_retries

        self._queue: deque = deque(maxlen=1)
        self._stop  = threading.Event()

        self._tx_count   = 0
        self._ack_ok     = 0
        self._ack_err    = 0
        self._last_ok_ts = 0.0
        self._lock       = threading.Lock()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(self._timeout)
        try:
            self._sock.bind(("", local_port))
        except OSError as e:
            logger.warning(f"bind porta {local_port}: {e}")

        self._thread = threading.Thread(
            target=self._worker, daemon=True, name="ServoCmd-TX"
        )
        self._thread.start()
        logger.info(f"ServoCmd pronto — ESP32 {esp32_ip}:{esp32_port}")

    def send(self, joints: Dict[str, float]) -> None:
        """Não-bloqueante. Só o pacote mais recente é enviado."""
        angles = [
            _logical_to_servo(j, joints.get(j, 0.0 if j != "garra" else 50.0))
            for j in JOINT_ORDER
        ]
        self._queue.append((angles, _build_packet(angles)))

    def is_alive(self) -> bool:
        return (time.monotonic() - self._last_ok_ts) < 2.0

    def stats(self) -> Dict[str, int]:
        with self._lock:
            return {"tx": self._tx_count, "ack_ok": self._ack_ok, "ack_err": self._ack_err}

    def close(self) -> None:
        neutral = {j: 0.0 for j in JOINT_ORDER}
        neutral["garra"] = 50.0
        self.send(neutral)
        time.sleep(0.3)
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close()
        logger.info("ServoCmd encerrado.")

    def _worker(self) -> None:
        while not self._stop.is_set():
            if not self._queue:
                time.sleep(0.005)
                continue
            angles, packet = self._queue.pop()
            self._dispatch(packet, angles)

    def _dispatch(self, packet: bytes, angles: List[int]) -> None:
        for attempt in range(self._retries + 1):
            try:
                self._sock.sendto(packet, (self._ip, self._port))
                with self._lock:
                    self._tx_count += 1

                raw, _ = self._sock.recvfrom(ACK_LEN)
                ack = _parse_ack(raw)
                if ack is None:
                    raise ValueError("ACK malformado")

                if ack["status"] == ACK_OK:
                    with self._lock:
                        self._ack_ok += 1
                        self._last_ok_ts = time.monotonic()
                    return

                if ack["status"] == ACK_RANGE:
                    logger.warning(
                        f"RANGE_ERROR servo {ack['servo_idx']} "
                        f"(val={angles[ack['servo_idx']]})"
                    )
                    with self._lock:
                        self._ack_err += 1
                    return

                if ack["status"] == ACK_BUSY:
                    time.sleep(0.02)
                    continue

            except (socket.timeout, OSError) as e:
                if attempt == self._retries:
                    logger.warning(f"ESP32 sem resposta após {self._retries + 1} tentativas.")
                    with self._lock:
                        self._ack_err += 1