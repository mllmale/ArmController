import sys
import logging
import pygame as pg
from typing import Dict, Any, Optional

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] - %(message)s')
logger = logging.getLogger("RobotController")


class RemoteControl:
    """
    Gerenciador de Joystick com Abstração de Hardware (HAL).
    Suporta mapeamento dinâmico para controles Xbox e PS4 em Linux e Windows.
    """

    PROFILES: Dict[str, Dict[str, Any]] = {
        'linux_xbox': {
            'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'ry': 4, 'l2': 2, 'r2': 5},
            'botoes': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7}
        },
        'linux_ps4': {
            'eixos': {'lx': 0, 'ly': 1, 'rx': 3, 'ry': 4, 'l2': 2, 'r2': 5},
            'botoes': {'X': 0, 'CIRCULO': 1, 'TRIANGULO': 2, 'QUADRADO': 3, 'L1': 4, 'R1': 5, 'L2_BTN': 6, 'R2_BTN': 7,
                       'SHARE': 8, 'OPTIONS': 9, 'PS': 10, 'L3': 11, 'R3': 12}
        },
        'win32_xbox': {
            'eixos': {'lx': 0, 'ly': 1, 'rx': 4, 'ry': 3, 'gatilho_duplo': 2},
            'botoes': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7}
        },
        'win32_ps4': {
            'eixos': {'lx': 0, 'ly': 1, 'rx': 2, 'ry': 3, 'l2': 4, 'r2': 5},
            'botoes': {'QUADRADO': 0, 'X': 1, 'CIRCULO': 2, 'TRIANGULO': 3, 'L1': 4, 'R1': 5, 'SHARE': 8, 'OPTIONS': 9}
        }
    }

    def __init__(self) -> None:
        pg.init()
        pg.joystick.init()
        self.controller: Optional[pg.joystick.Joystick] = None
        self.os_name: str = sys.platform
        self.hardware_type: str = "desconhecido"
        self.profile: Dict[str, Any] = {}
        self._connect_joystick()

    def _detect_profile(self) -> None:
        """Detecta o SO e o nome do hardware gerando a chave de perfil correta."""
        nome_controle = self.controller.get_name().lower()
        self.hardware_type = 'xbox' if 'xbox' in nome_controle else 'ps4'

        profile_key = f"{self.os_name}_{self.hardware_type}"

        self.profile = self.PROFILES.get(profile_key, self.PROFILES['linux_ps4'])

        logger.info(f"Hardware ativado: '{nome_controle}' | Perfil carregado: {profile_key}")

    def _connect_joystick(self) -> bool:
        """Tenta inicializar o primeiro joystick plugado."""
        if pg.joystick.get_count() > 0:
            self.controller = pg.joystick.Joystick(0)
            self.controller.init()
            self._detect_profile()
            return True
        return False

    def _get_default_state(self, status: str) -> Dict[str, Any]:
        """Evita repetição de código (DRY) ao retornar estados zerados."""
        return {
            "status": status,
            "l_stick": (0.0, 0.0), "r_stick": (0.0, 0.0),
            "triggers": {"l2": -1.0, "r2": -1.0},
            "buttons": {}, "dpad": (0, 0)
        }

    def _read_triggers(self, p: Dict[str, int]) -> Dict[str, float]:
        """Isola a lógica complexa de leitura dos gatilhos."""
        if self.os_name == 'win32' and self.hardware_type == 'xbox':
            eixo = self.controller.get_axis(p['gatilho_duplo'])
            return {
                'l2': round(eixo, 2) if eixo > 0 else -1.0,
                'r2': round(abs(eixo), 2) if eixo < 0 else -1.0
            }
        return {
            'l2': round(self.controller.get_axis(p['l2']), 2),
            'r2': round(self.controller.get_axis(p['r2']), 2)
        }

    def get_values(self) -> Dict[str, Any]:
        """Busca e mapeia os valores do controle com base no perfil ativo."""
        pg.event.pump()

        if not self.controller and not self._connect_joystick():
            return self._get_default_state("desconectado")

        try:
            p = self.profile['eixos']

            botoes_lidos = {
                nome: self.controller.get_button(id_botao)
                for nome, id_botao in self.profile['botoes'].items()
                if id_botao < self.controller.get_numbuttons()
            }

            dados = {
                "status": f"conectado ({self.hardware_type.upper()} no {self.os_name.upper()})",
                "l_stick": (
                    round(self.controller.get_axis(p['lx']), 2),
                    round(self.controller.get_axis(p['ly']), 2)
                ),
                "r_stick": (
                    round(self.controller.get_axis(p['rx']), 2),
                    round(self.controller.get_axis(p['ry']), 2)
                ),
                "triggers": self._read_triggers(p),
                "buttons": botoes_lidos,
                "dpad": self.controller.get_hat(0) if self.controller.get_numhats() > 0 else (0, 0)
            }
            return dados

        except pg.error as e:
            logger.error(f"Falha de hardware/desconexão abrupta: {e}")
            self.controller = None
            return self._get_default_state("erro_conexao")