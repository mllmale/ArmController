import pygame as pg


class RemoteControl:
    def __init__(self):
        pg.init()
        pg.joystick.init()
        self.controller = None
        self._connect_joystick()

    def _connect_joystick(self):
        if pg.joystick.get_count() > 0:
            self.controller = pg.joystick.Joystick(0)
            self.controller.init()
            return True
        return False

    def get_values(self):
        pg.event.pump()

        if not self.controller:
            if not self._connect_joystick():
                return {
                    "status": "desconectado",
                    "l_stick": (0.0, 0.0),
                    "r_stick": (0.0, 0.0),
                    "triggers": {"l2": 0.0, "r2": 0.0},
                    "buttons": {},
                    "dpad": (0, 0)
                }

        try:
            l2_raw = round(self.controller.get_axis(2), 2)
            r2_raw = round(self.controller.get_axis(5), 2)

            return {
                "status": "conectado",
                "l_stick": (
                    round(self.controller.get_axis(0), 2),
                    round(self.controller.get_axis(1), 2)
                ),
                "r_stick": (
                    round(self.controller.get_axis(3), 2),
                    round(self.controller.get_axis(4), 2)
                ),
                "triggers": {
                    "l2": l2_raw,
                    "r2": r2_raw
                },
                "buttons": {
                    "x": self.controller.get_button(0),
                    "circle": self.controller.get_button(1),
                    "triangle": self.controller.get_button(2),
                    "square": self.controller.get_button(3),
                    "l1": self.controller.get_button(4),
                    "r1": self.controller.get_button(5),
                    "share": self.controller.get_button(8),
                    "options": self.controller.get_button(9),
                    "ps": self.controller.get_button(10),
                    "l3": self.controller.get_button(11),
                    "r3": self.controller.get_button(12)
                },
                "dpad": self.controller.get_hat(0)
            }
        except pg.error as e:
            self.controller = None
            return {"status": "erro_conexao"}

    def get_info(self):
        if self.controller:
            return {
                "nome": self.controller.get_name(),
                "bateria": self.controller.get_power_level(),
                "id": self.controller.get_instance_id(),
                "eixos": self.controller.get_numaxes(),
                "botoes": self.controller.get_numbuttons()
            }
        return {"status": "Controle não encontrado"}
