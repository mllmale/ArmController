import pygame as pg
import sys
from game_pad.joystick import RemoteControl
from utils.pid import PIDController


def main():
    pg.init()
    largura, altura = 800, 600
    tela = pg.display.set_mode((largura, altura))
    pg.display.set_caption("SITL: Teste de PID com Joystick")
    relogio = pg.time.Clock()

    controle = RemoteControl()

    pid_x = PIDController(kp=5.0, ki=12, kd=0.5, min_out=-500, max_out=500)
    pid_y = PIDController(kp=5.0, ki=12, kd=0.5, min_out=-500, max_out=500)

    setpoint_x, setpoint_y = largura / 2, altura / 2
    velocidade_alvo = 400

    pv_x, pv_y = largura / 2, altura / 2

    fonte = pg.font.SysFont("monospace", 16, bold=True)

    while True:
        dt = relogio.tick(60) / 1000.0

        for evento in pg.event.get():
            if evento.type == pg.QUIT:
                pg.quit()
                sys.exit()

        dados = controle.get_values()

        if 'conectado' in dados['status']:
            joy_x, joy_y = dados['l_stick']

            if abs(joy_x) < 0.1: joy_x = 0
            if abs(joy_y) < 0.1: joy_y = 0

            setpoint_x += joy_x * velocidade_alvo * dt
            setpoint_y += joy_y * velocidade_alvo * dt

            setpoint_x = max(0, min(setpoint_x, largura))
            setpoint_y = max(0, min(setpoint_y, altura))

        sinal_controle_x = pid_x.compute(setpoint_x, pv_x, dt)
        sinal_controle_y = pid_y.compute(setpoint_y, pv_y, dt)

        pv_x += sinal_controle_x * dt
        pv_y += sinal_controle_y * dt

        tela.fill((20, 20, 25))

        pg.draw.circle(tela, (100, 100, 100), (int(setpoint_x), int(setpoint_y)), 15, 2)

        pg.draw.circle(tela, (0, 255, 100), (int(pv_x), int(pv_y)), 15)

        pg.draw.line(tela, (255, 50, 50), (int(pv_x), int(pv_y)), (int(setpoint_x), int(setpoint_y)), 2)

        textos = [
            f"Alvo X: {setpoint_x:.1f} | Alvo Y: {setpoint_y:.1f}",
            f"Robo X: {pv_x:.1f} | Robo Y: {pv_y:.1f}",
            f"Erro X: {(setpoint_x - pv_x):.1f} | Erro Y: {(setpoint_y - pv_y):.1f}",
            "Use o L-Stick para mover o alvo."
        ]

        for i, texto in enumerate(textos):
            superficie = fonte.render(texto, True, (200, 200, 200))
            tela.blit(superficie, (10, 10 + (i * 25)))

        pg.display.flip()


if __name__ == "__main__":
    main()