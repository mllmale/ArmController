from game_pad.joystick import RemoteControl
import pygame as pg
import sys


def main():
    pg.init()

    tela = pg.display.set_mode((400, 550))
    pg.display.set_caption("Painel de Teste - Braço Robótico")

    fonte = pg.font.SysFont("monospace", 18, bold=True)
    relogio = pg.time.Clock()

    controle = RemoteControl()

    while True:
        for evento in pg.event.get():
            if evento.type == pg.QUIT:
                pg.quit()
                sys.exit()

        dados = controle.get_values()

        tela.fill((15, 15, 15))

        if dados['status'] == 'conectado':
            cor_texto = (0, 255, 100)

            textos = [
                f"STATUS: {dados['status'].upper()}",
                "-" * 30,
                f"L-Stick X : {dados['l_stick'][0]:>5}",
                f"L-Stick Y : {dados['l_stick'][1]:>5}",
                "-" * 30,
                f"R-Stick X : {dados['r_stick'][0]:>5}",
                f"R-Stick Y : {dados['r_stick'][1]:>5}",
                "-" * 30,
                f"Gatilho L2: {dados['triggers']['l2']:>5}",
                f"Gatilho R2: {dados['triggers']['r2']:>5}",
                "-" * 30,
                f"D-Pad     : {dados['dpad']}",
                "-" * 30,
                "Botões Pressionados:"
            ]

            botoes_ativos = [nome for nome, estado in dados['buttons'].items() if estado == 1]
            if botoes_ativos:
                textos.append(" ".join(botoes_ativos))
            else:
                textos.append("NENHUM")

        else:
            cor_texto = (255, 50, 50)
            textos = [
                "STATUS: DESCONECTADO",
                "",
                "Por favor, conecte o",
                "controle via USB ou",
                "Bluetooth."
            ]

        y_pos = 20
        for linha in textos:
            superficie = fonte.render(linha, True, cor_texto)
            tela.blit(superficie, (20, y_pos))
            y_pos += 30

        pg.display.flip()
        relogio.tick(30)


if __name__ == "__main__":
    main()