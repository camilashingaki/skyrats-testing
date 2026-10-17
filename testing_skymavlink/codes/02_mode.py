# troca de modo varias vezes no chão (stabilize, loiter, guided, brake...), sem armar e sem voar

"""02 — set_mode(), pause() e loiter(). NÃO arma, NÃO voa. Só troca de modo no chão.

Confere no Mission Planner / QGC se o modo muda junto.
"""

import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL

# Números de custom_mode do ArduCopter (skymavlink/flight.py: _ACM_MODE)
#   STABILIZE 0 | ACRO 1 | ALT_HOLD 2 | AUTO 3 | GUIDED 4 | LOITER 5 | RTL 6
#   CIRCLE 7 | LAND 9 | SPORT 13 | POSHOLD 16 | BRAKE 17 | GUIDED_NOGPS 20 | SMART_RTL 21

with SkyMAVLink(ENDPOINT) as mav:
    mav.wait_for_connection()

    # ── set_mode ─────────────────────────────────────────────────────────────
    mav.set_mode('STABILIZE', timeout=15)
    print('modo atual:', mav.get_mode())
    mav.sleep(2)

    mav.set_mode('LOITER')
    print('modo atual:', mav.get_mode())
    mav.sleep(2)

    mav.set_mode('GUIDED')
    print('modo atual:', mav.get_mode())
    mav.sleep(2)

    # --- por extenso ---
    # MODE = 4   # GUIDED
    # t_end = time.time() + 15
    # last_send = 0.0
    # while time.time() < t_end:
    #     if mav.is_connected() and mav._mode == MODE:
    #         print('modo confirmado')
    #         break
    #     if time.time() - last_send >= 1.0:          # reenvia a cada 1 s
    #         mav._conn.mav.command_long_send(
    #             1, 1,                                            # target sys, comp
    #             mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,          # confirmation = 0
    #             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1 = base_mode
    #             MODE,                                            # param2 = custom_mode
    #             0, 0, 0, 0, 0)
    #         last_send = time.time()
    #     time.sleep(0.05)      # a thread de fundo já está drenando e mandando heartbeat
    # else:
    #     raise TimeoutError('modo não confirmado')
    #
    # # trocar para um modo que NÃO seja GUIDED/GUIDED_NOGPS também cancela
    # # qualquer velocidade/trajetória/atitude ativa (mav._vel_cmd = None etc.)

    # ── pause() e loiter(): atalhos de set_mode ──────────────────────────────
    mav.pause()     # entra em BRAKE e espera a confirmação
    print('modo atual:', mav.get_mode())
    mav.sleep(2)

    mav.loiter()    # entra em LOITER (precisa de GPS) e espera a confirmação
    print('modo atual:', mav.get_mode())
    mav.sleep(2)

    mav.set_mode('GUIDED')

    # --- por extenso ---
    # # pause() é só self.set_mode('BRAKE', timeout); loiter() é
    # # self.set_mode('LOITER', timeout). Não têm lógica própria.

    # ── modo inválido levanta ValueError ─────────────────────────────────────
    try:
        mav.set_mode('TURBO')
    except ValueError as err:
        print('ValueError esperado:', err)

    # --- por extenso ---
    # # a lib valida o nome contra _ACM_MODE antes de mandar qualquer coisa;
    # # no cru, um custom_mode inexistente é simplesmente ignorado pelo FC
    # # (e o loop de confirmação estoura TimeoutError).
