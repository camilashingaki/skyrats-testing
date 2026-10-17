# espera alguem armar pelo radio e desarma de volta, sem decolar

"""03 — disarm(). HÉLICES FORA. O motor gira.

Este script NÃO arma pelo código: o arm fica por conta do operador, manual,
pelo rádio/GCS — é a trava de segurança de verdade (motor só liga com um
humano decidindo). O script só espera mav.is_armed() virar True e então testa
disarm(), o desarme normal, sem force. O desarme forçado (kill(),
param2=21196) só aparece no bloco comentado — nunca rode isso fora de uma
emergência de verdade, porque em voo o drone cai.
"""

import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL

with SkyMAVLink(ENDPOINT) as mav:
    try:
        mav.wait_for_connection()
        mav.set_mode('GUIDED')

        # ── arm manual: espera o operador armar pelo rádio/GCS ───────────────
        print('arme o drone manualmente (rádio/GCS) — aguardando…')
        mav.wait_until(lambda: mav.is_armed(), timeout=120, description='arm manual')
        print('armado?', mav.is_armed())

        mav.sleep(5)
        print('armado depois de 5 s?', mav.is_armed())

        # ── disarm: desarme normal, no chão ─────────────────────────────────
        mav.disarm(timeout=15)
        print('armado?', mav.is_armed())

        # --- por extenso ---
        # t_end = time.time() + 15
        # last_send = 0.0
        # while time.time() < t_end:
        #     if mav.is_connected() and not mav.is_armed():
        #         print('disarm confirmado')
        #         break
        #     if time.time() - last_send >= 1.0:
        #         mav._conn.mav.command_long_send(
        #             1, 1,
        #             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        #             0,      # param1 = 0 → desarma
        #             0,      # param2 = 0 → sem force; o FC recusa se achar que está voando
        #             0, 0, 0, 0, 0)
        #         last_send = time.time()
        #     time.sleep(0.05)
        # else:
        #     raise TimeoutError('disarm não confirmado')
        #
        # # com force (= o que mav.kill() faz): param2 = 21196. No ar, o drone CAI.
        # # disarm() sem force é rejeitado pelo FC em voo — ver codes/04 pra esse caso.
    finally:
        if mav.is_armed():
            mav.kill()   # rede de segurança: aqui estamos sempre no chão, sem hélices
