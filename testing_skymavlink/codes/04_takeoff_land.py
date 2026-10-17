# decola até 1.5 metro, fica parado no ar um pouco e pousa

"""04 — takeoff() e land(). VOA. O menor voo possível: sobe, para, desce.

Mudou em relação à versão antiga da lib: takeoff() agora LEVANTA TimeoutError
se não alcançar 95% da altura dentro do timeout (antes só logava e seguia).
Também valida altitude <= 0 com ValueError antes de mandar qualquer coisa
pro FC, e land() valida speed_ms <= 0 do mesmo jeito.
"""

import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL
ALT = 1.5         # m acima do chão
LAND_SPEED = 0.3  # m/s de descida

with SkyMAVLink(ENDPOINT) as mav:
    try:
        mav.wait_for_connection()
        mav.wait_for_position(timeout=60)
        mav.set_mode('GUIDED')

        # ── arm manual: espera o operador armar pelo rádio/GCS ───────────────
        print('arme o drone manualmente (rádio/GCS) — aguardando…')
        mav.wait_until(lambda: mav.is_armed(), timeout=120, description='arm manual')

        # ── takeoff(altitude <= 0): ValueError antes de mandar comando ──────
        try:
            mav.takeoff(0.0)
        except ValueError as err:
            print('ValueError esperado:', err)

        # ── takeoff ──────────────────────────────────────────────────────────
        mav.takeoff(ALT, timeout=30)      # levanta TimeoutError se não alcançar 95%
        alt = -mav.get_position()[2]
        print(f'altura após takeoff: {alt:.2f} m  (alvo {ALT:.2f} m)')

        # --- por extenso ---
        # if ALT <= 0:
        #     raise ValueError('takeoff altitude must be positive')
        # mav._conn.mav.command_long_send(
        #     1, 1,                                        # target sys, comp
        #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,      # confirmation = 0
        #     0, 0, 0, 0,                                  # param1..4 não usados no copter
        #     0, 0,                                        # lat, lon = 0 → posição atual
        #     ALT)                                         # param7 = altitude (m)
        # t_end = time.time() + 30
        # while time.time() < t_end:
        #     # NED: down é negativo acima do chão → chegou quando down <= -0.95 * ALT
        #     p = mav.get_position()
        #     if p and p[2] <= -(0.95 * ALT):
        #         print('reached', -p[2])
        #         break
        #     time.sleep(0.05)
        # else:
        #     reached = -mav.get_position()[2] if mav.get_position() else 0.0
        #     raise TimeoutError(f'takeoff altitude {ALT} m not reached (reached {reached:.2f} m)')

        # ── disarm() é rejeitado em voo — só land()/rtl()/kill() funcionam ──
        try:
            mav.disarm(timeout=5)
        except TimeoutError as err:
            print('disarm em voo rejeitado, como esperado:', err)

        mav.sleep(5)

        # ── land(speed_ms <= 0): ValueError antes de tocar no FC ────────────
        try:
            mav.land(speed_ms=-1.0)
        except ValueError as err:
            print('ValueError esperado:', err)

        # ── land ─────────────────────────────────────────────────────────────
        mav.land(speed_ms=LAND_SPEED, timeout=60)
        print('desarmado?', not mav.is_armed())

        # --- por extenso ---
        # # 1) LAND_SPEED é em cm/s no FC (o arg da lib é m/s) e FICA GRAVADO no FC
        # if LAND_SPEED <= 0:
        #     raise ValueError('landing speed must be positive')
        # mav._conn.mav.param_set_send(
        #     1, 1,
        #     b'LAND_SPEED',                               # pymavlink exige bytes
        #     LAND_SPEED * 100.0,                          # m/s → cm/s
        #     mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # time.sleep(0.5)                                  # deixa o FC aplicar o param
        #
        # # 2) cancela qualquer velocidade/trajetória/atitude ativa e entra em LAND (custom_mode 9)
        # t_end = time.time() + 60
        # last_send = 0.0
        # while time.time() < t_end:
        #     if mav.is_connected() and not mav.is_armed():
        #         print('pousou e desarmou')
        #         break
        #     if time.time() - last_send >= 2.0:
        #         mav._conn.mav.command_long_send(
        #             1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        #             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #             9,                                   # 9 = LAND
        #             0, 0, 0, 0, 0)
        #         last_send = time.time()
        #     time.sleep(0.05)
        # else:
        #     raise TimeoutError('land não confirmado')
    finally:
        mav.stop()
        if mav.is_armed():
            mav.land()
