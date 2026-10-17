# conecta no drone, fica lendo posição e estado por uns segundos e desliga, sem armar e sem voar

"""01 — conexão, streams e leitura de estado. NÃO arma, NÃO voa.

Rode este primeiro. Se ele não imprimir posição, nada do resto funciona.
"""

import math
import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL

with SkyMAVLink(ENDPOINT) as mav:
    # ── wait_for_connection ──────────────────────────────────────────────────
    mav.wait_for_connection(timeout=30)

    # --- por extenso ---
    # # O construtor já abre uma thread de fundo (_pump_loop) que drena
    # # mensagens, manda heartbeat 1 Hz e reenvia setpoints — não existe mais
    # # nenhum "_tick()" manual pra chamar daqui. É só esperar o estado mudar.
    # while not mav._connected:
    #     time.sleep(0.05)
    # mav._request_streams()      # pede LOCAL_POSITION_NED, ATTITUDE,
    #                              # GLOBAL_POSITION_INT, GPS_RAW_INT, DISTANCE_SENSOR,
    #                              # SYS_STATUS, HOME_POSITION, MISSION_CURRENT... a 10 Hz
    # # equivalente público: mav.request_streams(rate_hz=10)

    print('conectado?', mav.is_connected(), ' modo:', mav.get_mode())

    # ── wait_ready: conecta + confere modo + (opcional) exige GPS ───────────
    mav.wait_ready(timeout=30, require_gps=False)

    # --- por extenso ---
    # if not mav._connected:
    #     mav.wait_for_connection(timeout=30)
    # else:
    #     mav._request_streams()
    # while not (mav.is_connected() and mav.get_mode() is not None):
    #     time.sleep(0.05)
    # # com require_gps=True também exige mav.has_gps_fix() e mav.get_global_position()

    # ── wait_for_position ────────────────────────────────────────────────────
    n, e, d = mav.wait_for_position(timeout=60)   # EKF frio demora ~40 s
    print(f'posição inicial: N={n:.2f} E={e:.2f} D={d:.2f}  (altura = {-d:.2f} m)')

    # --- por extenso ---
    # while mav.get_position() is None:
    #     time.sleep(0.05)
    # n, e, d = mav.get_position()

    # ── get_position / get_attitude / get_heading / get_mode / is_armed ─────
    for i in range(10):
        mav.sleep(1.0)     # hoje é só um time.sleep(); quem mantém o link vivo é a thread
        pos = mav.get_position()
        att = mav.get_attitude()
        yaw_deg = math.degrees(att[2]) if att else float('nan')
        print(f'[{i}] pos={pos}  yaw={yaw_deg:7.2f}°  heading={mav.get_heading()}  '
              f'modo={mav.get_mode()}  armed={mav.is_armed()}')

    # --- por extenso ---
    # # mav.sleep(s) hoje é só time.sleep(s) — a thread de fundo segue sozinha,
    # # então dá pra ler o estado mesmo depois de um time.sleep() comum.
    # time.sleep(1.0)
    # pos   = mav._pos_ned        # (north, east, down), protegido por _state_lock
    # att   = mav._att_ned        # (roll, pitch, yaw) em radianos
    # armed = mav._armed and mav._fresh('heartbeat', max_age=3.0)
