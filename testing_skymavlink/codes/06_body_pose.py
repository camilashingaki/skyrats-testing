# anda um quadrado de 1 metro na direção do nariz, sobe, desce e gira 90 graus pro leste

"""06 — set_body_pose. VOA. Deslocamento RELATIVO em FRD (frente, direita, baixo).

Atenção: set_body_pose NÃO é uma mensagem de corpo. A lib lê a posição e o yaw
atuais via get_position()/get_attitude(), gira o offset FRD para NED na mão, e
manda um alvo ABSOLUTO em MAV_FRAME_LOCAL_NED. Por isso precisa de posição:
sem fix ela só avisa (warning no logger) e volta sem mandar nada.
"""

import math
import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL
ALT  = 1.5   # m
SIDE = 1.0   # m

with SkyMAVLink(ENDPOINT) as mav:
    try:
        mav.wait_for_connection()
        mav.wait_for_position(timeout=60)     # obrigatório pro set_body_pose
        mav.set_mode('GUIDED')

        # ── arm manual: espera o operador armar pelo rádio/GCS ───────────────
        print('arme o drone manualmente (rádio/GCS) — aguardando…')
        mav.wait_until(lambda: mav.is_armed(), timeout=120, description='arm manual')

        mav.takeoff(ALT)
        mav.sleep(5)

        # ── set_body_pose: quadrado relativo ao nariz ────────────────────────
        for fwd, right in [(SIDE, 0), (0, SIDE), (-SIDE, 0), (0, -SIDE)]:
            antes = mav.get_position()
            mav.set_body_pose(fwd, right, 0)
            mav.sleep(8)
            depois = mav.get_position()
            andou = math.hypot(depois[0] - antes[0], depois[1] - antes[1])
            print(f'fwd={fwd:+.2f} right={right:+.2f} → andou {andou:.3f} m, '
                  f'pos={depois[0]:+.2f},{depois[1]:+.2f}')

        # --- por extenso ---
        # fwd, right, down = 1.0, 0.0, 0.0
        # position = mav.get_position()
        # if position is None:
        #     raise RuntimeError('sem posição — chame wait_for_position() antes')
        # attitude = mav.get_attitude()
        # yaw = attitude[2] if attitude is not None else mav._yaw_ned   # rad, CW do Norte
        # c, s = math.cos(yaw), math.sin(yaw)
        # pn, pe, pd = position
        # # frente = (cos yaw, sin yaw) em (N, E);  direita = (-sin yaw, cos yaw)
        # target_n = pn + fwd * c + right * (-s)
        # target_e = pe + fwd * s + right * c
        # target_d = pd + down
        # # o resto é exatamente o set_local_pose "por extenso" do codes/05:
        # mav._vel_cmd = mav._trajectory_cmd = mav._attitude_cmd = None
        # mav._conn.mav.set_position_target_local_ned_send(
        #     0, 1, 1,
        #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        #     0x0FF8,                             # só posição (0x09F8 se mandar yaw junto)
        #     target_n, target_e, target_d,
        #     0, 0, 0,
        #     0, 0, 0,
        #     0, 0)

        # ── set_body_pose só em altura: sobe 0.5 m, desce 0.5 m ─────────────
        mav.set_body_pose(0, 0, -0.5)          # down negativo = SOBE
        mav.sleep(6)
        print(f'altura: {-mav.get_position()[2]:.2f} m')

        mav.set_body_pose(0, 0, +0.5)          # down positivo = DESCE
        mav.sleep(6)
        print(f'altura: {-mav.get_position()[2]:.2f} m')

        # ── set_body_pose girando o nariz 90° e andando pra frente ──────────
        # Prova de que o "frente" acompanha o yaw: o mesmo fwd=+1 leva a direções
        # diferentes do mundo antes e depois do giro.
        mav.set_body_pose(0, 0, 0, yaw_deg=90.0)   # olha pro Leste
        mav.sleep(8)
        antes = mav.get_position()
        mav.set_body_pose(SIDE, 0, 0)              # 1 m "pra frente" = 1 m pro Leste
        mav.sleep(8)
        depois = mav.get_position()
        print(f'dN={depois[0]-antes[0]:+.2f}  dE={depois[1]-antes[1]:+.2f}  '
              f'(esperado dN≈0, dE≈{SIDE:+.2f})')
    finally:
        mav.stop()
        if mav.is_armed():
            mav.land()
