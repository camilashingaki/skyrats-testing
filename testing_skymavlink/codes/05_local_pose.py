# voa um quadrado de 1.5 metro indo direto pra cada canto e depois sobe olhando pro leste

"""05 — set_local_pose. VOA. Posição ABSOLUTA em NED, a partir da origem do EKF.

NED: +N norte, +E leste, +D para baixo → altura 1.5 m é down = -1.5.
yaw_deg: graus horários a partir do Norte (0 = N, 90 = L, 180 = S, 270 = O).
"""

import math
import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL
ALT  = 2  # m
SIDE = 1.5   # m

with SkyMAVLink(ENDPOINT) as mav:
    try:
        mav.wait_for_connection()
        mav.wait_for_position(timeout=60)
        mav.set_mode('GUIDED')

        # ── arm manual: espera o operador armar pelo rádio/GCS ───────────────
        print('arme o drone manualmente (rádio/GCS) — aguardando…')
        mav.wait_until(lambda: mav.is_armed(), timeout=120, description='arm manual')

        mav.takeoff(ALT)
        mav.sleep(5)

        n0, e0, _ = mav.get_position()     # origem do quadrado = onde estamos agora
        print(f'origem: N={n0:.2f} E={e0:.2f}')

        # ── NaN / inf levanta ValueError antes de mandar qualquer coisa ──────
        try:
            mav.set_local_pose(float('nan'), e0, -ALT)
        except ValueError as err:
            print('ValueError esperado:', err)

        # ── set_local_pose: quadrado em coordenadas absolutas ────────────────
        alvos = [
            (n0 + SIDE, e0,        -ALT, None),    # norte
            (n0 + SIDE, e0 + SIDE, -ALT, None),    # leste
            (n0,        e0 + SIDE, -ALT, None),    # sul
            (n0,        e0,        -ALT, None),    # oeste, volta pra origem
        ]
        for north, east, down, yaw in alvos:
            mav.set_local_pose(north, east, down, yaw_deg=yaw)
            mav.sleep(8)
            pn, pe, pd = mav.get_position()
            erro = math.hypot(pn - north, pe - east)
            print(f'alvo N={north:+.2f} E={east:+.2f} → chegou N={pn:+.2f} E={pe:+.2f} '
                  f'alt={-pd:.2f}  erro={erro:.3f} m')

        # --- por extenso ---
        # if not all(math.isfinite(v) for v in (north, east, down)):
        #     raise ValueError('pose values must be finite')
        # # qualquer posição cancela velocidade, trajetória e atitude ativas
        # mav._vel_cmd = None
        # mav._trajectory_cmd = None
        # mav._attitude_cmd = None
        # mav._conn.mav.set_position_target_local_ned_send(
        #     0,                                  # time_boot_ms (0 = agora)
        #     1, 1,                               # target sys, comp
        #     mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        #     0x0FF8,                             # type_mask: bit=1 → IGNORA. 0x0FF8 = só posição
        #     north, east, down,                  # x=N, y=E, z=D (metros)
        #     0, 0, 0,                            # vx, vy, vz   (ignorados pela máscara)
        #     0, 0, 0,                            # afx, afy, afz (ignorados)
        #     0, 0)                               # yaw (rad), yaw_rate (ignorados)
        #
        # # com yaw: máscara 0x09F8 e yaw em RADIANOS
        # # mav._conn.mav.set_position_target_local_ned_send(
        # #     0, 1, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x09F8,
        # #     north, east, down, 0,0,0, 0,0,0, math.radians(90.0), 0)

        # ── set_local_pose com yaw: sobe e olha pro Leste ────────────────────
        mav.set_local_pose(n0, e0, -(ALT + 0.5), yaw_deg=90.0)
        mav.sleep(8)
        att = mav.get_attitude()
        print(f'yaw medido: {math.degrees(att[2]) % 360:.1f}°  (alvo 90°)  '
              f'alt={-mav.get_position()[2]:.2f} m')
    finally:
        mav.stop()
        if mav.is_armed():
            mav.land()
