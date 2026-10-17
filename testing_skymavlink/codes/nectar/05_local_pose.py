# voa um quadrado de 1.5 metro indo direto pra cada canto, usando move_to(reference=TAKEOFF)

"""05 — move_to(reference=TAKEOFF), versão nectar-sdk do set_local_pose.

skymavlink manda um alvo ABSOLUTO em NED a partir da origem do EKF. O nectar
não deixa fazer isso direto: move_to(reference=MoveReference.WORLD) nem
existe (levanta CapabilityNotSupportedError de propósito). O mais parecido
é reference=TAKEOFF — um alvo absoluto, mas ancorado no ponto e no heading
de onde o drone decolou, em ENU/FLU: x=pra frente do heading de takeoff,
y=pra ESQUERDA do heading de takeoff (skymavlink: east era y positivo e
direita; aqui é o oposto).

yaw aqui (method=POSITION) é um deslocamento em graus ANTI-horário somado
ao heading de takeoff — não é um heading absoluto em graus como o yaw_deg
do skymavlink.
"""

import math

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource
from nectar.control.types import MoveReference, NavigationMethod

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL
BAUD = 921600
ALT  = 1.5   # m
SIDE = 1.5   # m

nectar.init()
drone = DroneFactory.create(
    'mavlink',
    MavlinkConfig(connection_string=ENDPOINT, baud=BAUD, pose_source=PoseSource.GPS,
                   start_driver=False),
)
try:
    if not drone.connect():
        raise RuntimeError('FC não conectou')
    drone.set_mode('GUIDED')

    print('arme o drone manualmente (rádio/GCS) — aguardando…')
    while not drone.is_armed:
        drone.delay(0.2)

    if not drone.takeoff(altitude=ALT):
        raise RuntimeError('takeoff falhou')
    drone.delay(5)

    # ── move_to: quadrado absoluto a partir do ponto de takeoff ─────────────
    alvos = [
        (SIDE, 0.0,  None),    # frente
        (SIDE, SIDE, None),    # frente-esquerda
        (0.0,  SIDE, None),    # esquerda
        (0.0,  0.0,  None),    # volta pro ponto de takeoff
    ]
    for x, y, yaw in alvos:
        ok = drone.move_to(x=x, y=y, z=0.0, yaw=yaw,
                            reference=MoveReference.TAKEOFF,
                            method=NavigationMethod.POSITION,
                            precision=0.15, timeout=20)
        pose = drone.local_pose
        print(f'alvo x={x:+.2f} y={y:+.2f} → chegou? {ok}  pose={pose}')

    # --- por extenso ---
    # move_to(reference=TAKEOFF, method=POSITION) por dentro:
    #   1) pega a posição/yaw de takeoff guardados (self._takeoff_local)
    #   2) gira (x, y) pelo heading de takeoff igual o set_body_pose do
    #      skymavlink gira por yaw_ned — só que em ENU, então o sinal da
    #      rotação sai trocado
    #   3) manda um alvo ABSOLUTO em SET_POSITION_TARGET_LOCAL_NED (frame
    #      MAV_FRAME_LOCAL_NED, convertido de ENU pra NED por dentro do
    #      transport) e espera chegar dentro de `precision`

    # ── move_to com yaw: sobe mais 0.5 m e gira 90° anti-horário ────────────
    ok = drone.move_to(x=0.0, y=0.0, z=0.5, yaw=90.0,
                        reference=MoveReference.TAKEOFF,
                        method=NavigationMethod.POSITION, precision=0.15, timeout=20)
    pose = drone.local_pose
    yaw_deg_enu = math.degrees(pose.yaw) % 360 if pose else float('nan')
    print(f'chegou? {ok}  yaw medido (ENU, 0=leste): {yaw_deg_enu:.1f}°  '
          f'altura: {drone.get_altitude()} m')
finally:
    if drone.is_armed:
        drone.land()
    drone.cleanup()
    nectar.shutdown()
