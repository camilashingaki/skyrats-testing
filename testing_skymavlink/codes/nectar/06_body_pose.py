# anda um quadrado de 1 metro na direção do nariz, sobe, desce e gira 90 graus, usando move_to(reference=BODY)

"""06 — move_to(reference=BODY), versão nectar-sdk do set_body_pose.

Mesma ideia do set_body_pose: desloca relativo à posição e ao heading
atuais. A diferença é FLU em vez de FRD — pra reaproveitar os mesmos
fwd/right/down do skymavlink, converte assim:

    x (nectar, frente)  =  fwd                (igual)
    y (nectar, esquerda) = -right              (direita positiva -> esquerda negativa)
    z (nectar, cima)     = -down               (baixo positivo -> cima negativo)

move_to(reference=BODY) exige posição local disponível (levanta
SensorNotAvailableError se não tiver GPS/visão fresca) — é o equivalente do
aviso "sem posição" do set_body_pose, só que aqui é exceção, não um warning
silencioso.
"""

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource
from nectar.control.types import MoveReference, NavigationMethod

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL
BAUD = 921600
ALT  = 1.5   # m
SIDE = 1.0   # m

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

    # ── move_to(BODY): quadrado relativo ao nariz ───────────────────────────
    for fwd, right in [(SIDE, 0), (0, SIDE), (-SIDE, 0), (0, -SIDE)]:
        pose_antes = drone.local_pose
        ok = drone.move_to(x=fwd, y=-right, z=0.0,
                            reference=MoveReference.BODY,
                            method=NavigationMethod.POSITION,
                            precision=0.15, timeout=15)
        pose_depois = drone.local_pose
        print(f'fwd={fwd:+.2f} right={right:+.2f} → chegou? {ok}  '
              f'antes={pose_antes}  depois={pose_depois}')

    # --- por extenso ---
    # move_to(reference=BODY) gira (x, y) pelo yaw ATUAL (não o de takeoff) e
    # soma à posição atual — exatamente a mesma conta do set_body_pose
    # (cos/sin do yaw), só que em ENU em vez de NED, o que já inverte o sinal
    # do componente lateral por conta própria.

    # ── sobe 0.5 m, desce 0.5 m (z é CIMA positivo aqui) ────────────────────
    drone.move_to(x=0, y=0, z=+0.5, reference=MoveReference.BODY,
                  method=NavigationMethod.POSITION, precision=0.1)
    print('altura:', drone.get_altitude(), 'm')

    drone.move_to(x=0, y=0, z=-0.5, reference=MoveReference.BODY,
                  method=NavigationMethod.POSITION, precision=0.1)
    print('altura:', drone.get_altitude(), 'm')

    # ── gira 90° (POSITION/BODY: graus anti-horário a partir do yaw atual) ─
    drone.move_to(x=0, y=0, z=0, yaw=90.0, reference=MoveReference.BODY,
                  method=NavigationMethod.POSITION)
    pose_antes = drone.local_pose
    drone.move_to(x=SIDE, y=0, z=0, reference=MoveReference.BODY,
                  method=NavigationMethod.POSITION, precision=0.15)
    pose_depois = drone.local_pose
    print(f'antes={pose_antes}  depois={pose_depois}  (prova que "frente" girou com o yaw)')
finally:
    if drone.is_armed:
        drone.land()
    drone.cleanup()
    nectar.shutdown()
