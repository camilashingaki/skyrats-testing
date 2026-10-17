# anda pra frente, direita, tras e esquerda por velocidade formando um quadrado, depois sobe e desce

"""07 — move_velocity(reference=BODY), versão nectar-sdk do set_body_velocity.

Conversão FRD (skymavlink) -> FLU (nectar), igual ao codes/06:

    vx (frente) = fwd                          (igual)
    vy (esq.)   = -right                        (direita positiva -> esquerda negativa)
    vz (cima)   = -down                         (baixo positivo -> cima negativo)
    vyaw        = -radians(yaw_rate_dps)        (graus/s horário -> rad/s anti-horário)

Duas coisas que já vêm resolvidas aqui, sem precisar fazer na mão como no
skymavlink:

1. o transport do nectar SEMPRE manda a máscara "velocidade + yaw_rate"
   (nunca a que ignora yaw_rate) — o mesmo bug do WP_YAW_BEHAVIOR que o
   skymavlink precisou corrigir já não existe aqui por padrão.
2. move_velocity(..., duration=T) já manda velocidade zero sozinho no final
   do laço — não precisa do "hold() e só depois stop()" do skymavlink pra
   evitar o coasting do GUID_TIMEOUT. Só não dá pra confiar 100% nisso:
   zero é mandado uma vez só, então se esse pacote se perder o FC ainda
   pode coastar até o GUID_TIMEOUT. Pra um freio garantido, chame de novo
   com vx=vy=vz=vyaw=0 e duration=1~2s antes de seguir.
"""

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource
from nectar.control.types import MoveReference

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL
BAUD = 921600
ALT   = 1.5   # m
SPEED = 0.5   # m/s
SIDE  = 1.5   # m de perna

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

    # ── move_velocity: quadrado por velocidade, com freio extra de garantia ─
    for fwd, right in [(SPEED, 0), (0, SPEED), (-SPEED, 0), (0, -SPEED)]:
        pose_antes = drone.local_pose

        drone.move_velocity(vx=fwd, vy=-right, vz=0.0, vyaw=0.0,
                             duration=SIDE / SPEED, reference=MoveReference.BODY)
        # move_velocity(duration=...) já termina mandando velocidade zero;
        # isto aqui é só o freio extra de garantia contra perda de pacote.
        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0,
                             duration=1.5, reference=MoveReference.BODY)

        pose_depois = drone.local_pose
        print(f'perna fwd={fwd:+.2f} right={right:+.2f} → '
              f'antes={pose_antes}  depois={pose_depois}')

    # --- por extenso ---
    # move_velocity(duration=T) por dentro: um laço a 50 Hz que chama
    # _emit_velocity(vx, vy, vz, vyaw) até passar T segundos, e no final
    # chama de novo com tudo zero. _emit_velocity manda
    # SET_POSITION_TARGET_LOCAL_NED com a máscara "velocidade + yaw_rate" —
    # o transport converte vx/vy/vz de FLU/ENU pra FRD/NED e o vyaw (rad/s
    # anti-horário) pra rad/s horário antes de mandar pro FC.

    # ── sobe e desce por velocidade (vz positivo aqui = SOBE) ───────────────
    drone.move_velocity(vx=0, vy=0, vz=+0.3, duration=3.0, reference=MoveReference.BODY)
    drone.move_velocity(vx=0, vy=0, vz=0, duration=1.5, reference=MoveReference.BODY)
    print('altura após subir:', drone.get_altitude(), 'm')

    drone.move_velocity(vx=0, vy=0, vz=-0.3, duration=3.0, reference=MoveReference.BODY)
    drone.move_velocity(vx=0, vy=0, vz=0, duration=1.5, reference=MoveReference.BODY)
    print('altura após descer:', drone.get_altitude(), 'm')
finally:
    if drone.is_armed:
        drone.land()
    drone.cleanup()
    nectar.shutdown()
