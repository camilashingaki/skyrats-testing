# espera alguem armar pelo radio e desarma de volta, sem decolar

"""03 — disarm(), versão nectar-sdk. HÉLICES FORA. O motor gira.

Igual ao codes/03_arm_disarm.py: não arma pelo código, espera o operador
armar manualmente pelo rádio/GCS e só então testa o desarme.

Diferença importante: no nectar `disarm()` é SEMPRE forçado (manda
MAV_CMD_COMPONENT_ARM_DISARM com param2=21196, igual ao kill() do
skymavlink) — não existe um "disarm normal" na API pública. Em voo isso
faria o drone cair igual; no chão é inofensivo. `emergency_stop()` é só um
apelido pra essa mesma chamada.
"""

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL
BAUD = 921600

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

    # ── arm manual: espera o operador armar pelo rádio/GCS ───────────────────
    print('arme o drone manualmente (rádio/GCS) — aguardando…')
    while not drone.is_armed:
        drone.delay(0.2)
    print('armado?', drone.is_armed)

    # --- por extenso ---
    # is_armed é só uma property (drone._transport.state.armed), lida do
    # último HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED — não manda
    # nada pro FC, só espera o estado mudar sozinho.

    drone.delay(5)
    print('armado depois de 5 s?', drone.is_armed)

    # ── disarm: SEMPRE forçado (não existe desarme "normal" aqui) ──────────
    ok = drone.disarm()
    print('disarm confirmado?', ok, ' armado?', drone.is_armed)

    # --- por extenso ---
    # disarm() manda MAV_CMD_COMPONENT_ARM_DISARM(param1=0, param2=21196) uma
    # vez e devolve bool(ack aceito) — não espera is_armed virar False, então
    # pode devolver True um instante antes do HEARTBEAT confirmar.
finally:
    if drone.is_armed:
        drone.emergency_stop()   # rede de segurança: aqui sempre no chão, sem hélices
    drone.cleanup()
    nectar.shutdown()
