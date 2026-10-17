# decola até 1.5 metro, fica parado no ar um pouco e pousa

"""04 — takeoff() e land(), versão nectar-sdk. VOA. O menor voo possível.

Diferença grande: takeoff() aqui ARMA POR DENTRO (faz set_mode('GUIDED') +
arm() como parte da própria sequência, com retry automático). Esperar o
operador armar manualmente antes (como no codes/03) não é opcional pra
evitar isso — só garante que, quando takeoff() chamar arm() de novo, já vai
estar armado e essa chamada não faz nada de novo. Também não levanta
TimeoutError nem ValueError: tudo vira um booleano.
"""

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL
BAUD = 921600
ALT = 1.5   # m

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

    # ── takeoff ──────────────────────────────────────────────────────────────
    ok = drone.takeoff(altitude=ALT)
    print('takeoff confirmado?', ok, ' altura:', drone.get_altitude(), 'm')
    if not ok:
        raise RuntimeError('takeoff falhou — ver log do nectar pra causa')

    # --- por extenso ---
    # takeoff(altitude) por dentro, até 2 tentativas (max_retries):
    #   arm() -> espera subida dos motores -> seta a posição de takeoff ->
    #   manda o comando -> espera (sequencer.wait_takeoff_settle) a altura
    #   assentar -> ajusta fino com move_to() se precisão não bateu.
    # Se a tentativa falha, desarma e tenta de novo; devolve False só depois
    # de esgotar as tentativas.

    drone.delay(5)

    # ── land ─────────────────────────────────────────────────────────────────
    ok = drone.land(timeout=60)
    print('land confirmado?', ok, ' armado?', drone.is_armed)

    # --- por extenso ---
    # land() manda o comando de pousar, espera a velocidade de descida parar
    # (sequencer.wait_landed) e então o HEARTBEAT confirmar desarmado — sem
    # LAND_SPEED/param explícito como no skymavlink; a velocidade de descida
    # é o que já estiver configurado no FC.
finally:
    if drone.is_armed:
        drone.land()
    drone.cleanup()
    nectar.shutdown()
