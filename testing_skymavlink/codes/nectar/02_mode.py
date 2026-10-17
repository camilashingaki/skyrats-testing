# troca de modo varias vezes no chão (stabilize, loiter, guided, brake...), sem armar e sem voar

"""02 — set_mode(), versão nectar-sdk. NÃO arma, NÃO voa. Só troca de modo no chão.

Diferença principal: set_mode() devolve True/False, não levanta ValueError
pra nome inválido — ele só manda a troca pro FC, que ignora nomes que não
existem, e o método acaba devolvendo False por timeout de confirmação.
Não existem pause()/loiter() prontos no nectar — é sempre set_mode('BRAKE')
ou set_mode('LOITER') na mão.
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

    # ── set_mode ─────────────────────────────────────────────────────────────
    ok = drone.set_mode('STABILIZE', timeout=15)
    print('STABILIZE confirmado?', ok, ' modo atual:', drone.flight_mode)
    drone.delay(2)

    ok = drone.set_mode('LOITER')
    print('LOITER confirmado?', ok, ' modo atual:', drone.flight_mode)
    drone.delay(2)

    ok = drone.set_mode('GUIDED')
    print('GUIDED confirmado?', ok, ' modo atual:', drone.flight_mode)
    drone.delay(2)

    # --- por extenso ---
    # set_mode(name, timeout) por dentro:
    #   1) drone._transport.set_mode(name) — manda MAV_CMD_DO_SET_MODE uma vez
    #      (sem retry automático, diferente do skymavlink que reenvia a cada 1 s)
    #   2) espera até `timeout` (default mode_timeout=10 s) o HEARTBEAT reportar
    #      o mesmo nome em flight_mode
    #   3) devolve True se confirmou, False (+ log de erro) se não confirmou —
    #      nunca levanta excessão daqui.

    # ── BRAKE/LOITER "na mão", já que não tem pause()/loiter() prontos ──────
    drone.set_mode('BRAKE')
    print('modo atual:', drone.flight_mode)
    drone.delay(2)

    drone.set_mode('LOITER')
    print('modo atual:', drone.flight_mode)
    drone.delay(2)

    drone.set_mode('GUIDED')

    # ── modo inválido: False, não ValueError ────────────────────────────────
    ok = drone.set_mode('TURBO', timeout=5)
    print('modo inválido confirmado?', ok, '(esperado: False, sem exceção)')
finally:
    drone.cleanup()
    nectar.shutdown()
