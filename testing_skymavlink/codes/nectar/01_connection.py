# conecta no drone, fica lendo posição e estado por uns segundos e desliga, sem armar e sem voar

"""01 — conexão e leitura de estado, versão nectar-sdk. NÃO arma, NÃO voa.

Mesma ideia do codes/01_connection.py, só que com o nectar-sdk no lugar do
skymavlink. Três diferenças grandes que valem a pena ler antes dos outros:

1. nectar é ROS 2 (precisa de rclpy + nectar-sdk instalados/buildados —
   `make setup` no repo do nectar-sdk; não é só `pip install` como o
   skymavlink). Por isso nada aqui importa de verdade nesta máquina agora
   (sem rclpy) — os scripts ficam prontos pra quando o ambiente tiver ROS 2.
2. frame INVERTIDO: nectar fala ENU/FLU (x=leste, y=norte, z=CIMA; corpo:
   frente, ESQUERDA, cima; yaw em radianos ENU, 0=leste, anti-horário
   positivo). skymavlink fala NED/FRD (down positivo, direita positiva,
   yaw horário). Troca de SDK aqui = troca de sinal em quase tudo.
3. erro não é exceção: quase todo método (`connect`, `arm`, `set_mode`,
   `takeoff`...) devolve True/False em vez de levantar TimeoutError/ValueError.
"""

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource

ENDPOINT = '/dev/ttyAMA0'          # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = '/dev/ttyACM0'        # Rasp -> FC direto por USB
# ENDPOINT = 'tcp:127.0.0.1:5762'  # SITL (porta SERIAL1 do ArduPilot)
BAUD = 921600

nectar.init()
drone = DroneFactory.create(
    'mavlink',
    MavlinkConfig(connection_string=ENDPOINT, baud=BAUD, pose_source=PoseSource.GPS,
                   start_driver=False),
)
try:
    # ── connect() ────────────────────────────────────────────────────────────
    if not drone.connect():
        raise RuntimeError('FC não conectou — confere fio, baud e SERIALx_PROTOCOL')

    # --- por extenso ---
    # connect() aqui é só: espera até 30 s (config.sensor_timeout) o transporte
    # marcar connected=True. Por dentro disso tem MavlinkConnection fazendo o
    # mesmo que mavutil.mavlink_connection(...) faz no skymavlink — abre a porta,
    # manda heartbeat 1 Hz, espera o primeiro HEARTBEAT do FC. Só que aqui
    # também sobe um nó ROS 2 por baixo (self._node), que o skymavlink não tem.

    print('conectado?', drone.is_fcu_connected, ' modo:', drone.flight_mode)

    # ── leitura de estado por uns segundos ──────────────────────────────────
    for i in range(10):
        drone.delay(1.0)      # equivalente ao mav.sleep() do skymavlink
        pose = drone.local_pose      # LocalPose ENU: position.x=leste, .y=norte, .z=cima; yaw rad ENU
        print(f'[{i}] pose={pose}  heading={drone.heading}°  '
              f'modo={drone.flight_mode}  armed={drone.is_armed}')
finally:
    drone.cleanup()
    nectar.shutdown()
