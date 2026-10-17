# anda pra frente, direita, tras e esquerda por velocidade formando um quadrado, depois sobe e desce

"""07 — set_body_velocity. VOA. Velocidade contínua em FRD, reenviada a 20 Hz
pela thread de fundo (não precisa mais de mav.sleep() pra manter o setpoint
vivo — isso agora é automático).

Dois cuidados que este teste existe pra mostrar:
  1. yaw_rate_dps=0 é enviado explicitamente (nunca "ignorado"), senão o FC
     vira o nariz pra direção do voo (WP_YAW_BEHAVIOR) — ver skymavlink/motion.py;
  2. stop() NÃO freia: ele só cancela a retransmissão. hold() é o freio ativo
     de verdade (manda velocidade NED zero contínua). Use hold() e só depois
     stop(), como no README: `drone.hold(); drone.sleep(1); drone.stop()`.
"""

import math
import time

from pymavlink import mavutil   # usado só pelos blocos "por extenso"

from skymavlink import SkyMAVLink

ENDPOINT = 'serial:/dev/ttyAMA0:921600'    # Rasp -> FC por UART (TX/RX na TELEM)
# ENDPOINT = 'serial:/dev/ttyACM0:115200'   # Rasp -> FC direto por USB
# ENDPOINT = 'udpout:127.0.0.1:14552'       # Rasp, atrás do mavp2p
# ENDPOINT = 'tcp:127.0.0.1:5760'           # SITL
ALT   = 1.5   # m
SPEED = 0.5   # m/s
SIDE  = 1.5   # m de perna

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

        # ── valores não finitos levantam ValueError antes de mandar nada ─────
        try:
            mav.set_body_velocity(float('inf'), 0.0, 0.0)
        except ValueError as err:
            print('ValueError esperado:', err)

        # ── set_body_velocity: quadrado por velocidade, com freada ativa ────
        for fwd, right in [(SPEED, 0), (0, SPEED), (-SPEED, 0), (0, -SPEED)]:
            antes = mav.get_position()

            mav.set_body_velocity(fwd, right, 0.0)     # yaw_rate_dps=0 → segura o nariz
            mav.sleep(SIDE / SPEED)                    # a thread de fundo reenvia sozinha

            mav.hold()           # freio ativo de verdade: velocidade NED zero contínua
            mav.sleep(2.0)
            mav.stop()           # agora sim: a velocidade já está ~0, cancela a retransmissão

            depois = mav.get_position()
            andou = math.hypot(depois[0] - antes[0], depois[1] - antes[1])
            yaw = math.degrees(mav.get_attitude()[2]) % 360
            print(f'perna fwd={fwd:+.2f} right={right:+.2f} → {andou:.3f} m  yaw={yaw:6.2f}°')

        # --- por extenso ---
        # fwd, right, down, yaw_rate_dps = 0.5, 0.0, 0.0, 0.0
        # if not all(math.isfinite(v) for v in (fwd, right, down, yaw_rate_dps)):
        #     raise ValueError('velocity values must be finite')
        # yr = math.radians(yaw_rate_dps)
        #
        # # set_body_velocity() HOJE é só isto — guardar a tupla sob o lock.
        # # Não existe mais loop de reenvio no código da missão: a thread de
        # # fundo (_pump_once) já pega mav._vel_cmd e reenvia a 20 Hz sozinha.
        # with mav._state_lock:
        #     mav._attitude_cmd = None
        #     mav._trajectory_cmd = None
        #     mav._vel_cmd = (mavutil.mavlink.MAV_FRAME_BODY_NED, 0x05C7,
        #                     fwd, right, down, yr)
        # time.sleep(SIDE / SPEED)      # só espera; não precisa mandar nada na mão
        #
        # # hold() por extenso: a mesma coisa, mas em MAV_FRAME_LOCAL_NED (mundo),
        # # com tudo zero — funciona em qualquer frame porque velocidade zero é
        # # zero nos dois.
        # with mav._state_lock:
        #     mav._vel_cmd = (mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0x05C7, 0.0, 0.0, 0.0, 0.0)
        #
        # # stop() por extenso: só limpa os três comandos contínuos.
        # with mav._state_lock:
        #     mav._vel_cmd = mav._trajectory_cmd = mav._attitude_cmd = None
        #
        # # NUNCA troque 0x05C7 por 0x0DC7 (_MASK_VEL) quando o yaw_rate for 0.
        # # Marcar yaw_rate como "ignorar" entrega o nariz pro WP_YAW_BEHAVIOR do FC,
        # # que vira o nariz pra direção do voo: 80.7° de deriva medidos num quadrado
        # # de 1 m a 0.8 m/s. Mandar zero explícito segura o nariz (0.046°).

        # ── set_body_velocity vertical: sobe e desce por velocidade ─────────
        mav.set_body_velocity(0.0, 0.0, -0.3)   # down negativo = SOBE a 0.3 m/s
        mav.sleep(3.0)
        mav.hold()
        mav.sleep(2.0)
        mav.stop()
        print(f'altura após subir: {-mav.get_position()[2]:.2f} m')

        mav.set_body_velocity(0.0, 0.0, +0.3)   # desce
        mav.sleep(3.0)
        mav.hold()
        mav.sleep(2.0)
        mav.stop()
        print(f'altura após descer: {-mav.get_position()[2]:.2f} m')
    finally:
        mav.stop()
        if mav.is_armed():
            mav.land()
