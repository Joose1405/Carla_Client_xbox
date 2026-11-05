#!/usr/bin/env python3
"""
Cliente CARLA + pygame + ROS 2 (reemplaza a bridge_carla_connector + xbox_custom_teleop).

Cadena de control:

  Xbox (pygame) --> /vehicle/raw_cmd_vel --> [ GUARDIAN ] --> /vehicle/safe_cmd_vel --+
                                                                                     |
  CARLA <-- VehicleControl (PI de velocidad + dirección) <---------------------------+
  CARLA --> /vehicle/speedometer       (Float32, m/s)        --> [ GUARDIAN ]
  CARLA --> /perception/lead_obstacle  (Point: x=gap m, y=desfase lateral m, z=v_rel m/s)

Convenciones (REP-103 / ROS):
  linear.x  = velocidad demandada (m/s), negativa = reversa
  angular.z = tasa de guiñada demandada (rad/s), POSITIVA = giro a la IZQUIERDA
  lead.z    = v_ego - v_lead  (positiva = acercándose)
  lead.y    = positivo = obstáculo a la izquierda

Mapeo del mando (SDL2 / pygame 2, driver xpad):
  Stick izquierdo (X) : dirección        RT : acelerador (demanda de velocidad)
  LT                  : freno            A  : alternar reversa
  B (mantener)        : freno de mano local (no pasa por el Guardian)
  Start               : reiniciar el coche   Back / ESC : salir
"""
import argparse
import datetime
import math
import queue
import random
import sys
import time

import carla
import numpy as np
import pygame
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Float32

# ------------------------- configuración -------------------------
WIDTH, HEIGHT = 1280, 720
FPS = 50                      # = ciclo de 20 ms del Guardian: 1 tick de CARLA por ciclo
DT = 1.0 / FPS
DEADZONE = 0.08

V_DEMAND_MAX = 20.0           # m/s que pide RT a fondo (el Guardian recorta a 13.89)
V_REVERSE_MAX = 3.0           # m/s máximo en reversa
W_MAX = 0.8                   # rad/s que pide el stick a fondo (el Guardian lo limita a 0.3 g)
RELEASE_DECEL = 1.0           # m/s^2: cuánto baja la demanda al soltar RT (evita frenazo)

CMD_TIMEOUT = 0.2             # s sin safe_cmd_vel -> el coche frena
NO_OBSTACLE = 1000.0          # m
LANE_HALF_WIDTH = 2.0         # m, ancho considerado "mi carril"
WHEELBASE = 2.875             # m (Tesla Model 3, aproximado)
KP, KI = 0.5, 0.1             # PI de velocidad

AX = dict(STEER=0, LT=2, RT=5)
BTN = dict(A=0, B=1, X=2, Y=3, LB=4, RB=5, BACK=6, START=7)
# -----------------------------------------------------------------


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_deadzone(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)


def shape_stick(s):
    """Curva suave: poca sensibilidad cerca del centro."""
    return s * abs(s) * 0.9 + s * (1 - abs(s)) * 0.3


class Trigger:
    """Gatillo SDL2: reposo = -1.0, a fondo = +1.0 (algunos drivers dan 0.0
    hasta el primer movimiento; mientras valga 0.0 se trata como suelto)."""

    def __init__(self, joy, idx):
        self.joy, self.idx, self.seen = joy, idx, False

    def read(self):
        v = self.joy.get_axis(self.idx)
        if not self.seen:
            if v == 0.0:
                return 0.0
            self.seen = True
        return clamp((v + 1.0) / 2.0, 0.0, 1.0)


class DemandShaper:
    """Convierte RT/LT en magnitud de velocidad demandada.
    - Al apretar RT la demanda sube al instante.
    - Al soltarla baja suave desde la velocidad real (no un frenazo).
    - LT pone la demanda en 0 de inmediato."""

    def __init__(self):
        self.mag = 0.0

    def reset(self):
        self.mag = 0.0

    def step(self, target, lt, speed_mag):
        if lt > 0.1:
            self.mag = 0.0
        elif target >= self.mag:
            self.mag = target
        else:
            self.mag = max(target, min(self.mag, speed_mag) - RELEASE_DECEL * DT)
        return self.mag


class SpeedController:
    """Twist (v, w) -> carla.VehicleControl.  PI de velocidad + dirección cinemática."""

    def __init__(self, max_steer_rad):
        self.max_steer = max_steer_rad
        self.int_err = 0.0

    def reset(self):
        self.int_err = 0.0

    def compute(self, v_t, w_t, v_fwd):
        ctrl = carla.VehicleControl()
        direction = -1.0 if v_t < -0.05 else 1.0
        v_cmd = abs(v_t) if abs(v_t) > 0.05 else 0.0
        v_meas = direction * v_fwd            # velocidad en el sentido pedido

        if v_cmd == 0.0:                      # parada / AEB
            self.int_err = 0.0
            ctrl.brake = 1.0 if abs(v_fwd) > 0.1 else 0.0 # freno fuerte si aún se mueve, freno suave si ya está parado. valor de 0.5 si si lo queremos
        elif v_meas < -0.3:                   # va en sentido contrario: frenar primero
            self.int_err = 0.0
            ctrl.brake = 1.0
        else:
            err = v_cmd - v_meas
            self.int_err = clamp(self.int_err + err * DT, -5.0, 5.0)
            u = KP * err + KI * self.int_err
            ctrl.reverse = direction < 0
            if u >= 0.0:
                ctrl.throttle = clamp(u, 0.0, 1.0)
            else:
                ctrl.brake = clamp(-u, 0.0, 1.0)

        # w (rad/s, ROS: +izquierda) -> ángulo de ruedas (CARLA: +derecha)
        v_eff = direction * max(abs(v_fwd), 1.0)
        delta = math.atan(WHEELBASE * w_t / v_eff)
        ctrl.steer = clamp(-delta / self.max_steer, -1.0, 1.0)
        return ctrl


class GuardianLink(Node):
    """Todo lo de ROS 2: publicadores hacia el Guardian y suscripción a su salida."""

    def __init__(self):
        super().__init__('carla_pygame_client')
        self.pub_raw = self.create_publisher(Twist, '/vehicle/raw_cmd_vel', 10)
        self.pub_speed = self.create_publisher(Float32, '/vehicle/speedometer', 10)
        self.pub_obs = self.create_publisher(Point, '/perception/lead_obstacle', 10)
        self.create_subscription(Twist, '/vehicle/safe_cmd_vel', self._safe_cb, 10)
        self.safe_v = 0.0
        self.safe_w = 0.0
        self._safe_stamp = None

    def _safe_cb(self, msg):
        self.safe_v = msg.linear.x
        self.safe_w = msg.angular.z
        self._safe_stamp = time.monotonic()

    def safe_age(self):
        return float('inf') if self._safe_stamp is None else time.monotonic() - self._safe_stamp

    def publish_raw(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub_raw.publish(m)

    def publish_telemetry(self, speed, gap, dy, closing):
        self.pub_speed.publish(Float32(data=float(speed)))
        self.pub_obs.publish(Point(x=float(gap), y=float(dy), z=float(closing)))


def forward_speed(actor):
    """Velocidad con signo a lo largo del eje longitudinal del coche (m/s)."""
    f = actor.get_transform().get_forward_vector()
    v = actor.get_velocity()
    return v.x * f.x + v.y * f.y + v.z * f.z


def lead_info(ego, lead):
    """(gap longitudinal parachoques-a-parachoques, desfase lateral ROS, v_rel) o None."""
    t = ego.get_transform()
    f, r = t.get_forward_vector(), t.get_right_vector()
    ol = lead.get_location()
    rx, ry = ol.x - t.location.x, ol.y - t.location.y
    dx = rx * f.x + ry * f.y
    dy = rx * r.x + ry * r.y                 # CARLA: + = derecha
    if dx <= 0.0 or abs(dy) > LANE_HALF_WIDTH:
        return None
    gap = max(dx - ego.bounding_box.extent.x - lead.bounding_box.extent.x, 0.0)
    ev, ov = ego.get_velocity(), lead.get_velocity()
    closing = (ev.x - ov.x) * f.x + (ev.y - ov.y) * f.y
    return gap, -dy, closing                  # ROS: + = izquierda


def spawn_ego(world, spawn_points, index=None):
    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    bp.set_attribute('role_name', 'ego')
    candidates = list(spawn_points)
    if index is not None:
        candidates = [candidates[index % len(candidates)]]
    else:
        random.shuffle(candidates)
    for sp in candidates:
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            return actor, sp
    raise RuntimeError('No se pudo crear el vehículo (spawn point ocupado).')


def spawn_lead(world, spawn_tf, distance):
    """Audi estático con freno de mano, `distance` m adelante POR EL CARRIL."""
    nxt = world.get_map().get_waypoint(spawn_tf.location).next(distance)
    if not nxt:
        return None
    tf = nxt[0].transform
    tf.location.z += 0.5
    actor = world.try_spawn_actor(
        world.get_blueprint_library().find('vehicle.audi.tt'), tf)
    if actor is not None:
        actor.apply_control(carla.VehicleControl(hand_brake=True))
    return actor


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--debug', action='store_true', help='imprime ejes y botones del mando')
    ap.add_argument('--bypass-guardian', action='store_true',
                    help='aplica raw_cmd_vel directo a CARLA (para comparar sin Guardian)')
    ap.add_argument('--external-teleop', action='store_true',
                    help='no leer el mando aquí: usar tu joy_node + xbox_custom_teleop')
    ap.add_argument('--no-obstacle', action='store_true', help='no crear el Audi líder')
    ap.add_argument('--lead-distance', type=float, default=35.0)
    ap.add_argument('--spawn-index', type=int, default=None,
                    help='spawn point fijo (por defecto aleatorio)')
    args = ap.parse_args()
    if args.bypass_guardian and args.external_teleop:
        ap.error('--bypass-guardian necesita el teleop interno (sin --external-teleop)')
    use_pad = not args.external_teleop

    pygame.init()
    pygame.joystick.init()
    joy = None
    if pygame.joystick.get_count() > 0:
        joy = pygame.joystick.Joystick(0)
        print(f'Mando: {joy.get_name()}  ({joy.get_numaxes()} ejes, {joy.get_numbuttons()} botones)')
    elif use_pad:
        sys.exit('No se detectó ningún mando. Conéctalo y vuelve a intentar.')

    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption('CARLA - ROS 2 - Xbox')
    font = pygame.font.SysFont('monospace', 20)
    clock = pygame.time.Clock()

    rclpy.init()
    node = GuardianLink()

    world = None
    vehicle = camera = lead = None
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)
        world = client.get_world()

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        world.apply_settings(settings)

        vehicle, spawn_tf = spawn_ego(world, world.get_map().get_spawn_points(), args.spawn_index)
        if not args.no_obstacle:
            lead = spawn_lead(world, spawn_tf, args.lead_distance)
            if lead is None:
                print('AVISO: no se pudo crear el obstáculo (usa otro --spawn-index o --no-obstacle).')

        vehicle_name = ' '.join(vehicle.type_id.replace('_', ' ').split('.')[1:]).title()
        map_name = world.get_map().name.split('/')[-1]
        print(f'Vehículo: {vehicle_name} | Mapa: {map_name}')

        max_steer = math.radians(vehicle.get_physics_control().wheels[0].max_steer_angle)
        controller = SpeedController(max_steer)

        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(WIDTH))
        cam_bp.set_attribute('image_size_y', str(HEIGHT))
        cam_bp.set_attribute('fov', '90')
        cam_tf = carla.Transform(carla.Location(x=-6.0, z=2.6), carla.Rotation(pitch=-10.0))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        image_q = queue.Queue()
        camera.listen(image_q.put)

        rt = Trigger(joy, AX['RT']) if (use_pad and joy) else None
        lt = Trigger(joy, AX['LT']) if (use_pad and joy) else None
        shaper = DemandShaper()
        reverse = False
        connected = True
        v_raw = w_raw = 0.0
        info = None
        white, red, green = (255, 255, 255), (255, 90, 90), (120, 255, 120)

        while True:
            clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return
                if event.type == pygame.JOYDEVICEREMOVED and use_pad:
                    connected = False
                    print('¡Mando desconectado! Demanda 0 y freno.')
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN['BACK']:
                        return
                    if event.button == BTN['A'] and use_pad:
                        reverse = not reverse
                    if event.button == BTN['START']:
                        vehicle.set_transform(spawn_tf)
                        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                        vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
                        reverse = False
                        shaper.reset()
                        controller.reset()

            # ---- ROS 2: recibir safe_cmd_vel (sin hilos: un solo bucle) ----
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.0)

            v_fwd = forward_speed(vehicle)

            # ---- mando -> Twist crudo -> Guardian ----
            if use_pad:
                if connected and rt is not None:
                    s = shape_stick(apply_deadzone(joy.get_axis(AX['STEER'])))
                    mag = shaper.step(rt.read() * (V_REVERSE_MAX if reverse else V_DEMAND_MAX),
                                      lt.read(), abs(v_fwd))
                    v_raw = -mag if reverse else mag
                    # stick izquierda = s<0 = yaw ROS positivo. En reversa se invierte
                    # para que el stick siga actuando como un volante.
                    w_raw = -s * W_MAX * (-1.0 if reverse else 1.0)
                else:
                    v_raw = w_raw = 0.0
                node.publish_raw(v_raw, w_raw)

            # ---- ¿qué comando se aplica? ----
            if args.bypass_guardian:
                v_t, w_t, link_ok = v_raw, w_raw, True
            else:
                link_ok = node.safe_age() <= CMD_TIMEOUT          # watchdog del cliente
                v_t, w_t = (node.safe_v, node.safe_w) if link_ok else (0.0, 0.0)

            ctrl = controller.compute(v_t, w_t, v_fwd)
            if use_pad and connected and joy is not None and joy.get_button(BTN['B']):
                ctrl.hand_brake = True
            if use_pad and not connected:
                ctrl = carla.VehicleControl(brake=1.0, hand_brake=True)
            vehicle.apply_control(ctrl)

            if args.debug and use_pad and connected:
                axes = ' '.join(f'{i}:{joy.get_axis(i):+.2f}' for i in range(joy.get_numaxes()))
                btns = ''.join(str(joy.get_button(i)) for i in range(joy.get_numbuttons()))
                print(f'\rejes {axes} | botones {btns}   ', end='', flush=True)

            # ---- avanzar simulación, imagen de ESE frame ----
            frame = world.tick()
            image = None
            while image is None or image.frame < frame:
                image = image_q.get(timeout=5.0)

            # ---- telemetría -> Guardian (estado ya actualizado tras el tick) ----
            speed_now = abs(forward_speed(vehicle))
            info = lead_info(vehicle, lead) if (lead is not None and lead.is_alive) else None
            gap, dy, closing = info if info else (NO_OBSTACLE, 0.0, 0.0)
            node.publish_telemetry(speed_now, gap, dy, closing)

            # ---- imagen -> numpy -> pygame ----
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
            screen.blit(pygame.surfarray.make_surface(arr.swapaxes(0, 1)), (0, 0))

            # ---- HUD ----
            sim_time = datetime.timedelta(
                seconds=int(world.get_snapshot().timestamp.elapsed_seconds))
            if args.bypass_guardian:
                mode = ('Modo: BYPASS (sin Guardian)', red)
            elif link_ok:
                mode = ('Modo: GUARDIAN  enlace OK', green)
            else:
                mode = ('Modo: GUARDIAN  SIN safe_cmd_vel -> frenando', red)
            if info:
                ttc = gap / closing if closing > 0.1 else None
                lead_txt = (f'Lead: {gap:5.1f} m  v_rel {closing:+5.1f} m/s  '
                            + (f'TTC {ttc:4.1f} s' if ttc else 'TTC --'))
                lead_col = red if (ttc is not None and ttc < 1.5) else white
            else:
                lead_txt, lead_col = 'Lead: sin obstaculo en el carril', white
            lines = [
                (f'Vehicle: {vehicle_name}', white),
                (f'Map: {map_name}', white),
                (f'Simulation time: {sim_time}   {clock.get_fps():4.1f} FPS', white),
                ('', white),
                (f'{3.6 * speed_now:5.1f} km/h   {"R" if v_fwd < -0.1 or reverse else "D"}', white),
                mode,
                (f'raw  v={v_raw:+6.2f} m/s  w={w_raw:+5.2f} rad/s', white),
                (f'safe v={node.safe_v:+6.2f} m/s  w={node.safe_w:+5.2f} rad/s', white),
                (lead_txt, lead_col),
                (f'acel {ctrl.throttle:.2f}  freno {ctrl.brake:.2f}  dir {ctrl.steer:+.2f}', white),
            ]
            for i, (text, color) in enumerate(lines):
                if text:
                    screen.blit(font.render(text, True, color, (0, 0, 0)), (12, 12 + 26 * i))
            pygame.display.flip()

    finally:
        print()
        if camera is not None:
            try:
                camera.stop()
                camera.destroy()
            except Exception as e:
                print(f'Limpieza cámara: {e}')
        for actor in (vehicle, lead):
            if actor is not None:
                try:
                    actor.destroy()
                except Exception as e:
                    print(f'Limpieza actor: {e}')
        if world is not None:
            try:
                # Si el servidor se queda en modo síncrono sin cliente, se congela.
                s = world.get_settings()
                s.synchronous_mode = False
                s.fixed_delta_seconds = None
                world.apply_settings(s)
            except Exception as e:
                print(f'No se pudo restaurar el modo asíncrono: {e}')
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
