#!/usr/bin/env python3
"""
Cliente CARLA + pygame controlado con un mando de Xbox.

Sigue el esquema del tutorial oficial (tuto_G_pygame):
  - modo síncrono
  - cámara RGB con sensor.listen() -> Queue
  - buffer de imagen -> numpy -> pygame.Surface

Mapeo (SDL2 / pygame 2 con driver xpad en Linux):
  Stick izquierdo (X) : dirección
  RT                  : acelerador
  LT                  : freno
  A                   : alternar marcha atrás
  B (mantener)        : freno de mano
  Start               : reiniciar el coche en el punto de spawn
  Back / ESC          : salir

Si algún eje/botón no coincide con tu mando, ejecuta con --debug
y ajusta los diccionarios AX y BTN de abajo.
"""
import argparse
import datetime
import queue
import random
import sys

import carla
import numpy as np
import pygame

# ------------------------- configuración -------------------------
WIDTH, HEIGHT = 1280, 720
FPS = 30                      # = 1 / fixed_delta_seconds
DEADZONE = 0.08

AX = dict(STEER=0, LT=2, RT=5)
BTN = dict(A=0, B=1, X=2, Y=3, LB=4, RB=5, BACK=6, START=7)
# -----------------------------------------------------------------


def apply_deadzone(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    # reescala para que la salida arranque suave en 0 tras la zona muerta
    return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)


class Trigger:
    """Gatillo SDL2: reposo = -1.0, a fondo = +1.0.
    Algunos drivers reportan 0.0 hasta que se mueve el gatillo por primera
    vez; mientras no cambie de 0.0 lo tratamos como 'sin presionar'."""

    def __init__(self, joy, idx):
        self.joy, self.idx, self.seen = joy, idx, False

    def read(self):
        v = self.joy.get_axis(self.idx)
        if not self.seen:
            if v == 0.0:
                return 0.0
            self.seen = True
        return max(0.0, min(1.0, (v + 1.0) / 2.0))


def spawn_ego(world, spawn_points):
    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    bp.set_attribute('role_name', 'ego')
    random.shuffle(spawn_points)
    for sp in spawn_points:
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            return actor, sp
    raise RuntimeError('No se pudo crear el vehículo (todos los spawn points ocupados).')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--debug', action='store_true',
                    help='imprime ejes y botones del mando para verificar el mapeo')
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit('No se detectó ningún mando. Conéctalo y vuelve a intentar.')
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f'Mando: {joy.get_name()}  ({joy.get_numaxes()} ejes, {joy.get_numbuttons()} botones)')

    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption('CARLA - Xbox')
    font = pygame.font.SysFont('monospace', 20)
    clock = pygame.time.Clock()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    original_settings = world.get_settings()

    vehicle = camera = None
    try:
        # Modo síncrono: un world.tick() = un paso de simulación
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / FPS
        world.apply_settings(settings)

        vehicle, spawn_tf = spawn_ego(world, world.get_map().get_spawn_points())

        # Cámara RGB detrás del coche
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(WIDTH))
        cam_bp.set_attribute('image_size_y', str(HEIGHT))
        cam_bp.set_attribute('fov', '90')
        cam_tf = carla.Transform(carla.Location(x=-6.0, z=2.6), carla.Rotation(pitch=-10.0))
        camera = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
        image_q = queue.Queue()
        camera.listen(image_q.put)

        rt, lt = Trigger(joy, AX['RT']), Trigger(joy, AX['LT'])
        reverse = False
        connected = True
        ctrl = carla.VehicleControl()

        while True:
            clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    return
                if event.type == pygame.JOYDEVICEREMOVED:
                    connected = False
                    print('¡Mando desconectado! Frenando.')
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN['BACK']:
                        return
                    if event.button == BTN['A']:
                        reverse = not reverse
                    if event.button == BTN['START']:
                        vehicle.set_transform(spawn_tf)
                        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                        vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
                        reverse = False

            # ---- mando -> VehicleControl ----
            if connected:
                steer = apply_deadzone(joy.get_axis(AX['STEER']))
                steer = steer * abs(steer) * 0.9 + steer * (1 - abs(steer)) * 0.3  # curva suave
                ctrl.steer = max(-1.0, min(1.0, steer))   # CARLA: + = derecha (igual que el stick)
                ctrl.throttle = rt.read()
                ctrl.brake = lt.read()
                ctrl.reverse = reverse
                ctrl.hand_brake = bool(joy.get_button(BTN['B']))
            else:
                # Watchdog sencillo: sin mando, frenar
                ctrl = carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)

            vehicle.apply_control(ctrl)

            if args.debug and connected:
                axes = ' '.join(f'{i}:{joy.get_axis(i):+.2f}' for i in range(joy.get_numaxes()))
                btns = ''.join(str(joy.get_button(i)) for i in range(joy.get_numbuttons()))
                print(f'\rejes {axes} | botones {btns}   ', end='', flush=True)

            # ---- avanzar simulación y recibir la imagen de ESE frame ----
            frame = world.tick()
            image = None
            while image is None or image.frame < frame:
                image = image_q.get(timeout=5.0)

            # ---- imagen -> numpy -> pygame ----
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]  # BGRA -> RGB
            screen.blit(pygame.surfarray.make_surface(arr.swapaxes(0, 1)), (0, 0))

            # ---- HUD ----
            v = vehicle.get_velocity()
            kmh = 3.6 * (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
            lines = [
                f'{kmh:5.1f} km/h   {"R" if reverse else "D"}',
                f'acel {ctrl.throttle:.2f}  freno {ctrl.brake:.2f}  dir {ctrl.steer:+.2f}',
            ]
            for i, text in enumerate(lines):
                screen.blit(font.render(text, True, (255, 255, 255), (0, 0, 0)), (12, 12 + 26 * i))
            pygame.display.flip()

    finally:
        # Muy importante: si el servidor se queda en modo síncrono sin cliente, se congela
        print()
        try:
            if camera is not None:
                camera.stop()
                camera.destroy()
            if vehicle is not None:
                vehicle.destroy()
            world.apply_settings(original_settings)
        finally:
            pygame.quit()


if __name__ == '__main__':
    main()
