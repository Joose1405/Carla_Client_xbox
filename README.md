# Cliente CARLA con mando Xbox

Esta carpeta contiene dos scripts de Python para conducir un vehículo en el
simulador CARLA con un mando Xbox y visualizar la cámara del vehículo en una
ventana de Pygame.

## Contenido

- `scripts/carla_xbox_client.py`: cliente con un HUD básico que muestra la
  velocidad, la marcha y los valores de aceleración, freno y dirección.
- `scripts/carla_xbox_client2.py`: variante que añade al HUD el nombre del
  vehículo, el mapa y el tiempo transcurrido de simulación.

Ambos scripts usan una simulación síncrona a 30 FPS, crean un Tesla Model 3 en
un punto de inicio disponible y montan una cámara RGB detrás del vehículo.
Al salir, destruyen los actores creados y restauran la configuración original
del mundo.

## Requisitos

- Un servidor CARLA en ejecución y accesible desde el equipo.
- Python con la API de CARLA, `pygame` y `numpy` instalados.
- Un mando compatible detectado por Pygame.

## Ejecución

Desde la carpeta raíz del proyecto:

```bash
python3 scripts/carla_xbox_client.py
```

Para ejecutar la variante con más información en pantalla:

```bash
python3 scripts/carla_xbox_client2.py
```

Por defecto, ambos clientes se conectan a `localhost:2000`. El host y el puerto
se pueden indicar como argumentos:

```bash
python3 scripts/carla_xbox_client.py --host 127.0.0.1 --port 2000
```

Agrega `--debug` para imprimir los ejes y botones detectados y comprobar el
mapeo del mando:

```bash
python3 scripts/carla_xbox_client.py --debug
```

## Controles

| Control | Acción |
| --- | --- |
| Stick izquierdo, eje horizontal | Dirección |
| Gatillo RT | Acelerador |
| Gatillo LT | Freno |
| Botón A | Alternar marcha atrás |
| Mantener B | Freno de mano |
| Start | Reiniciar el vehículo en su punto de inicio |
| Back o tecla Esc | Salir |

El mapeo de ejes y botones está configurado en las constantes `AX` y `BTN` de
cada script; puede ser necesario ajustarlo según el mando y el controlador
utilizado.
