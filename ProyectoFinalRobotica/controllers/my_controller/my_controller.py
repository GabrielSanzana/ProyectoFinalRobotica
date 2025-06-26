import math
import random
import numpy as np
import warnings
from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display

warnings.filterwarnings("error")

# --- Constantes ---
TIME_STEP = 64
CELL_SIZE = 0.5
GRID_SIZE = 8
DISPLAY_WIDTH = 256
DISPLAY_HEIGHT = 256
SPEED = 10.0
ANGLE_THRESHOLD = 0.05
MOVEMENT_THRESHOLD = 0
MAX_STEP_RRT = 0.7
RRT_MAX_NODES = 1000
RRT_GOAL_THRESHOLD = 0.5

TURN_ANGLE_RAD = math.pi / 2  # 90 grados

# Colores
COLOR_BACKGROUND = 0xFFFFFF
COLOR_OBSTACLE = 0xFF0000
COLOR_GOAL = 0x00FF00
COLOR_ROBOT = 0x0000FF
COLOR_PATH = 0x000000

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def angle_diff(target, current):
    a = target - current
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def world2map(xw, yw):
    world_map_width = GRID_SIZE * CELL_SIZE
    world_map_height = GRID_SIZE * CELL_SIZE
    if any([math.isnan(xw), math.isnan(yw), math.isinf(xw), math.isinf(yw)]):
        raise ValueError(f"world2map recibió valores inválidos: xw={xw}, yw={yw}")
    px_world_units = (xw + world_map_width / 2) / CELL_SIZE
    py_world_units = (yw + world_map_height / 2) / CELL_SIZE
    px = int(px_world_units * (DISPLAY_WIDTH / GRID_SIZE))
    py = int(py_world_units * (DISPLAY_HEIGHT / GRID_SIZE))
    py = DISPLAY_HEIGHT - 1 - py
    return px, py

def distance(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# --- Nueva función para calcular la longitud de la ruta ---
def calculate_path_length(path):
    length = 0.0
    if path is None or len(path) < 2:
        return 0.0
    for i in range(len(path) - 1):
        length += distance(path[i], path[i+1])
    return length

def is_free_point(point):
    try:
        px, py = world2map(point[0], point[1])
    except ValueError:
        return False
    area_check = [(px + dx, py + dy) for dx in range(-2, 3) for dy in range(-2, 3)]
    return not any((x, y) in obstacle_map_pixels for (x, y) in area_check)

def sample():
    for _ in range(100):  # Reintentar hasta 100 veces
        x = random.uniform(-GRID_SIZE*CELL_SIZE/2, GRID_SIZE*CELL_SIZE/2)
        y = random.uniform(-GRID_SIZE*CELL_SIZE/2, GRID_SIZE*CELL_SIZE/2)
        if is_free_point([x, y]):
            return [x, y]
    return [0.0, 0.0]

def steer(x_nearest, x_rand, max_step=MAX_STEP_RRT):
    direction = np.array(x_rand) - np.array(x_nearest)
    dist = np.linalg.norm(direction)
    if dist == 0:
        return x_nearest
    if np.isnan(dist) or np.isnan(direction).any():
        print(f"NaN detectado en steer: dist={dist}, direction={direction}")
        return x_nearest
    if dist < max_step:
        return x_rand
    direction = direction / dist
    new_point = np.array(x_nearest) + direction * max_step
    return new_point.tolist()

def obstacle_check(p1, p2):
    if p1 is None or p2 is None:
        return False
    if any(math.isnan(v) or math.isinf(v) for v in p1 + p2):
        return False
    steps = 20
    for i in range(steps + 1):
        interp_x = p1[0] + (p2[0] - p1[0]) * i / steps
        interp_y = p1[1] + (p2[1] - p1[1]) * i / steps
        try:
            px, py = world2map(interp_x, interp_y)
        except ValueError:
            return False
        area_check = [(px + dx, py + dy) for dx in range(-2, 3) for dy in range(-2, 3)]
        if any((x, y) in obstacle_map_pixels for (x, y) in area_check):
            return False
    return True

def build_path(parents, last, goal):
    path = [goal, last]
    while parents.get(tuple(last)) is not None:
        last = parents[tuple(last)]
        path.append(last)
    return path[::-1]

def rrt(start, goal, obstacle_check_func, sample_func, steer_func, max_nodes=RRT_MAX_NODES, threshold=RRT_GOAL_THRESHOLD):
    print(f"RRT: start={start}, goal={goal}")
    if any(math.isnan(v) or math.isinf(v) for v in start) or any(math.isnan(v) or math.isinf(v) for v in goal):
        print("Valores inválidos en start o goal, abortando RRT")
        return None

    tree = [start]
    parents = {tuple(start): None}
    
    for _ in range(max_nodes):
        x_rand = sample_func()
        x_nearest = min(tree, key=lambda n: distance(n, x_rand))
        x_new = steer_func(x_nearest, x_rand)

        if obstacle_check_func(x_nearest, x_new):
            tree.append(x_new)
            parents[tuple(x_new)] = x_nearest

            if distance(x_new, goal) < threshold:
                print("¡Meta alcanzada en RRT!")
                return build_path(parents, x_new, goal)
    print("No se encontró camino.")
    return None

def dibujar_ruta(path):
    display.setColor(COLOR_PATH)
    for i in range(len(path)-1):
        px1, py1 = world2map(path[i][0], path[i][1])
        px2, py2 = world2map(path[i+1][0], path[i+1][1])
        # No dibujar líneas que pasen sobre obstáculos detectados
        line_clear = True
        steps = 10
        for step in range(steps + 1):
            interp_x = int(px1 + (px2 - px1) * step / steps)
            interp_y = int(py1 + (py2 - py1) * step / steps)
            if (interp_x, interp_y) in obstacle_map_pixels:
                line_clear = False
                break
        if line_clear:
            display.drawLine(px1, py1, px2, py2)

def ruta_bloqueada(path):
    if path is None or len(path) < 2:
        return True
    for i in range(len(path) -1):
        p1 = path[i]
        p2 = path[i+1]
        steps = 20
        for step in range(steps + 1):
            interp_x = p1[0] + (p2[0] - p1[0]) * step / steps
            interp_y = p1[1] + (p2[1] - p1[1]) * step / steps
            try:
                px, py = world2map(interp_x, interp_y)
            except ValueError:
                return True  # Coordenada inválida = bloqueada
            if (px, py) in obstacle_map_pixels:
                return True  # Obstáculo en el camino
    return False  # No bloqueada

robot = Robot()

wheels = []
for i in range(1, 5):
    wheel_name = "wheel" + str(i)
    wheel = robot.getDevice(wheel_name)
    if wheel:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(0.0)
        wheels.append(wheel)
    else:
        print(f"Advertencia: No se encontró la rueda {wheel_name}")

if not wheels:
    print("Error: No se encontraron ruedas.")
    robot.step(0)
    exit()

ds = []
ds_names = ["ds_left", "ds_right"]
for name in ds_names:
    s = robot.getDevice(name)
    if s:
        s.enable(TIME_STEP)
        ds.append(s)
    else:
        print(f"Advertencia: No se encontró el sensor de distancia {name}")

lidar = robot.getDevice("lidar")
if lidar:
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
else:
    print("Error: No se encontró el LIDAR.")
    robot.step(0)
    exit()

gps = robot.getDevice("gps")
if gps:
    gps.enable(TIME_STEP)
else:
    print("Error: No se encontró el GPS.")
    robot.step(0)
    exit()

imu = robot.getDevice("inertial unit")
if imu:
    imu.enable(TIME_STEP)
else:
    print("Error: No se encontró la Unidad Inercial.")
    robot.step(0)
    exit()

display = robot.getDevice("display")
if not display:
    print("Error: No se encontró el Display.")
    robot.step(0)
    exit()

goal = Point(0.0, 0.0)
obstacle_map_pixels = set()
prev_robot_x = None
prev_robot_y = None

turning = False
turn_start_yaw = 0.0
turn_direction = 1

planned_path = None
path_index = 0

rotating_to_path = False
target_yaw_for_path = 0.0

reached_goal = False

# --- ESPERA A GPS VÁLIDO ANTES DE PLANIFICAR ---
pose = gps.getValues()
while any(math.isnan(v) or math.isinf(v) for v in pose):
    robot.step(TIME_STEP)
    pose = gps.getValues()

start_point = [pose[0], pose[1]]
goal_point = [goal.x, goal.y]

# --- Inicialización de Métricas de Desempeño ---
start_time = robot.getTime()
total_distance_traveled = 0.0
replanning_count = 0
metrics_printed = False # Flag para asegurar que las métricas se impriman solo una vez


print("Planificando ruta inicial RRT...")
planned_path = rrt(start_point, goal_point, obstacle_check, sample, steer)
path_index = 0

if planned_path:
    print("Ruta inicial encontrada, deshabilitando lidar para giro inicial.")
    lidar.disable()
    first_target = planned_path[0]
    dx = first_target[0] - start_point[0]
    dy = first_target[1] - start_point[1]
    target_yaw_for_path = math.atan2(dy, dx)
    rotating_to_path = True
else:
    print("No se encontró ruta inicial.")

# Dibujar mapa inicial y ruta
display.setColor(COLOR_BACKGROUND)
display.fillRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT)

display.setColor(COLOR_OBSTACLE)
for (px, py) in obstacle_map_pixels:
    display.fillRectangle(px, py, 2, 2)

goal_px, goal_py = world2map(goal.x, goal.y)
if 0 <= goal_px < DISPLAY_WIDTH and 0 <= goal_py < DISPLAY_HEIGHT:
    display.setColor(COLOR_GOAL)
    display.fillRectangle(goal_px, goal_py, 4, 4)

robot_px, robot_py = world2map(start_point[0], start_point[1])
if 0 <= robot_px < DISPLAY_WIDTH and 0 <= robot_py < DISPLAY_HEIGHT:
    display.setColor(COLOR_ROBOT)
    display.fillRectangle(robot_px, robot_py, 4, 4)

if planned_path is not None:
    dibujar_ruta(planned_path)

# --- LOOP PRINCIPAL ---
while robot.step(TIME_STEP) != -1:
    pose = gps.getValues()
    robot_x = pose[0]
    robot_y = pose[1]

    imu_rpy = imu.getRollPitchYaw()
    yaw = imu_rpy[2]

    if reached_goal:
        for w in wheels:
            w.setVelocity(0.0)
        lidar.disable()
        continue

    if not rotating_to_path:
        ranges = lidar.getRangeImage()
        resolution = lidar.getHorizontalResolution()
        fov = lidar.getFov()
    else:
        ranges = None
        resolution = 0
        fov = 0

    if prev_robot_x is None:
        prev_robot_x, prev_robot_y = robot_x, robot_y

    moved_distance = math.sqrt((robot_x - prev_robot_x) ** 2 + (robot_y - prev_robot_y) ** 2)
    
    # --- Acumulación de métrica de distancia ---
    total_distance_traveled += moved_distance


    if not rotating_to_path and moved_distance > MOVEMENT_THRESHOLD:
        for i in range(resolution):
            # Corregir cálculo del ángulo relativo para que coincida derecha= derecha en mapa
            lidar_angle_relative = fov / 2 - i * (fov / resolution)
            dist = ranges[i]
            if math.isinf(dist) or dist > lidar.getMaxRange() or dist < 0.1:
                continue
            global_angle = yaw + lidar_angle_relative
            obs_x = robot_x + dist * math.cos(global_angle)
            obs_y = robot_y + dist * math.sin(global_angle)
            px, py = world2map(obs_x, obs_y)
            if 0 <= px < DISPLAY_WIDTH and 0 <= py < DISPLAY_HEIGHT:
                obstacle_map_pixels.add((px, py))
        prev_robot_x, prev_robot_y = robot_x, robot_y

    ds_detect_near = any(s.getValue() < 950.0 for s in ds)

    left_speed = 0.0
    right_speed = 0.0

    if rotating_to_path:
        diff = angle_diff(target_yaw_for_path, yaw)
        Kp_rotate = 5.0
        turn_speed = Kp_rotate * diff
        max_turn_speed = SPEED
        turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))
        left_speed = -turn_speed
        right_speed = turn_speed
        if abs(diff) < ANGLE_THRESHOLD:
            rotating_to_path = False
            lidar.enable(TIME_STEP)

    elif planned_path is not None:
        # Verificar si la ruta está bloqueada
        if ruta_bloqueada(planned_path):
            print("Ruta bloqueada, replanificando...")
            # --- Conteo de métrica de replanificación ---
            replanning_count += 1
            start = [robot_x, robot_y]
            planned_path = rrt(start, goal_point, obstacle_check, sample, steer)
            path_index = 0
            if planned_path:
                lidar.disable()
                first_target = planned_path[0]
                dx = first_target[0] - robot_x
                dy = first_target[1] - robot_y
                target_yaw_for_path = math.atan2(dy, dx)
                rotating_to_path = True
        else:
            if path_index < len(planned_path):
                target_pos = planned_path[path_index]
                dx = target_pos[0] - robot_x
                dy = target_pos[1] - robot_y
                dist_to_target = math.sqrt(dx*dx + dy*dy)
                if dist_to_target < 0.1:
                    path_index += 1
                    if path_index >= len(planned_path):
                        # --- CÓDIGO PARA MOSTRAR MÉTRICAS AL LLEGAR A LA META ---
                        if not metrics_printed:
                            print("¡Llegó a la meta!")
                            reached_goal = True
                            
                            # --- CÁLCULO Y MUESTRA DE MÉTRICAS DE DESEMPEÑO ---
                            end_time = robot.getTime()
                            total_time = end_time - start_time
                            straight_line_distance = distance(start_point, goal_point)
                            path_efficiency = (total_distance_traveled / straight_line_distance) if straight_line_distance > 0 else float('inf')

                            print("\n" + "="*30)
                            print("   MÉTRICAS DE DESEMPEÑO")
                            print("="*30)
                            print(f"Tiempo total de ejecución: {total_time:.2f} segundos")
                            print(f"Distancia total recorrida: {total_distance_traveled:.2f} metros")
                            print(f"Distancia en línea recta (inicio a fin): {straight_line_distance:.2f} metros")
                            print(f"Eficiencia de la ruta (Recorrido / Línea Recta): {path_efficiency:.2f}")
                            print(f"Número de replanificaciones (llamadas a RRT): {replanning_count}")
                            print("="*30 + "\n")

                            metrics_printed = True # Marcar como impresas

                        for w in wheels:
                            w.setVelocity(0.0)
                        lidar.disable()
                        continue
                else:
                    target_angle = math.atan2(dy, dx)
                    diff = angle_diff(target_angle, yaw)
                    Kp_angular = 5.0
                    turn_speed = Kp_angular * diff
                    max_turn_speed = SPEED
                    turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))
                    if abs(diff) > ANGLE_THRESHOLD:
                        left_speed = -turn_speed
                        right_speed = turn_speed
                    else:
                        left_speed = SPEED
                        right_speed = SPEED
            else:
                # Sin más puntos, avanzar hacia goal directamente
                dx = goal.x - robot_x
                dy = goal.y - robot_y
                target_angle = math.atan2(dy, dx)
                diff = angle_diff(target_angle, yaw)
                Kp_angular = 5.0
                turn_speed = Kp_angular * diff
                max_turn_speed = SPEED
                turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))
                if abs(diff) > ANGLE_THRESHOLD:
                    left_speed = -turn_speed
                    right_speed = turn_speed
                else:
                    left_speed = SPEED
                    right_speed = SPEED

    else:
        if turning:
            turned_angle = angle_diff(yaw, turn_start_yaw)
            if turn_direction == 1:
                if turned_angle >= TURN_ANGLE_RAD or turned_angle < 0:
                    turning = False
            else:
                if turned_angle <= -TURN_ANGLE_RAD or turned_angle > 0:
                    turning = False
            angular_speed = 2.0 * turn_direction
            left_speed = -angular_speed
            right_speed = angular_speed
        elif ds_detect_near:
            print("Obstáculo cerca, planeando ruta...")
            # --- Conteo de métrica de replanificación ---
            replanning_count += 1
            start = [robot_x, robot_y]
            planned_path = rrt(start, goal_point, obstacle_check, sample, steer)
            path_index = 0
            if planned_path:
                lidar.disable()
                first_target = planned_path[0]
                dx = first_target[0] - robot_x
                dy = first_target[1] - robot_y
                target_yaw_for_path = math.atan2(dy, dx)
                rotating_to_path = True
            turning = True
            turn_start_yaw = yaw
            turn_direction = -1
            angular_speed = 2.0 * turn_direction
            left_speed = -angular_speed
            right_speed = angular_speed
        else:
            dx = goal.x - robot_x
            dy = goal.y - robot_y
            target_angle = math.atan2(dy, dx)
            diff = angle_diff(target_angle, yaw)
            Kp_angular = 5.0
            turn_speed = Kp_angular * diff
            max_turn_speed = SPEED
            turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))
            if abs(diff) > ANGLE_THRESHOLD:
                left_speed = -turn_speed
                right_speed = turn_speed
            else:
                left_speed = SPEED
                right_speed = SPEED

    if len(wheels) == 4:
        wheels[0].setVelocity(left_speed)
        wheels[1].setVelocity(right_speed)
        wheels[2].setVelocity(left_speed)
        wheels[3].setVelocity(right_speed)
    elif len(wheels) == 2:
        wheels[0].setVelocity(left_speed)
        wheels[1].setVelocity(right_speed)
    else:
        for wheel in wheels:
            wheel.setVelocity(0.0)

    # Dibujar mapa, robot, objetivo y ruta (solo si no llegó a la meta)
    if not reached_goal:
        display.setColor(COLOR_BACKGROUND)
        display.fillRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT)

        display.setColor(COLOR_OBSTACLE)
        for (px, py) in obstacle_map_pixels:
            display.fillRectangle(px, py, 2, 2)

        goal_px, goal_py = world2map(goal.x, goal.y)
        if 0 <= goal_px < DISPLAY_WIDTH and 0 <= goal_py < DISPLAY_HEIGHT:
            display.setColor(COLOR_GOAL)
            display.fillRectangle(goal_px, goal_py, 4, 4)

        robot_px, robot_py = world2map(robot_x, robot_y)
        if 0 <= robot_px < DISPLAY_WIDTH and 0 <= robot_py < DISPLAY_HEIGHT:
            display.setColor(COLOR_ROBOT)
            display.fillRectangle(robot_px, robot_py, 4, 4)

        if planned_path is not None:
            dibujar_ruta(planned_path)