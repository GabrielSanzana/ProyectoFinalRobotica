from controller import Robot
import math

# Constantes
TIME_STEP = 64
GRID_SIZE = 8
CELL_SIZE = 0.5
MAX_PATH_LEN = 100
MAX_OPEN_NODES = 1000
SPEED = 10.0

# Estructuras de datos
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    def __init__(self, x, y, g, h, f, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = f
        self.parent_index = parent_index

# Heurística Manhattan
def heuristic(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

# Planificación de ruta (A*)
def plan_path(grid, start, goal, max_path_len):
    open_list = []
    closed_list = []

    start_node = Node(start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1)
    start_node.f = start_node.g + start_node.h
    open_list.append(start_node)

    while open_list:
        best_index = min(range(len(open_list)), key=lambda i: open_list[i].f)
        current = open_list.pop(best_index)
        closed_list.append(current)

        if current.x == goal.x and current.y == goal.y:
            path = []
            n = current
            while n.parent_index != -1 and len(path) < max_path_len:
                path.append(Point(n.x, n.y))
                n = closed_list[n.parent_index]
            path.append(Point(start.x, start.y))
            path.reverse()
            return path

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = current.x + dx, current.y + dy
            if not (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE):
                continue
            if grid[nx][ny] == 1:
                continue
            if any(n.x == nx and n.y == ny for n in closed_list):
                continue

            g = current.g + 1
            h = heuristic(nx, ny, goal.x, goal.y)
            f = g + h

            existing = next((n for n in open_list if n.x == nx and n.y == ny), None)
            if existing:
                if f < existing.f:
                    existing.g = g
                    existing.h = h
                    existing.f = f
                    existing.parent_index = len(closed_list) - 1
            elif len(open_list) < MAX_OPEN_NODES:
                open_list.append(Node(nx, ny, g, h, f, len(closed_list) - 1))
    return []

# Inicio del controlador
robot = Robot()

# Motores
wheel_names = ["motor1", "motor2", "motor3", "motor4"]
wheels = [robot.getDevice(name) for name in wheel_names]
for wheel in wheels:
    wheel.setPosition(float('inf'))

# Sensores de distancia
ds_names = ["distIzq", "distDer"]
ds = [robot.getDevice(name) for name in ds_names]
for sensor in ds:
    sensor.enable(TIME_STEP)

# LIDAR
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# GPS
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# Mapa de grilla
grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
avoid_obstacle_counter = 0

# Bucle principal
while robot.step(TIME_STEP) != -1:
    left_speed = 10.0
    right_speed = 10.0

    if avoid_obstacle_counter > 0:
        avoid_obstacle_counter -= 1
        left_speed = 1.0
        right_speed = -1.0
    else:
        ds_values = [sensor.getValue() for sensor in ds]
        if ds_values[0] < 950.0 or ds_values[1] < 950.0:
            avoid_obstacle_counter = 100

    # Posición del robot
    pose = gps.getValues()
    robot_x = pose[0]
    robot_y = pose[1]  # Webots usa X-Z como plano horizontal

    # Datos del LIDAR
    ranges = lidar.getRangeImage()
    resolution = lidar.getHorizontalResolution()
    fov = lidar.getFov()

    for i in range(resolution):
        angle = -fov / 2 + i * (fov / resolution)
        dist = ranges[i]
        if dist < 1.0:
            obs_x = robot_x + dist * math.cos(angle)
            obs_y = robot_y + dist * math.sin(angle)
            cell_x = int((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE)
            cell_y = int((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE)
            if 0 <= cell_x < GRID_SIZE and 0 <= cell_y < GRID_SIZE:
                grid[cell_x][cell_y] = 1

    # Planificación de ruta
    start = Point(GRID_SIZE // 2, GRID_SIZE // 2)
    goal = Point(GRID_SIZE - 2, GRID_SIZE - 2)
    path = plan_path(grid, start, goal, MAX_PATH_LEN)

    # Movimiento si hay ruta
    if path:
        for i in range(4):
            wheels[i].setVelocity(left_speed if i % 2 == 0 else right_speed)

# Limpieza (Webots lo maneja automáticamente en Python, no es obligatorio)
