from controller import Robot, Motor, DistanceSensor, Lidar, GPS
import math

TIME_STEP = 64
GRID_SIZE = 10
CELL_SIZE = 1.0
MAX_PATH_LEN = 100
MAX_OPEN_NODES = 1000
SPEED = 4.0

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    def __init__(self, x, y, g, h, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent_index = parent_index

def heuristic(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def plan_path(grid, start, goal, max_path_len):
    open_list = []
    closed_list = []

    start_node = Node(start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), -1)
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

        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            nx, ny = current.x + dx, current.y + dy
            if nx < 0 or ny < 0 or nx >= GRID_SIZE or ny >= GRID_SIZE:
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
            else:
                open_list.append(Node(nx, ny, g, h, len(closed_list) - 1))
    return []

# === Inicio del robot ===
robot = Robot()

# Motores
wheels = []
for name in ["motor1", "motor2", "motor3", "motor4"]:
    motor = robot.getDevice(name)
    motor.setPosition(float('inf'))
    wheels.append(motor)

# Sensores
ds = []
for name in ["distIzq", "distDer"]:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ds.append(sensor)

# LIDAR
lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# GPS
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# Display
display = robot.getDevice("display")
cell_pixel_size = display.getWidth() // GRID_SIZE

# Grilla y planificación
grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
path = []

avoid_obstacle_counter = 0

while robot.step(TIME_STEP) != -1:
    left_speed = 10.0
    right_speed = 10.0

    if avoid_obstacle_counter > 0:
        avoid_obstacle_counter -= 1
        left_speed = 5.0
        right_speed = -5.0
    else:
        ds_values = [sensor.getValue() for sensor in ds]
        if ds_values[0] < 950.0 or ds_values[1] < 950.0:
            avoid_obstacle_counter = 10

    # GPS
    pose = gps.getValues()
    robot_x = pose[0]
    robot_y = pose[2]

    # LIDAR
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
                if grid[cell_x][cell_y] == 0:
                    grid[cell_x][cell_y] = 1
                    # Dibuja obstáculo en rojo (corrigiendo eje Y)
                    display.setColor(0xFF0000)
                    px = cell_x * cell_pixel_size
                    py = display.getHeight() - (cell_y + 1) * cell_pixel_size
                    display.fillRectangle(px, py, cell_pixel_size, cell_pixel_size)

    # Planificación de ruta (no se dibuja)
    start = Point(GRID_SIZE // 2, GRID_SIZE // 2)
    goal = Point(GRID_SIZE - 2, GRID_SIZE - 2)
    path = plan_path(grid, start, goal, MAX_PATH_LEN)

    # Motores
    for i, wheel in enumerate(wheels):
        if i in [0, 3]:
            wheel.setVelocity(left_speed)
        else:
            wheel.setVelocity(right_speed)
