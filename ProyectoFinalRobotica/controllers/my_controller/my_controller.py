import math
from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display

# --- Constantes ---
TIME_STEP = 64
CELL_SIZE = 0.5
GRID_SIZE = 8
DISPLAY_WIDTH = 256
DISPLAY_HEIGHT = 256
SPEED = 6.0
ANGLE_THRESHOLD = 0.05
MOVEMENT_THRESHOLD = 0.05
TURN_ANGLE_RAD = math.pi / 2  # 90 grados

# Colores
COLOR_BACKGROUND = 0xFFFFFF
COLOR_OBSTACLE = 0xFF0000
COLOR_GOAL = 0x00FF00
COLOR_ROBOT = 0x0000FF

# Filtros para lidar
MIN_LIDAR_RANGE = 0.15
MAX_ROLL_PITCH = 0.2
NEIGHBOR_RADIUS_PX = 3
MIN_NEIGHBORS = 2

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
    px_world_units = (xw + world_map_width / 2) / CELL_SIZE
    py_world_units = (yw + world_map_height / 2) / CELL_SIZE
    px = int(px_world_units * (DISPLAY_WIDTH / GRID_SIZE))
    py = int(py_world_units * (DISPLAY_HEIGHT / GRID_SIZE))
    py = DISPLAY_HEIGHT - 1 - py
    return px, py

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
    print("Error: No se encontraron ruedas. Asegúrate de que los nombres coincidan en Webots.")
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
goal_reached = False  # Nuevo

while robot.step(TIME_STEP) != -1:
    pose = gps.getValues()
    robot_x = pose[0]
    robot_y = pose[1]

    imu_rpy = imu.getRollPitchYaw()
    roll = imu_rpy[0]
    pitch = imu_rpy[1]
    yaw = imu_rpy[2]

    # Verificar si se alcanzó la meta
    distance_to_goal = math.sqrt((robot_x - goal.x) ** 2 + (robot_y - goal.y) ** 2)
    if distance_to_goal < 0.1:
        goal_reached = True

    if goal_reached:
        for wheel in wheels:
            wheel.setVelocity(0.0)
        if lidar:
            lidar.disable()
        continue

    ranges = lidar.getRangeImage()
    resolution = lidar.getHorizontalResolution()
    fov = lidar.getFov()

    if prev_robot_x is None:
        prev_robot_x, prev_robot_y = robot_x, robot_y

    moved_distance = math.sqrt((robot_x - prev_robot_x) ** 2 + (robot_y - prev_robot_y) ** 2)

    if abs(roll) < MAX_ROLL_PITCH and abs(pitch) < MAX_ROLL_PITCH:
        if moved_distance > MOVEMENT_THRESHOLD:
            new_points = []
            for i in range(resolution):
                lidar_angle_relative = -fov / 2 + i * (fov / resolution)
                dist = ranges[i]
                if math.isinf(dist) or dist > lidar.getMaxRange() or dist < MIN_LIDAR_RANGE:
                    continue
                global_angle = yaw + lidar_angle_relative
                obs_x = robot_x + dist * math.cos(global_angle)
                obs_y = robot_y + dist * math.sin(global_angle)
                px, py = world2map(obs_x, obs_y)
                if 0 <= px < DISPLAY_WIDTH and 0 <= py < DISPLAY_HEIGHT:
                    new_points.append((px, py))

            filtered_points = []
            for (px, py) in new_points:
                neighbors = 0
                for (qx, qy) in new_points:
                    if abs(px - qx) <= NEIGHBOR_RADIUS_PX and abs(py - qy) <= NEIGHBOR_RADIUS_PX:
                        neighbors += 1
                    if neighbors >= MIN_NEIGHBORS:
                        break
                if neighbors >= MIN_NEIGHBORS:
                    filtered_points.append((px, py))

            for p in filtered_points:
                obstacle_map_pixels.add(p)

            prev_robot_x, prev_robot_y = robot_x, robot_y

    ds_detect_near = False
    for s in ds:
        if s.getValue() < 950.0:
            ds_detect_near = True
            break

    left_speed = 0.0
    right_speed = 0.0

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
        print("Configuración de ruedas no estándar, ajusta el código.")
        for wheel in wheels:
            wheel.setVelocity(0.0)

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
