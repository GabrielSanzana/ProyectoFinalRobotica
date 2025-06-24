from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display
import math

# Constantes
TIME_STEP = 64
GRID_SIZE = 6
CELL_SIZE = 0.5          # metros
SPEED = 10.0             # velocidad de avance
OBSTACLE_THRESHOLD = 1.5 # metros (lidar)
TURN_ANGLE = math.pi / 4 # 45 grados en radianes

# Coordenadas de la meta (celda)
GOAL_CELL = (5, 0)

# Estados
STATE_MOVE = 0
STATE_TURN = 1

# Inicialización del robot
robot = Robot()

# Motores
motors = []
for name in ["motor1","motor2","motor3","motor4"]:
    m = robot.getDevice(name)
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    motors.append(m)

# Sensores
ds_left = robot.getDevice("distIzq"); ds_left.enable(TIME_STEP)
ds_right = robot.getDevice("distDer"); ds_right.enable(TIME_STEP)
gps     = robot.getDevice("gps");     gps.enable(TIME_STEP)
lidar   = robot.getDevice("lidar");   lidar.enable(TIME_STEP)
imu     = robot.getDevice("imu"); imu.enable(TIME_STEP)

# Display para el mapa
display   = robot.getDevice("display")
cell_px   = display.getWidth() // GRID_SIZE
offset_x  = (display.getWidth() - cell_px * GRID_SIZE) // 2
offset_y  = (display.getHeight() - cell_px * GRID_SIZE) // 2

# Mapa y planificación
grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
# (Se asume que ya tienes la función plan_path heredada de tu código anterior)

# Variables de estado
state = STATE_MOVE
start_yaw = 0.0
target_yaw = 0.0
turn_dir = 1  # +1=giro izq, -1=giro der

def gps_to_cell(x,y):
    cx = int((x + (GRID_SIZE*CELL_SIZE)/2)//CELL_SIZE)
    cy = int((y + (GRID_SIZE*CELL_SIZE)/2)//CELL_SIZE)
    return cx,cy

def normalize(angle):
    # normaliza a [-pi,pi)
    while angle > math.pi:   angle -= 2*math.pi
    while angle <= -math.pi: angle += 2*math.pi
    return angle

def set_wheel_velocity(v0,v1,v2,v3):
    motors[0].setVelocity(v0)
    motors[1].setVelocity(v1)
    motors[2].setVelocity(v2)
    motors[3].setVelocity(v3)

def stop():
    set_wheel_velocity(0,0,0,0)

def move_forward():
    set_wheel_velocity(SPEED, SPEED, SPEED, SPEED)

def start_turn(direction):
    global start_yaw, target_yaw, state, turn_dir
    # dirección: +1 para izquierda, -1 para derecha
    yaw = imu.getRollPitchYaw()[2]
    start_yaw = yaw
    target_yaw = normalize(start_yaw + direction * TURN_ANGLE)
    turn_dir = direction
    state = STATE_TURN
    # girar en su lugar
    v = SPEED * direction
    set_wheel_velocity(-v, v, -v, v)

# Bucle principal
while robot.step(TIME_STEP) != -1:
    # 1) Leer posición y orientación
    rx, ry = gps.getValues()[0], gps.getValues()[1]
    yaw = imu.getRollPitchYaw()[2]
    cell = gps_to_cell(rx, ry)

    # 2) Actualizar mapa y dibujar (igual que antes)
    if 0 <= cell[0] < GRID_SIZE and 0 <= cell[1] < GRID_SIZE:
        grid[cell[0]][cell[1]] = 2

    # 3) Meta alcanzada
    if cell == GOAL_CELL:
        stop()
        print("¡Meta alcanzada!")
        break

    # 4) Obstáculos con LIDAR
    ranges     = lidar.getRangeImage()
    resolution = lidar.getHorizontalResolution()
    fov        = lidar.getFov()
    for i in range(resolution):
        angle = -fov/2 + i*(fov/resolution)
        dist  = ranges[i]
        if 0.01 < dist < OBSTACLE_THRESHOLD:
            ox = rx + dist * math.cos(angle)
            oy = ry + dist * math.sin(angle)
            cx,cy = gps_to_cell(ox,oy)
            if 0 <= cx < GRID_SIZE and 0 <= cy < GRID_SIZE:
                grid[cx][cy] = 1

    # 5) Detectar muro por distancia
    hit_wall = (ds_left.getValue() < 980 or ds_right.getValue() < 980)

    # 6) Estado de movimiento
    if state == STATE_MOVE:
        # si choca, iniciamos giro en dirección opuesta a la última
        if hit_wall:
            start_turn(-turn_dir)
        else:
            move_forward()

        # además, si estamos alineados a un nuevo paso del A*, preparamos un giro
        # (calcula la próxima celda del camino)
        path = plan_path(grid, Point(cell[0],cell[1]),
                         Point(GOAL_CELL[0],GOAL_CELL[1]), MAX_PATH_LEN=100)
        if len(path) >= 2:
            # vector objetivo
            nx,ny = path[1].x - cell[0], path[1].y - cell[1]
            desired = math.atan2(ny, nx)
            diff = normalize(desired - yaw)
            if abs(diff) > 0.1:
                # gira siempre 45°, en la dirección que acerque al objetivo
                start_turn(1 if diff > 0 else -1)

    # 7) Estado de giro
    else:  # STATE_TURN
        # seguimos girando hasta alcanzar target_yaw
        diff = normalize(yaw - start_yaw)
        if abs(normalize(yaw - target_yaw)) < 0.02 or abs(diff) >= TURN_ANGLE:
            # fin de giro: volver a mover
            state = STATE_MOVE
            stop()

