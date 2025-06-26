from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display
import math

# --- Constantes ---
TIME_STEP = 64
CELL_SIZE = 0.5 # Tamaño de cada "celda" en el mapa, ajustar según el nivel de detalle deseado
GRID_SIZE = 8 # Tamaño del mapa en unidades de CELL_SIZE (8x8 celdas)
DISPLAY_WIDTH = 256 # Ancho del display en píxeles (idealmente un múltiplo de GRID_SIZE * (DISPLAY_WIDTH / GRID_SIZE) )
DISPLAY_HEIGHT = 256 # Alto del display en píxeles (idealmente un múltiplo de GRID_SIZE * (DISPLAY_HEIGHT / GRID_SIZE) )
SPEED = 6.0 # Velocidad lineal del robot
ANGLE_THRESHOLD = 0.05 # radianes para considerar alineado, valor más ajustado
GOAL_REACHED_THRESHOLD = 0.1 # Distancia para considerar que la meta ha sido alcanzada (en metros)

# --- Mapeo de colores para el display ---
COLOR_BACKGROUND = 0xFFFFFF # Blanco
COLOR_OBSTACLE = 0xFF0000 # Rojo
COLOR_GOAL = 0x00FF00 # Verde
COLOR_ROBOT = 0x0000FF # Azul
COLOR_TRAIL = 0x000000 # Negro (para el rastro del robot, opcional)

# --- Clase para puntos (simplificación) ---
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# --- Función para calcular la diferencia angular más corta ---
def angle_diff(target, current):
    a = target - current
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# --- Función para transformar coordenadas del mundo a coordenadas del display ---
def world2map(xw, yw):
    # Calcular las dimensiones del mundo que abarca el display
    world_map_width = GRID_SIZE * CELL_SIZE
    world_map_height = GRID_SIZE * CELL_SIZE # Asumiendo un mapa cuadrado

    # Calcular la posición en el "mundo de celdas" relativa al centro del mapa
    # Sumamos world_map_width/2 para desplazar el origen (0,0) al centro del área mapeada
    # Luego dividimos por CELL_SIZE para obtener la posición en unidades de celda
    px_world_units = (xw + world_map_width / 2) / CELL_SIZE
    py_world_units = (yw + world_map_height / 2) / CELL_SIZE

    # Escalar las unidades de celda a píxeles del display
    # (DISPLAY_WIDTH / GRID_SIZE) es cuántos píxeles ocupa cada "celda"
    px = int(px_world_units * (DISPLAY_WIDTH / GRID_SIZE))
    py = int(py_world_units * (DISPLAY_HEIGHT / GRID_SIZE))

    # Invertir el eje Y para que el Y positivo sea hacia arriba en el display (como en el mundo real)
    # y el origen (0,0) del display sea la esquina superior izquierda.
    py = DISPLAY_HEIGHT - 1 - py # Restar 1 para el índice de píxel, ya que DISPLAY_HEIGHT es el tamaño.

    return px, py

# --- Inicialización del robot ---
robot = Robot()

# Obtener dispositivos
# Ruedas
wheels = []
for i in range(1, 5): # Asume "wheel1", "wheel2", "wheel3", "wheel4"
    wheel_name = "wheel" + str(i)
    wheel = robot.getDevice(wheel_name)
    if wheel:
        wheel.setPosition(float('inf')) # Modo velocidad
        wheel.setVelocity(0.0)
        wheels.append(wheel)
    else:
        print(f"Advertencia: No se encontró la rueda {wheel_name}")

if not wheels:
    print("Error: No se encontraron ruedas. Asegúrate de que los nombres coincidan en Webots.")
    robot.step(0) # Salir si no hay ruedas
    exit()

# Sensores de distancia (opcional para navegación básica)
ds = []
ds_names = ["ds_left", "ds_right"] # Ajusta si tus sensores tienen otros nombres
for name in ds_names:
    s = robot.getDevice(name)
    if s:
        s.enable(TIME_STEP)
        ds.append(s)
    else:
        print(f"Advertencia: No se encontró el sensor de distancia {name}")

# Lidar (esencial para el mapeo)
lidar = robot.getDevice("lidar")
if lidar:
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud() # Necesario para obtener los datos de distancia (ranges)
else:
    print("Error: No se encontró el LIDAR. Asegúrate de que el nombre sea 'lidar' en Webots.")
    robot.step(0)
    exit()

# GPS (esencial para la posición global del robot)
gps = robot.getDevice("gps")
if gps:
    gps.enable(TIME_STEP)
else:
    print("Error: No se encontró el GPS. Asegúrate de que el nombre sea 'gps' en Webots.")
    robot.step(0)
    exit()

# Inertial Unit (IMU - esencial para la orientación del robot)
imu = robot.getDevice("inertial unit")
if imu:
    imu.enable(TIME_STEP)
else:
    print("Error: No se encontró la Unidad Inercial. Asegúrate de que el nombre sea 'inertial unit' en Webots.")
    robot.step(0)
    exit()

# Display (esencial para dibujar el mapa)
display = robot.getDevice("display")
if display:
    pass # Ya está habilitado por defecto si existe
else:
    print("Error: No se encontró el Display. Asegúrate de que el nombre sea 'display' en Webots.")
    robot.step(0)
    exit()

# --- Variables de estado ---
goal = Point(0.0, 0.0) # Posición de la meta inicial del robot
# Puedes cambiar la meta para que el robot explore diferentes áreas.
# Por ejemplo:
# goal = Point(2.0, 3.0)
# goal = Point(-1.0, -2.0)

# Almacenar los puntos de obstáculos detectados en el mapa
# Usamos un conjunto para evitar puntos duplicados y mejorar el rendimiento
# La clave es un tuple (px, py) de coordenadas del display
obstacle_map_pixels = set()

# --- Bucle principal de simulación ---
while robot.step(TIME_STEP) != -1:

    # 1. Lectura de Sensores
    # Posición del robot (X, Y, Z) - solo usamos X e Y
    pose = gps.getValues()
    robot_x = pose[0]
    robot_y = pose[1]

    # Orientación del robot (Roll, Pitch, Yaw) - solo usamos Yaw
    imu_rpy = imu.getRollPitchYaw()
    yaw = imu_rpy[2] # Ángulo de cabeceo (Yaw) en radianes

    # Lecturas del Lidar
    ranges = lidar.getRangeImage() # Array de distancias a los obstáculos
    resolution = lidar.getHorizontalResolution() # Número de lecturas horizontales
    fov = lidar.getFov() # Campo de visión del Lidar

    # Sensores de distancia (para evitar colisiones inmediatas)
    ds_detect_near = False
    for s in ds:
        if s.getValue() < 950.0: # Umbral de detección cercano, ajustar según tu robot
            ds_detect_near = True
            break

    # 2. Procesamiento del Lidar y Actualización del Mapa
    # Itera sobre cada lectura del Lidar para obtener la posición de los obstáculos
    for i in range(resolution):
        # Calcular el ángulo de esta lectura del Lidar respecto al frente del robot
        lidar_angle_relative = -fov / 2 + i * (fov / resolution)

        dist = ranges[i]
        
        # Ignorar lecturas infinitas (sin obstáculo en esa dirección) o fuera de un rango útil
        if math.isinf(dist) or dist > lidar.getMaxRange() or dist < 0.1: # Ajusta dist < 0.1 para evitar puntos demasiado cercanos/ruido
            continue

        # Calcular la posición absoluta del obstáculo en el mundo
        # Sumar el yaw del robot al ángulo relativo del LIDAR
        global_angle = yaw + lidar_angle_relative

        # Coordenadas X, Y del obstáculo en el sistema de coordenadas del mundo
        obs_x = robot_x + dist * math.cos(global_angle)
        obs_y = robot_y + dist * math.sin(global_angle)

        # Convertir las coordenadas del mundo a píxeles del display
        px, py = world2map(obs_x, obs_y)

        # Añadir el píxel del obstáculo al conjunto de puntos del mapa
        # Asegurarse de que el píxel esté dentro de los límites del display
        if 0 <= px < DISPLAY_WIDTH and 0 <= py < DISPLAY_HEIGHT:
            obstacle_map_pixels.add((px, py))

    # 3. Control de Movimiento del Robot (Ejemplo: Ir a la meta, evitar obstáculos)
    dx = goal.x - robot_x
    dy = goal.y - robot_y
    distance_to_goal = math.sqrt(dx**2 + dy**2) # Calcular distancia a la meta

    left_speed = 0.0
    right_speed = 0.0

    if distance_to_goal < GOAL_REACHED_THRESHOLD:
        # El robot ha llegado a la meta, detenerlo
        left_speed = 0.0
        right_speed = 0.0
        print(f"Meta alcanzada en ({robot_x:.2f}, {robot_y:.2f})!")
        # Opcional: Puedes romper el bucle aquí si quieres que la simulación termine
        # break
    else:
        # Si no ha llegado a la meta, continuar con la navegación
        target_angle = math.atan2(dy, dx) # Ángulo hacia la meta
        diff = angle_diff(target_angle, yaw) # Diferencia angular entre la orientación actual y la meta

        Kp_angular = 5.0 # Ganancia proporcional para el control angular
        turn_speed = Kp_angular * diff
        max_turn_speed = SPEED # Limitar la velocidad de giro
        turn_speed = max(-max_turn_speed, min(max_turn_speed, turn_speed))

        if ds_detect_near:
            left_speed = SPEED / 2
            right_speed = -SPEED / 2
        elif abs(diff) > ANGLE_THRESHOLD:
            left_speed = -turn_speed
            right_speed = turn_speed
        else:
            left_speed = SPEED
            right_speed = SPEED

    # Aplicar velocidades a las ruedas
    # Asume que las ruedas 1 y 3 son las izquierdas, 2 y 4 son las derechas (o ajusta según tu robot)
    if len(wheels) == 4: # Para un robot de 4 ruedas
        wheels[0].setVelocity(left_speed)  # wheel1 (frontal izquierda)
        wheels[1].setVelocity(right_speed) # wheel2 (frontal derecha)
        wheels[2].setVelocity(left_speed)  # wheel3 (trasera izquierda)
        wheels[3].setVelocity(right_speed) # wheel4 (trasera derecha)
    elif len(wheels) == 2: # Para un robot de 2 ruedas (diferencial)
        wheels[0].setVelocity(left_speed)
        wheels[1].setVelocity(right_speed)
    else:
        # Si tienes un número diferente de ruedas, ajusta aquí
        print("Configuración de ruedas no estándar, ajusta el código.")
        for wheel in wheels:
            wheel.setVelocity(0.0) # Detener el robot por seguridad

    # 4. Dibujar el Mapa en el Display
    # Limpiar el display
    display.setColor(COLOR_BACKGROUND)
    display.fillRectangle(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT)

    # Dibujar obstáculos (píxeles rojos)
    display.setColor(COLOR_OBSTACLE)
    for (px, py) in obstacle_map_pixels:
        # Dibujar un pequeño cuadrado para hacer los obstáculos más visibles
        display.fillRectangle(px, py, 2, 2) # Dibuja un cuadrado de 2x2 píxeles

    # Dibujar la meta (píxel verde)
    goal_px, goal_py = world2map(goal.x, goal.y)
    if 0 <= goal_px < DISPLAY_WIDTH and 0 <= goal_py < DISPLAY_HEIGHT:
        display.setColor(COLOR_GOAL)
        display.fillRectangle(goal_px, goal_py, 4, 4) # Dibuja un cuadrado de 4x4 píxeles

    # Dibujar el robot (píxel azul)
    robot_px, robot_py = world2map(robot_x, robot_y)
    if 0 <= robot_px < DISPLAY_WIDTH and 0 <= robot_py < DISPLAY_HEIGHT:
        display.setColor(COLOR_ROBOT)
        display.fillRectangle(robot_px, robot_py, 4, 4) # Dibuja un cuadrado de 4x4 píxeles

    # Opcional: Dibujar el rastro del robot (píxeles negros)
    # Esto puede sobrecargar el display con el tiempo, úsalo con precaución o vacía el mapa cada cierto tiempo
    # display.setColor(COLOR_TRAIL)
    # display.fillRectangle(robot_px, robot_py, 1, 1)