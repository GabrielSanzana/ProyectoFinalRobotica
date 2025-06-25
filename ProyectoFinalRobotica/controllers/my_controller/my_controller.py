from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display
import math

# --- Constantes ---
TIME_STEP = 64
GRID_SIZE = 10
CELL_SIZE = 0.5
SPEED = 4.0
GOAL_CELL = (5, 0)
OBSTACLE_MAPPING_DIST = 1.5
IMMINENT_COLLISION_DIST = 0.2
KP, KI, KD = 2.8, 0.05, 0.5

# --- Estados ---
STATE_FOLLOW_PATH = 0
STATE_TURN = 1
STATE_STOP = 2
STATE_MOVE_FORWARD_AFTER_TURN = 3

class Point:
    def __init__(self, x, y): self.x, self.y = x, y

class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x, self.y, self.g, self.h, self.parent = x, y, g, h, parent
        self.f = g + h

# — Inicialización Robot y dispositivos —
robot = Robot()
motors = []
for name in ["motor1","motor2","motor3","motor4"]:
    m = robot.getDevice(name)
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    motors.append(m)

ds_left  = robot.getDevice("distIzq");  ds_left.enable(TIME_STEP)
ds_right = robot.getDevice("distDer");  ds_right.enable(TIME_STEP)
gps      = robot.getDevice("gps");      gps.enable(TIME_STEP)
imu      = robot.getDevice("imu");      imu.enable(TIME_STEP)
lidar    = robot.getDevice("lidar");    lidar.enable(TIME_STEP); lidar.enablePointCloud()
display  = robot.getDevice("display")
cell_px  = display.getWidth() // GRID_SIZE
offset_x = (display.getWidth()  - cell_px * GRID_SIZE)//2
offset_y = (display.getHeight() - cell_px * GRID_SIZE)//2

# Variables para métricas
start_navigation_time = robot.getTime()
path_planning_time = 0.0

# — Variables globales —
grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
path = []
state = STATE_FOLLOW_PATH
previous_cell = None
yaw = 0.0
turn_target_yaw = None
forward_start_pos = None
integral_error = 0.0
previous_error = 0.0
total_path_length = None
total_path_distance = None

def heuristic(a,b): return ((a.x-b.x)**2+(a.y-b.y)**2)**0.5

def plan_path(grid, start, goal):
    open_list = [Node(start.x,start.y,0,heuristic(start,goal))]
    closed = set()
    while open_list:
        open_list.sort(key=lambda n: n.f)
        current = open_list.pop(0)
        if (current.x,current.y)==(goal.x,goal.y):
            rev=[]
            n=current
            while n:
                rev.append(Point(n.x,n.y))
                n=n.parent
            return rev[::-1]
        closed.add((current.x,current.y))
        for dx,dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            nx,ny=current.x+dx,current.y+dy
            if not (0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE): continue
            if grid[nx][ny]==1 or (nx,ny) in closed: continue
            neigh=Node(nx,ny,current.g+1,heuristic(Point(nx,ny),goal),current)
            exists = next((n for n in open_list if (n.x,n.y)==(nx,ny)), None)
            if exists and exists.f <= neigh.f: continue
            if exists: open_list.remove(exists)
            open_list.append(neigh)
    return []

def gps_to_cell(x,z):
    half = GRID_SIZE*CELL_SIZE/2.0
    cx = int((x + half)/CELL_SIZE)
    cy = int((half - z)/CELL_SIZE)
    return max(0,min(GRID_SIZE-1,cx)), max(0,min(GRID_SIZE-1,cy))

def normalize(a):
    while a>math.pi: a-=2*math.pi
    while a<=-math.pi: a+=2*math.pi
    return a

def set_vel(l,r):
    for i in [0,2]: motors[i].setVelocity(l)
    for i in [1,3]: motors[i].setVelocity(r)

def stop(): set_vel(0,0)

def update_display(cell, path_to_draw):
    display.setColor(0x000000)
    display.fillRectangle(0,0,display.getWidth(),display.getHeight())
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            px,py = offset_x+x*cell_px, offset_y+y*cell_px
            color = 0xFFFFFF
            if (x,y)==GOAL_CELL: color=0x00FF00
            elif grid[x][y]==1: color=0xFF0000
            elif grid[x][y]==2: color=0x505050
            display.setColor(color)
            display.fillRectangle(px,py,cell_px,cell_px)
            display.setColor(0x000000)
            display.drawRectangle(px,py,cell_px,cell_px)
    display.setColor(0x0000FF)
    for p in path_to_draw:
        display.fillRectangle(offset_x+p.x*cell_px+cell_px//4,
                              offset_y+p.y*cell_px+cell_px//4,
                              cell_px//2,cell_px//2)
    display.setColor(0xFFFF00)
    display.fillOval(offset_x+cell[0]*cell_px+cell_px//2,
                     offset_y+cell[1]*cell_px+cell_px//2,
                     cell_px//3,cell_px//3)

while robot.step(TIME_STEP)!=-1:
    rx,ry,rz = gps.getValues()
    yaw = imu.getRollPitchYaw()[2]
    cx,cy = gps_to_cell(rx,ry)
    cell = (cx,cy)

    if cell==GOAL_CELL:
        stop()
        end_navigation_time = robot.getTime()
        total_navigation_time = end_navigation_time - start_navigation_time
        explored_cells = sum(row.count(1) + row.count(2) for row in grid)
        total_cells = GRID_SIZE * GRID_SIZE
        explored_percentage = (explored_cells / total_cells) * 100

        print("\n¡Meta alcanzada!")
        print(f"Tiempo total de navegación: {total_navigation_time:.2f} segundos")
        if total_path_length is not None:
            print(f"[A*] Tiempo de planificación: {path_planning_time:.2f} milisegundos")
            print(f"[A*] Longitud total del path (celdas): {total_path_length}")
            print(f"[A*] Distancia total (metros): {total_path_distance:.2f}")
        print(f"Porcentaje del mapa explorado: {explored_percentage:.2f} %")
        update_display(cell,path)
        break

    if grid[cx][cy]==0:
        grid[cx][cy]=2

    lidar_ranges = lidar.getRangeImage()
    collision_imminent = any(0<d<IMMINENT_COLLISION_DIST for d in lidar_ranges)
    mapped=False
    for i,d in enumerate(lidar_ranges):
        if 0 < d < OBSTACLE_MAPPING_DIST:
            rel_angle = i*lidar.getFov()/lidar.getHorizontalResolution() - lidar.getFov()/2
            glob_angle = normalize(yaw+rel_angle)
            ox,oz = rx+math.cos(glob_angle)*d, ry+math.sin(glob_angle)*d
            ocx,ocy = gps_to_cell(ox,oz)
            if 0<=ocx<GRID_SIZE and 0<=ocy<GRID_SIZE and grid[ocx][ocy] == 0:
                grid[ocx][ocy]=1
                mapped=True

    should_replan = mapped or previous_cell != cell or not path
    if should_replan:
        start_time = robot.getTime()
        path = plan_path(grid, Point(cx, cy), Point(*GOAL_CELL))
        end_time = robot.getTime()
        path_planning_time = (end_time - start_time) * 1000  # ms
        elapsed_ms = path_planning_time
        if path and path[0].x == cx and path[0].y == cy:
            path.pop(0)
        if not path and cell != GOAL_CELL:
            print("No path found to goal! Stopping.")
            state = STATE_STOP
            stop()
        if total_path_length is None:
            total_path_length = len(path) + 1
            total_path_distance = total_path_length * CELL_SIZE
            print(f"[A*] Tiempo de planificación: {elapsed_ms:.2f} ms")
            print(f"[A*] Longitud total del path (celdas): {total_path_length}")
            print(f"[A*] Distancia total (metros): {total_path_distance:.2f}")

    if collision_imminent and state == STATE_FOLLOW_PATH:
        state = STATE_TURN
        turn_target_yaw = normalize(yaw + math.pi/2)
        stop()

    if state == STATE_TURN:
        err = normalize(turn_target_yaw - yaw)
        if abs(err) < 0.05:
            state = STATE_MOVE_FORWARD_AFTER_TURN
            forward_start_pos = Point(rx, ry)
            stop()
            integral_error = previous_error = 0.0
        else:
            v = SPEED * 0.6
            set_vel(-v if err > 0 else v, v if err > 0 else -v)

    elif state == STATE_MOVE_FORWARD_AFTER_TURN:
        if math.hypot(rx - forward_start_pos.x, ry - forward_start_pos.y) < CELL_SIZE * 0.7:
            set_vel(SPEED * 0.7, SPEED * 0.7)
        else:
            state = STATE_FOLLOW_PATH
            stop()

    elif state == STATE_FOLLOW_PATH:
        if not path:
            stop()
            state = STATE_STOP
        else:
            wp = path[0]
            tx = wp.x*CELL_SIZE + CELL_SIZE/2 - (GRID_SIZE*CELL_SIZE)/2
            tz = (GRID_SIZE*CELL_SIZE)/2 - (wp.y*CELL_SIZE + CELL_SIZE/2)
            dx,dz = tx-rx, tz-ry
            dist_to_wp = math.hypot(dx,dz)
            desired_yaw = math.atan2(dz,dx)
            err = normalize(desired_yaw - yaw)
            integral_error += err*(TIME_STEP/1000.0)
            derivative = (err - previous_error)/(TIME_STEP/1000.0)
            turn_correction = KP*err + KI*integral_error + KD*derivative
            previous_error = err
            vl = max(0, min(SPEED, SPEED - turn_correction))
            vr = max(0, min(SPEED, SPEED + turn_correction))
            set_vel(vl,vr)
            if dist_to_wp < CELL_SIZE * 0.4:
                path.pop(0)

    elif state == STATE_STOP:
        stop()

    update_display(cell, path)
    previous_cell = cell
