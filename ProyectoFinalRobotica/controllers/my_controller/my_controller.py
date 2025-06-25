from controller import Robot, Motor, DistanceSensor, Lidar, GPS, InertialUnit, Display
import math

# --- Constantes ---
TIME_STEP = 64
GRID_SIZE = 10
CELL_SIZE = 0.5
SPEED = 5.0
GOAL_CELL = (9, 0) # This is a grid cell (x, y)
OBSTACLE_MAPPING_DIST = 1.5
IMMINENT_COLLISION_DIST = 0.25
KP, KI, KD = 2.8, 0.05, 0.5

# --- Estados ---
STATE_FOLLOW_PATH = 0
STATE_TURN = 1
STATE_STOP = 2
STATE_MOVE_FORWARD_AFTER_TURN = 3 # <--- NEW STATE

class Point:
    def __init__(self, x, y): self.x, self.y = x, y

class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x, self.y, self.g, self.h, self.parent = x, y, g, h, parent
        self.f = g + h

# — Inicialización Robot y dispositivos —
robot = Robot()
# Motores
motors = []
for name in ["motor1","motor2","motor3","motor4"]:
    m = robot.getDevice(name)
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    motors.append(m)
# Sensores
ds_left  = robot.getDevice("distIzq");  ds_left.enable(TIME_STEP)
ds_right = robot.getDevice("distDer");  ds_right.enable(TIME_STEP)
gps      = robot.getDevice("gps");      gps.enable(TIME_STEP)
imu      = robot.getDevice("imu");      imu.enable(TIME_STEP)
lidar    = robot.getDevice("lidar");    lidar.enable(TIME_STEP); lidar.enablePointCloud()
display  = robot.getDevice("display")
cell_px  = display.getWidth() // GRID_SIZE
offset_x = (display.getWidth()  - cell_px * GRID_SIZE)//2
offset_y = (display.getHeight() - cell_px * GRID_SIZE)//2

# — Variables globales —
grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
path = []
state = STATE_FOLLOW_PATH
previous_cell = None
# Variables PID
integral_error = 0.0
previous_error = 0.0
# Para giro en sitio
turn_target_yaw = None
# Variables for STATE_MOVE_FORWARD_AFTER_TURN
forward_start_pos = None # To track how much it has moved forward

yaw = 0.0 # <--- CRITICAL FIX: Initialize yaw globally to prevent NameError

# — Funciones auxiliares —
def heuristic(a,b): return abs(a.x-b.x)+abs(a.y-b.y)

def plan_path(grid, start, goal):
    open_list = [Node(start.x,start.y,0,heuristic(start,goal))]
    closed = set()
    while open_list:
        open_list.sort(key=lambda n: n.f)
        current = open_list.pop(0)
        if (current.x,current.y)==(goal.x,goal.y):
            # reconstruct path
            rev=[]
            n=current
            while n:
                rev.append(Point(n.x,n.y))
                n=n.parent
            return rev[::-1]
        closed.add((current.x,current.y))
        for dx,dy in [(0,1),(1,0),(0,-1),(-1,0)]: # 4-directional movement
            nx,ny=current.x+dx,current.y+dy
            if not (0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE): continue
            if grid[nx][ny]==1 or (nx,ny) in closed: continue # Check for obstacles (1)
            neigh=Node(nx,ny,current.g+1,heuristic(Point(nx,ny),goal),current)
            exists = next((n for n in open_list if (n.x,n.y)==(nx,ny)), None)
            if exists and exists.f <= neigh.f: continue
            if exists: open_list.remove(exists)
            open_list.append(neigh)
    return []

def gps_to_cell(x,z):
    half = GRID_SIZE*CELL_SIZE/2.0
    # Assuming robot's 'x' corresponds to grid 'x', and 'z' to grid 'y' (inverted for typical Webots view)
    cx = int((x + half)/CELL_SIZE)
    cy = int((half - z)/CELL_SIZE) # Invert z to match display's y-axis (top-left is 0,0)
    cx = max(0,min(GRID_SIZE-1,cx))
    cy = max(0,min(GRID_SIZE-1,cy))
    return cx,cy

def normalize(a):
    while a>math.pi: a-=2*math.pi
    while a<=-math.pi: a+=2*math.pi
    return a

def set_vel(l,r):
    motors[0].setVelocity(l)
    motors[1].setVelocity(r)
    motors[2].setVelocity(l)
    motors[3].setVelocity(r)

def stop(): set_vel(0,0)

def update_display(cell, path_to_draw):
    display.setColor(0x000000)
    display.fillRectangle(0,0,display.getWidth(),display.getHeight())
    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            px,py = offset_x+x*cell_px, offset_y+y*cell_px
            if (x,y)==GOAL_CELL: color=0x00FF00 # Goal
            elif grid[x][y]==1:    color=0xFF0000 # Obstacle
            elif grid[x][y]==2:    color=0x505050 # Visited
            else:                  color=0xFFFFFF # Empty
            display.setColor(color)
            display.fillRectangle(px,py,cell_px,cell_px)
            display.setColor(0x000000)
            display.drawRectangle(px,py,cell_px,cell_px)
    # Path
    display.setColor(0x0000FF)
    for p in path_to_draw:
        # Draw small blue square for path points
        display.fillRectangle(offset_x+p.x*cell_px+cell_px//4,
                              offset_y+p.y*cell_px+cell_px//4,
                              cell_px//2,cell_px//2)
    # Robot
    display.setColor(0xFFFF00) # Yellow
    display.fillOval(offset_x+cell[0]*cell_px+cell_px//2,
                     offset_y+cell[1]*cell_px+cell_px//2,
                     cell_px//3,cell_px//3)

# — Bucle principal —
while robot.step(TIME_STEP)!=-1:
    # Lectura sensores
    rx,ry,rz = gps.getValues()
    yaw = imu.getRollPitchYaw()[2] # yaw is updated here every step
    cx,cy = gps_to_cell(rx,ry) # Correct: Use ry for the cell's Y-coordinate
    cell = (cx,cy)

    # --- Debugging Print Statements (Uncomment these to get detailed feedback in the Webots console) ---
    # print(f"Time: {robot.getTime():.2f}s | State: {state}")
    # print(f"  GPS: ({rx:.2f}, {ry:.2f}, {ry:.2f}) | Cell: ({cx}, {cy}) | Prev Cell: {previous_cell}")
    # print(f"  Yaw: {math.degrees(yaw):.1f}°")
    # if path:
    #     wp = path[0]
    #     # Convert waypoint cell coordinates to world coordinates for distance calculation
    #     tx_wp_world = wp.x*CELL_SIZE + CELL_SIZE/2 - (GRID_SIZE*CELL_SIZE)/2
    #     tz_wp_world = (GRID_SIZE*CELL_SIZE)/2 - (wp.y*CELL_SIZE + CELL_SIZE/2)
    #     wp_dist = math.hypot(tx_wp_world - rx, tz_wp_world - ry)
    #     print(f"  Path Len: {len(path)} | Next WP: ({wp.x}, {wp.y}) | WP Dist: {wp_dist:.2f}")
    # else:
    #     print("  Path is empty.")
    #
    # lidar_ranges = lidar.getRangeImage()
    # current_collision_imminent = any(0<d<IMMINENT_COLLISION_DIST for d in lidar_ranges)
    # print(f"  Collision Imminent: {current_collision_imminent} | Mapped new: {mapped}")
    # if state == STATE_MOVE_FORWARD_AFTER_TURN and forward_start_pos:
    #     current_fwd_dist = math.hypot(rx - forward_start_pos.x, ry - forward_start_pos.y)
    #     print(f"  Forward Dist After Turn: {current_fwd_dist:.2f}")
    # print("-" * 40)


    # Meta alcanzada
    if cell==GOAL_CELL:
        stop()
        print("¡Meta alcanzada!")
        update_display(cell,path)
        break

    # Marcar visitado
    # Only mark if it's currently empty, not if it's an obstacle or already visited
    if grid[cx][cy]==0:
        grid[cx][cy]=2

    # Mapeo obstáculo con Lidar
    lidar_ranges = lidar.getRangeImage()
    collision_imminent = any(0<d<IMMINENT_COLLISION_DIST for d in lidar_ranges)
    mapped=False
    for i,d in enumerate(lidar_ranges):
        if 0 < d < OBSTACLE_MAPPING_DIST: # Check d > 0.0 to ignore infinite ranges (already implied by 0<d)
            rel_angle = i*lidar.getFov()/lidar.getHorizontalResolution() - lidar.getFov()/2
            glob_angle = normalize(yaw+rel_angle)
            ox,oz = rx+math.cos(glob_angle)*d, ry+math.sin(glob_angle)*d
            ocx,ocy = gps_to_cell(ox,oz)
            # Check if obstacle cell is valid and not already mapped as obstacle or visited
            if 0<=ocx<GRID_SIZE and 0<=ocy<GRID_SIZE and grid[ocx][ocy] == 0:
                grid[ocx][ocy]=1
                mapped=True
                # print(f"Mapped new obstacle at ({ocx}, {ocy}) from Lidar range {d:.2f}") # Debug new obstacle mapping

    # --- Path Replanning Logic ---
    should_replan = False
    if mapped:
        should_replan = True
    elif previous_cell != cell:
        should_replan = True
    elif not path: # No current path
        should_replan = True
    # The new STATE_MOVE_FORWARD_AFTER_TURN ensures cell change which triggers replan.

    if should_replan:
        # print("REPLANNING PATH!")
        path = plan_path(grid, Point(cx,cy), Point(*GOAL_CELL))
        # If the current cell is the first point in the new path, it means we are already there, pop it.
        if path and path[0].x == cx and path[0].y == cy:
            path.pop(0)
        if not path and cell != GOAL_CELL: # If no path is found and not at goal, stop
             print("No path found to goal! Stopping.")
             state = STATE_STOP # Consider adding a stop state for this case
             stop()

    # If there is an imminent collision and we are not already handling a turn or forward move
    if collision_imminent and state == STATE_FOLLOW_PATH:
        state = STATE_TURN
        # You might want to decide the turn direction more intelligently here
        # For simplicity, keeping it 90 degrees left.
        turn_target_yaw = normalize(yaw + math.pi/2) # turn 90 degrees left
        stop()
        # print(f"Entering STATE_TURN. Target Yaw: {math.degrees(turn_target_yaw):.1f}°")

    # --- State Machine Logic ---
    if state == STATE_TURN:
        err = normalize(turn_target_yaw - yaw)
        if abs(err) < 0.05: # Angle tolerance for completing turn
            state = STATE_MOVE_FORWARD_AFTER_TURN # <--- Transition to new state
            forward_start_pos = Point(rx, ry) # Store current position (X and Z)
            stop()
            integral_error = previous_error = 0.0 # Reset PID
            # print("Turn complete. Entering STATE_MOVE_FORWARD_AFTER_TURN.")
        else:
            v = SPEED * 0.6 # Slower turn speed
            if err > 0: set_vel(-v, v) # Turn left (positive error = turn counter-clockwise)
            else:       set_vel(v, -v) # Turn right (negative error = turn clockwise)

    elif state == STATE_MOVE_FORWARD_AFTER_TURN:
        # Calculate distance moved in the XZ plane
        current_pos_dist = math.hypot(rx - forward_start_pos.x, ry - forward_start_pos.y)
        # Move forward a fixed small distance, e.g., 0.7 * CELL_SIZE
        if current_pos_dist < CELL_SIZE * 0.7:
            set_vel(SPEED * 0.7, SPEED * 0.7) # Move slower to allow GPS to catch up and register cell change
            # print(f"Moving forward after turn. Dist moved: {current_pos_dist:.2f}")
        else:
            state = STATE_FOLLOW_PATH # <--- Return to path following
            stop()
            # print("Moved forward enough. Returning to STATE_FOLLOW_PATH.")
            # The cell change from this movement will trigger a replan in the next cycle.

    elif state == STATE_FOLLOW_PATH:
        if not path: # No path, maybe goal unreachable or already at goal.
            # This should ideally be caught by the 'should_replan' logic above if no path is found
            # but as a fallback:
            stop()
            # print("No path to follow in STATE_FOLLOW_PATH. Stopping.")
            state = STATE_STOP
        else:
            # Waypoint actual
            wp = path[0]
            # Convert waypoint cell coordinates to world coordinates (tx, tz)
            tx = wp.x*CELL_SIZE + CELL_SIZE/2 - (GRID_SIZE*CELL_SIZE)/2
            tz = (GRID_SIZE*CELL_SIZE)/2 - (wp.y*CELL_SIZE + CELL_SIZE/2)
            dx,dz = tx-rx, tz-ry # Vector from robot to waypoint
            dist_to_wp = math.hypot(dx,dz) # Distance to waypoint
            desired_yaw = math.atan2(dz,dx) # Desired angle to waypoint
            err = normalize(desired_yaw - yaw) # Angle error

            # PID Control for steering
            integral_error += err*(TIME_STEP/1000.0)
            derivative = (err - previous_error)/(TIME_STEP/1000.0)
            turn_correction = KP*err + KI*integral_error + KD*derivative
            previous_error = err

            # Calculate wheel velocities, limiting to SPEED
            vl = max(0, min(SPEED, SPEED - turn_correction))
            vr = max(0, min(SPEED, SPEED + turn_correction))
            set_vel(vl,vr)

            # Check if waypoint is reached
            if dist_to_wp < CELL_SIZE * 0.4: # Tolerance for reaching waypoint
                path.pop(0)
                # print(f"Waypoint ({wp.x}, {wp.y}) reached. Path remaining: {len(path)}")

    elif state == STATE_STOP:
        stop()
        # print("Robot is in STOP state.")
        pass # Robot just stays stopped


    update_display(cell, path)
    previous_cell = cell # Update previous_cell at the end of the loop