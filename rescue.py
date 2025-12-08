import cozmo
from cozmo.util import degrees, radians, Angle, distance_mm, Pose
from cozmo.objects import LightCube, LightCube1Id, LightCube2Id, LightCube3Id
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from frame2d import Frame2D

WHEEL_SPEED = 250
DISTANCE_PER_MOVE = 100
CALIBRATION_TIME = 1.0
IMAGE_DIFF_THRESHOLD = 8
CALIBRATED_CONSTANT = 2.25

WALL_RADIUS=100
WALL_THRESHOLD = 50
ANGLE_THRESHOLD = 0.05
MIN_ROTATION_SEC = 0.30  


#Tracking variables
map = dict()
path = []
walls = []
marked_walls_seen=[]
walls_angles = []
cubes = {cozmo.objects.LightCube1Id: [False, None],
         cozmo.objects.LightCube2Id: [False, None],
         cozmo.objects.LightCube3Id: [False, None]}
start_time = time.time()
MOVE_DISTANCE = 400  #Distance to move per step
RANGE_MAP_KEY = 20
WALL_RADIUS = 100
RADIUS_CIRCLES = 300

#Setup the plot
fig, ax = plt.subplots(figsize=(10, 10))
plt.ion()
plt.show()

### NAVIGATION ###
def add_position(x, y):
    global path
    path.append((x, y))
def normalise_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def check_for_wall(robot, move_duration):
    img1 = robot.world.latest_image
    if img1 is None:
        time.sleep(0.1)
        return False
    
    gray1 = np.array(img1.raw_image.convert('L'))
    
    robot.drive_wheels(WHEEL_SPEED, WHEEL_SPEED, duration=move_duration)
    time.sleep(move_duration)
    
    img2 = robot.world.latest_image
    if img2 is None:
        return False
    
    gray2 = np.array(img2.raw_image.convert('L'))
    difference = np.mean(np.abs(gray1.astype(float) - gray2.astype(float)))
    
    if difference < IMAGE_DIFF_THRESHOLD:
        robot.drive_wheels(0, 0)
        print(f"Wall detected! Difference: {difference:.2f}")

        x,y = get_current_pos(robot)
        add_wall(x, y)
        add_position(x,y)
        
        robot.drive_wheels(-WHEEL_SPEED, -WHEEL_SPEED, duration=move_duration)
        time.sleep(move_duration)
        robot.drive_wheels(0, 0)
        time.sleep(0.2)
        return True
    
    return False

def isLeft(a, b, c):
    return (b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0]) > 0

def calculate_next_target(px, py, x1, y1, x2, y2):
    """Return shortest distance from point (px,py) to segment (x1,y1)-(x2,y2).
    All units are assumed to be mm.
    """
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    seg_len_sq = vx*vx + vy*vy
    if seg_len_sq == 0:
        return False, None, None
    t = (wx*vx + wy*vy) / seg_len_sq
    if t <= 0:
        print("Wall before initial point")
        return False, None, None
    elif t >= 1:
        print("Wall after target point")
        return False, None, None
    
    cx = x1 + t * vx
    cy = y1 + t * vy

    if math.hypot(px - cx, py - cy) > WALL_RADIUS:
        print("Wall too far from segment " + str(math.hypot(px - cx, py - cy)))
        return False, None, None
    
    absolute_angle = math.atan2(vy, vx)

    sign = 1
    print(isLeft((x1, y1), (x2, y2), (px, py)))
    if isLeft((x1, y1), (x2, y2), (px, py)) == False:
        print("Wall on the right")
        sign = 1
    else: 
        print("Wall on the left")
        sign = -1
    
    print("absolute angle of segment: " + str(absolute_angle))

    target_x = px + sign * (WALL_RADIUS+WALL_THRESHOLD) * math.cos(absolute_angle+math.pi/2)
    target_y = py + sign * (WALL_RADIUS+WALL_THRESHOLD) * math.sin(absolute_angle+math.pi/2)
    
    return True, target_x, target_y

def navigate_with_avoidance(robot, x_target, y_target):
    tolerance = 50
    max_attempts = 40
    attempts = 0
    
    x_current, y_current = get_current_pos(robot)
    dx = x_target - x_current
    dy = y_target - y_current
    distance = math.hypot(dx, dy)
    
    for wall in walls: 
        if distance < math.hypot(wall[0]-x_target, wall[1]-y_target):
            #Avoid situation where wall is too close to the position to always be considered
            continue

        obstructed, new_target_x, new_target_y = calculate_next_target(wall[0], wall[1], x_current, y_current, x_target, y_target)
        if obstructed and math.hypot(new_target_x-x_current, new_target_y-y_target)>10 and math.hypot(new_target_x-x_current, new_target_y-y_target):
            print("Obstacle found at %f, %f", wall[0], wall[1])
            print("New partial target at %f, %f", new_target_x, new_target_y)
            navigate_with_avoidance(robot, new_target_x, new_target_y, x_current, y_current)
            x_current, y_current = new_target_x, new_target_y

    # Initialise tracking
    #reset_navigation_data(x_current, y_current, x_target, y_target)
    start_time = time.time()
    
    print(f"Navigating from ({x_current:.0f}, {y_current:.0f}) to ({x_target:.0f}, {y_target:.0f})")
    
    while attempts < max_attempts:
        #check the heading
        dx = x_target - x_current
        dy = y_target - y_current
        distance = math.hypot(dx, dy)
        robot_heading = math.atan2(dy, dx)

        print(f"Initial: Distance {distance:.0f}mm, Angle {math.degrees(robot_heading):.0f}°")
        rotation(robot, radians(robot_heading - get_current_heading(robot)))
        time.sleep(0.2)

        print(f"\n--- Attempt {attempts + 1} ---")
        print(f"Current position: ({x_current:.0f}, {y_current:.0f})")
        print(f"Current heading: {math.degrees(robot_heading):.0f}°") 
        print(f"Distance to target: {distance:.0f}mm")
        
        if distance < tolerance:
            print("Target reached!")
            return True
        
        move_distance = min(DISTANCE_PER_MOVE, distance)
        move_duration = move_distance / WHEEL_SPEED * CALIBRATED_CONSTANT
        
        print(f"Checking for wall ahead...")
        wall_hit = check_for_wall(robot, move_duration)

        x_current, y_current = get_current_pos(robot)
        add_position(x_current, y_current)

        if wall_hit:
            print(f">>> WALL HIT at attempt {attempts + 1} <<<")

            #Find new way 
            obstructed, new_target_x, new_target_y = calculate_next_target(walls[-1][0], walls[-1][1], x_current, y_current, x_target, y_target)
            if obstructed and math.hypot(new_target_x-x_current, new_target_y-y_target)>10:
                print("New partial target at %f, %f", new_target_x, new_target_y)
                navigate_with_avoidance(robot, new_target_x, new_target_y, x_current, y_current)
                x_current, y_current = get_current_pos(robot)
                add_position(x_current, y_current)
            
        else:
            print("No wall detected, moving forward")
            print(f"Moved forward to ({x_current:.0f}, {y_current:.0f})")
        
        attempts += 1
        time.sleep(0.2)
    
    print("\n!!! Could not reach target !!!")
    return False

def get_current_pos(robot: cozmo.robot.Robot):
    robotPose = Frame2D.fromPose(robot.pose)
    return robotPose.x(), robotPose.y()

def get_current_heading(robot: cozmo.robot.Robot):
    robotPose = Frame2D.fromPose(robot.pose)
    return robotPose.angle()

def rotation(robot: cozmo.robot.Robot, angle):  
    #debug
    attempts = 0

    move_duration_2pi = 3.0  
    target = get_current_heading(robot) + angle
    print("Target: " + str(target))

    difference = get_current_heading(robot) - target
    difference = normalise_angle(difference)

    while abs(difference) > ANGLE_THRESHOLD:
        #debug
        # attempts+=1
        # print("Difference from target: " + str(difference))
        move_duration = abs(move_duration_2pi * difference / (2 * math.pi))
        move_duration = max(MIN_ROTATION_SEC, move_duration)
        if difference < 0:
            robot.drive_wheels(-WHEEL_SPEED, WHEEL_SPEED, duration=move_duration)
        else:
            robot.drive_wheels(WHEEL_SPEED, -WHEEL_SPEED, duration=move_duration)
        time.sleep(move_duration)
        # robot.drive_wheels(0, 0)
        # time.sleep(0.1)

        difference = get_current_heading(robot) - target
        difference = normalise_angle(difference)

    # print("Attempts: " + str(attempts))
    # print("Difference: " + str(difference))

### -------------------------------------------–----------------- ###
### EXPLORE API FUNCTIONS ###

def create_cozmo_walls(robot: cozmo.robot.Robot):
    types = [CustomObjectTypes.CustomType01,
             CustomObjectTypes.CustomType02,
             CustomObjectTypes.CustomType03,
             CustomObjectTypes.CustomType04,
             CustomObjectTypes.CustomType05,
             CustomObjectTypes.CustomType06,
             CustomObjectTypes.CustomType07,
             CustomObjectTypes.CustomType08,
             CustomObjectTypes.CustomType09,
             CustomObjectTypes.CustomType10,
             CustomObjectTypes.CustomType11,
             CustomObjectTypes.CustomType12,
             CustomObjectTypes.CustomType13,
             CustomObjectTypes.CustomType14,
             CustomObjectTypes.CustomType15,
             CustomObjectTypes.CustomType16]
    markers = [CustomObjectMarkers.Circles2,
             CustomObjectMarkers.Diamonds2,
             CustomObjectMarkers.Hexagons2,
             CustomObjectMarkers.Triangles2,
             CustomObjectMarkers.Circles3,
             CustomObjectMarkers.Diamonds3,
             CustomObjectMarkers.Hexagons3,
             CustomObjectMarkers.Triangles3,
             CustomObjectMarkers.Circles4,
             CustomObjectMarkers.Diamonds4,
             CustomObjectMarkers.Hexagons4,
             CustomObjectMarkers.Triangles4,
             CustomObjectMarkers.Circles5,
             CustomObjectMarkers.Diamonds5,
             CustomObjectMarkers.Hexagons5,
             CustomObjectMarkers.Triangles5]
    cozmo_walls = []
    for i in range(0,8):
        cozmo_walls.append(robot.world.define_custom_wall(types[i], markers[i], 200, 60, 50, 50, True))
    
    for i in range(8,16):
        cozmo_walls.append(robot.world.define_custom_wall(types[i], markers[i], 300, 60, 50, 50, True))

def add_wall(wall_x, wall_y):
    walls.append((wall_x, wall_y))


def draw_map(robot: cozmo.robot.Robot):
    global ax
    ax.clear()

    robotPose = Frame2D.fromPose(robot.pose)
    robot_angle = robotPose.angle()
    
    #Centre view on robot
    ax.set_xlim(robotPose.x() - 1000, robotPose.x() + 1000)
    ax.set_ylim(robotPose.y() - 1000, robotPose.y() + 1000)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    
    elapsed = int(time.time() - start_time)
    ax.set_title(f'Time: {elapsed}s | Walls: {len(walls)} | Cubes Found: {len(cubes)}')
    
    #Draw the path
    if len(path) > 1:
        path_xs = [p[0] for p in path]
        path_ys = [p[1] for p in path]
        ax.plot(path_xs, path_ys, 'b-', alpha=0.5)
    for (x,y) in path:
        circle = plt.Circle((x, y), RADIUS_CIRCLES, color='g')
        ax.add_patch(circle)

    
    #Draw walls (as points)
    for i in range(len(walls)):
        wall_x, wall_y = walls[i]
        ax.plot(wall_x, wall_y, 'rs', markersize=10, markeredgecolor='darkred', markeredgewidth=2)

        wall_x1 = wall_x - WALL_RADIUS * math.cos(walls_angles[i])
        wall_y1 = wall_y - WALL_RADIUS * math.sin(walls_angles[i])
        wall_x2 = wall_x + WALL_RADIUS * math.cos(walls_angles[i])
        wall_y2 = wall_y + WALL_RADIUS * math.sin(walls_angles[i])
        ax.plot([wall_x1, wall_x2], [wall_y1, wall_y2], 'r-', linewidth=3)
    
    #Draw cubes (yellow squares)
    count_cubes = 0
    cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
    for cubeID in cubeIDs: 
        if cubes[cubeID][0] == True:
            count_cubes += 1
            ax.plot(cubes[cubeID][1].x(), cubes[cubeID][1].y(), 'ys', markersize=15, markeredgecolor='orange', markeredgewidth=2)
    
    ax.set_title(f'Time: {elapsed}s | Walls: {len(walls)} | Cubes Found: {count_cubes}')

    #Draw Cozmo
    ax.plot(robotPose.x(), robotPose.y(), 'bo', markersize=10)
    
    #Arrow showing which way the Cozmo is facing
    
    arrow_len = 80
    arrow_x = robotPose.x() + arrow_len * math.cos(robot_angle)
    arrow_y = robotPose.y() + arrow_len * math.sin(robot_angle)
    ax.arrow(robotPose.x(), robotPose.y(), arrow_x - robotPose.x(), arrow_y - robotPose.y(), 
             head_width=30, head_length=30, fc='blue', ec='blue')
    
    plt.draw()
    plt.pause(0.01)


def scan_for_cubes(robot: cozmo.robot.Robot):
    #Do a 360-degree stepwise scan looking for cubes
    
    steps = 18  #18 steps * 20 degrees = 360 degrees
    step_angle = 20
    
    print("Scanning 360 degrees for cubes...")
    
    for step in range(steps):
        #Check each cube ID to see if visible at this angle
        cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
        for cubeID in cubeIDs: 
            cube = robot.world.get_light_cube(cubeID)
            if cube is not None and cube.is_visible:
                cubePose2D = Frame2D.fromPose(cube.pose)
                #robot.go_to_object(cube, distance_mm(50.0)).wait_for_completed()
                cubes[cubeID][1] = cubePose2D
                cubes[cubeID][0] = True
                print(f"Found a cube at" + str(cubePose2D) + "mm during scan!")
                
                robot.drive_wheels(0, 0)
                time.sleep(0.2)
                draw_map(robot)
                #Continue scanning for other cubes
        
        #Turn to next scan position
        if step < steps - 1:  #Don't turn on the last step
            rotation(robot, radians(step_angle))
            time.sleep(0.1)
            draw_map(robot)
    
    return False  #Scan complete

def _distance_point_to_segment(px, py, x1, y1, x2, y2):
    """Return shortest distance from point (px,py) to segment (x1,y1)-(x2,y2).
    All units are assumed to be mm.
    """
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    seg_len_sq = vx*vx + vy*vy
    if seg_len_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = (wx*vx + wy*vy) / seg_len_sq
    if t <= 0:
        cx, cy = x1, y1
    elif t >= 1:
        cx, cy = x2, y2
    else:
        cx = x1 + t * vx
        cy = y1 + t * vy
    return math.hypot(px - cx, py - cy)


def is_path_blocked(start_f, end_f, obstacles_list, clearance_mm=50.0):
    """Check whether any obstacle in obstacles_list lies within clearance_mm
    (plus obstacle radius if available) of the straight-line segment from
    start_f to end_f. Returns (blocked: bool, obstacle, distance_mm).
    """
    sx, sy = start_f[0], start_f[1]
    ex, ey = end_f[0], end_f[1]

    min_dist = float('inf')
    for obs in obstacles_list:
        try:
            ox, oy = obs[0], obs[1]
        except Exception:
            continue
        dist = _distance_point_to_segment(ox, oy, sx, sy, ex, ey)
        min_dist = min(min_dist, dist)

    if min_dist <= (clearance_mm):
        return True, min_dist
    return False, None
    
def add_reachable_position_to_map(robot: cozmo.robot.Robot, current_position):
    
    possible_map_positions = [
        (current_position[0] + MOVE_DISTANCE, current_position[1]),
        (current_position[0] - MOVE_DISTANCE, current_position[1]),
        (current_position[0], current_position[1] + MOVE_DISTANCE),
        (current_position[0], current_position[1] - MOVE_DISTANCE)
    ]
    max_distance = 0  # Max distance Cozmo can travel in one go
    default_position = None
    for position in possible_map_positions:
        blocked, distance = is_path_blocked(current_position, position, walls, clearance_mm=WALL_RADIUS)
        
        if distance is not None and distance > max_distance:
            max_distance = distance
            default_position = position

        if blocked:
            print("Path to position %s is BLOCKED by an obstacle." % str(position))
            possible_map_positions.remove(position)

    if default_position is not None and len(possible_map_positions) == 0:
        possible_map_positions.insert(0, default_position)

    return possible_map_positions

def choose_next_position(possible_map_positions):
    for position in possible_map_positions:
        position_seen = False
        print("Possible position to explore: " + str(position))
        for map_key in map.keys():
            if position[0] > map_key[0]-RANGE_MAP_KEY and position[0] < map_key[0]+RANGE_MAP_KEY and position[1] > map_key[1]-RANGE_MAP_KEY and position[1] < map_key[1]+RANGE_MAP_KEY:
                position_seen = True
        if(not position_seen):
            return position
    return possible_map_positions[0]

def handle_object_observed(evt, **kw):
    global walls
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        if evt.obj not in marked_walls_seen:
            marked_walls_seen.append(evt.obj)
            print("Cozmo observed a wall at %s" % str(evt.obj.pose.position))
            print(evt.obj)
            add_wall(evt.obj.pose.position.x, evt.obj.pose.position.y)
            walls_angles.append(evt.obj.pose.rotation.angle_z.radians + math.pi/2)

def print_stats():
    elapsed = time.time() - start_time
    
    #Calculate total distance traveled
    total_dist = 0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i-1][0]
        dy = path[i][1] - path[i-1][1]
        total_dist += math.sqrt(dx*dx + dy*dy)

    cubes_found = 0
    for cubeID in cubes:
        if cubes[cubeID][0] == True:
            cubes_found += 1
    
    print("\nStats")
    print(f"Time: {elapsed:.1f}s")
    print(f"Distance: {total_dist:.0f}mm")
    print(f"Walls: {len(walls)}")
    print(f"Cubes found: {cubes_found}")

### -------------------------------------------–----------------- ###

def save_cube(robot: cozmo.robot.Robot, cubeID):
    global cubes
    navigate_with_avoidance(robot, cubes[cubeID][1].x(), cubes[cubeID][1].y())
    robot.set_lift_height(1.0).wait_for_completed()
    navigate_with_avoidance(robot, 0, 0)

    robot.set_lift_height(0.0).wait_for_completed()
    robot.drive_wheels(-WHEEL_SPEED, -WHEEL_SPEED, duration=1.0)

    cubes[cubeID][0] = False
    cubes[cubeID][1] = None

def rescue(robot: cozmo.robot.Robot):
    create_cozmo_walls(robot)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    time.sleep(1)
    robot.set_head_angle(Angle(0)).wait_for_completed
    time.sleep(1)
    
    global path

    robot.camera.image_stream_enabled = True
    robot.set_lift_height(0.0).wait_for_completed()
    
    print("Starting exploration with position-based navigation")
    
    try:
        while True:
            scan_for_cubes(robot)
            time.sleep(0.5)

            last_position_x, last_position_y = get_current_pos(robot)
            cubeIDs = (cozmo.objects.LightCube1Id,cozmo.objects.LightCube2Id,cozmo.objects.LightCube3Id)
            for cubeID in cubeIDs: 
                if cubes[cubeID][0] == True:
                    print("Rescue cube " + str(cubeID))
                    save_cube(robot, cubeID)

            possible_map_positions=add_reachable_position_to_map(robot, (last_position_x, last_position_y))

            path.append((last_position_x, last_position_y))
            #Update map with positions
            map[(last_position_x, last_position_y)] = possible_map_positions

            #go to next unseen position
            target = choose_next_position(possible_map_positions)
            
            #Try to move to that position
            reached = navigate_with_avoidance(robot, target[0], target[1])
                    
            
            
            
    except KeyboardInterrupt:
        print("\nStopping")
        robot.drive_wheels(0, 0)
        robot.set_lift_height(0.0).wait_for_completed()
        print_stats()
        plt.ioff()
        plt.show()


cozmo.run_program(rescue)