from buildhat import Motor
from basehat import IMUSensor, UltrasonicSensor, IRSensor
import time
import json
import math

try:
    from tqdm import tqdm
except ImportError:
    print("tqdm not found. Run 'pip install tqdm' for the cool loading bar!")
    def tqdm(iterable, **kwargs): return iterable
    
# Pathfinding is done via event logs, so when it turns it gets logged per cell
# Initial direction the robot is facing is assumed to be north

motorL = Motor('D')
motorR = Motor('C')
cargoMotor = Motor('A')

sensor_front = UltrasonicSensor(26)
sensor_right = UltrasonicSensor(18)
sensor_left = UltrasonicSensor(24)
IR = IRSensor(4, 5)
IMU = IMUSensor()

# Set to true during actual run
JSON_LOG = True #  save path to a json, set to false if it errors
CALIBRATE = True # Calibrate gyro, set to false if it causes issues

# Testing variables (set to false during actual run)
TEST = False # Just log sensor distance without moving
TIME_BASED = False # Run for only a set amount of time

# initial variables
SPEED = 30 # Base speed of the robot
SLOW_SPEED = 5
CELL_DIST = 10 # Cell distance

DIST_MIN = 7 # Distance considered too close in cm
DIST_MAX = 30 # Max distance before wall isn't considered significant

TURN_SLOW_THRESHOLD = 8 # How many degrees before target to start slow turning
GYRO_BIAS = 0.0 # Gets set later, for gyro error correction

MAGNET_SOURCE_MAGNITUDE = 100 # Min magnitude of a magnetic source in uT
HEAT_SOURCE_MAGNITUDE = 10 # Min magnitude of a heat source in W

# PID variables
MOTOR_CMD_MAX = 22.5
MOTOR_CMD_MIN = 15
# Turning
Kp_turn = 5
Ki_turn = 0.01
Kd_turn = 0.06
# Moving
Kp_move = 1.25
Ki_move = 0
Kd_move = 0.05

run_finished = False # if the run is finished
critical_z = 1.5 # Max acceptable z-score before a value is an outlier. 12.73% alpha, so max of 3 outliers

# JSON style pathing, lets upload to JSON file once code terminates
path = []

x = 0
y = 0
direction = 0 # initial facing directions (north)
directions = ['N', 'E', 'S', 'W']

cur_time = time.time()

def stop():
    motorL.stop()
    motorR.stop()
    cargoMotor.stop()

def startL(speed):
    motorL.start(-speed)

def startR(speed):
    motorR.start(speed)

def start(speed=SPEED):
    startR(speed)
    startL(speed)
   
def turn_right(speed=SPEED):
    startL(speed)
    startR(-speed)

def turn_left(speed=SPEED):
    startL(-speed)
    startR(speed)
    
def unload():
    print("Depositing")
    cargoMotor.start(50)
    time.sleep(2)
    cargoMotor.stop()
    move_one_cell()
    move_one_cell()
    time.sleep(2)
    stop()

def get_safe_dist(sensor, default=DIST_MAX + 1):
    try:
        dist = sensor.getDist
        # If the sensor returns None, use the default value
        if dist is None:
            return default
        return float(dist)
    except Exception:
        # If the sensor temporarily disconnects or throws an error, don't crash
        return default

# Initial gyro calibration, basically running gyro while stationary
# and tracking unwanted movement
def calibrate_gyro(samples=200):
    s = 0.0
    print(f"Calibrating gyro... Should take {0.01 * samples:.2f} seconds")
    
    for _ in tqdm(range(samples), desc="Calibrating"):
        gx, gy, gz = IMU.getGyro()
        s += gz
        time.sleep(0.01)
        
    print(f"Done! Gyro Bias: {s / samples}\n")
    return s / samples

'''
def get_target_dist(samples=200):
    s = 0.0
    print(f"Getting Target Distance (Make sure robot is centered)... Should take {0.01 * samples:.2f} seconds")
    
    for _ in tqdm(range(samples), desc="Calibrating"):
        right_dist = sensor_right.getDist
        while right_dist is None:
            sensor_right.getDist

        s += right_dist
        time.sleep(0.01)
        
    print(f"Done! Target Dist: {s / samples}\n")
    return s / samples
'''

def get_target_dist(base_queue, samples=200):
    s = 0.0
    print(f"Getting Target Distance (Make sure robot is centered)... Should take ~{0.015 * samples:.2f} seconds")
    dists = base_queue.copy()

    for _ in tqdm(range(samples), desc="Calibrating"):
        right_dist = get_safe_dist(sensor_right)
        z = get_z_score(dists, right_dist)

        while z > critical_z:
            dists.pop(0)
            dists.append(right_dist)
 
            right_dist = get_safe_dist(sensor_right)
            z = get_z_score(dists, right_dist)

        dists.pop(0)
        dists.append(right_dist)

        s += right_dist
        time.sleep(0.01)
        
    print(f"Done! Target Dist: {s / samples}\n")
    return s / samples

def form_queue(sensor, samples=10):
    dist_queue = []
    print(f"Creating Distance Queue... Should take {0.01 * samples:.2f} seconds")
    for _ in tqdm(range(samples)):
        dist = get_safe_dist(sensor)
        dist_queue.append(dist)
        time.sleep(0.01)

    return dist_queue

def get_z_score(vals, val):
    std = 0
    avg = sum(vals) /  len(vals)
    for v in vals:
        std += (v - avg) ** 2
    std = max(math.sqrt(std / len(vals)), 1e-1)

    return (val - avg) / std

def log(turned=False, heat_magnitude=0.0, magnetic_magnitude=0.0, exit_point=False):
    path.append({
        "pos": (x, y),
        "dir": directions[direction],
        "turned": turned,
        "heat_source": abs(heat_magnitude) >= HEAT_SOURCE_MAGNITUDE,
        "magnetic_source": abs(magnetic_magnitude) >= MAGNET_SOURCE_MAGNITUDE,
        "exit_point": exit_point
    })
    # print(f"({x}, {y}) going {directions[direction]}")
 
def update_coordinates():
    global x, y
    if direction == 0: y += 1 # North
    elif direction == 1: x += 1 # East
    elif direction == 2: y -= 1 # South
    elif direction == 3: x -= 1 # West

def turn_degrees_pid(deg=90.0, clockwise=True, tolerance=3, timeout=5.0):
    target = -float(deg) if clockwise else float(deg)
    prev_error = target
    total = 0.0 
    integral = 0.0
    error = total - target

    prev_time = time.time()
    start_time = prev_time
    
    try:
        while True:
            cur_time = time.time()
            dt = cur_time - prev_time
            prev_time = cur_time

            gx, gy, gz = IMU.getGyro()
            total += (gz - GYRO_BIAS) * dt

            error = total - target

            integral += error * dt
            integral = max(min(integral, 50.0), -50.0) # anti-windup cap
            
            derivative = (error - prev_error) / max(dt, 1e-4)
            prev_error = error

            adjustment = Kp_turn * error + Ki_turn * integral + Kd_turn * derivative

            new_speed = int(min(MOTOR_CMD_MAX, abs(adjustment)))
            
            # Ensure motor receives enough power to actually move
            if new_speed > 0:
                new_speed = max(MOTOR_CMD_MIN, new_speed)

            if adjustment >= 0:
                turn_right(new_speed)
            else:
                turn_left(new_speed)

            # Error within tolerance (turn is complete) or taking too long
            if abs(error) <= tolerance or time.time() - start_time > timeout:
                break

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Done while turning")
        global run_finished
        run_finished = True
        stop()
        return
    
    stop()

def move_one_cell(timeout=5.0):
    global dist_queue

    prev_error = get_safe_dist(sensor_right) - TARGET_DIST
    prev_time = time.time()
    start_time = prev_time
    integral = 0.0
    vy, y = 0, 0

    try:
        while time.time() - start_time <= timeout:
            front_dist = get_safe_dist(sensor_front)
            right_dist = get_safe_dist(sensor_right)

            z_score = get_z_score(dist_queue, right_dist)
            dist_queue.pop(0)
            dist_queue.append(right_dist)
            if z_score > critical_z:
                continue

            cur_time = time.time()
            dt = cur_time - prev_time
            prev_time = cur_time 

            _, ay, _ = IMU.getAccel()
            vy = vy + ay * dt
            y = y + vy * dt
            
            # Reached distance or wall, or can turn right
            if abs(y) >= CELL_DIST or front_dist < DIST_MIN or right_dist > DIST_MAX:
                break

            # PID to align between the two walls
            error = right_dist - TARGET_DIST
            
            integral += error * dt
            integral = max(min(integral, 50.0), -50.0)

            derivative = (error - prev_error) / max(dt, 1e-4) # using last error rather than dt formula since error could reset to 0
            derivative = max(min(derivative, 10.0), -10.0)

            prev_error = error
            adjustment = (Kp_move * error) + (Kd_move * derivative) + (Ki_move * integral)
            adjustment = max(min(adjustment, MOTOR_CMD_MAX), -MOTOR_CMD_MAX)
            
            startL(SPEED + adjustment)
            startR(SPEED - adjustment)
    
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Done while onward")
        global run_finished
        run_finished = True
        stop()
        return
    
    stop()

# Main Code starts here --------------------------------------------------------------------------
stop()
print("Loading Cargo...")
cargoMotor.start(-50)
time.sleep(2)
cargoMotor.stop()

# queue of the last few right distances for checking outliers
dist_queue = form_queue(sensor_right, samples=20)

if CALIBRATE:
    GYRO_BIAS = calibrate_gyro()
    TARGET_DIST = get_target_dist(dist_queue)
else:
    GYRO_BIAS = 0.3
    TARGET_DIST = 11 # Target distance to stay from wall (dist from right sensor to right wall) (try to make slightly smaller in order to keep more right)
    

while time.time() - cur_time < 10 or not TIME_BASED:
    try:
        front_dist = get_safe_dist(sensor_front)
        right_dist = get_safe_dist(sensor_right)
        left_dist = get_safe_dist(sensor_left)
        
        z_score = get_z_score(dist_queue, right_dist)
        dist_queue.pop(0)
        dist_queue.append(right_dist)

        # If the current value is an outlier, then ignore
        if z_score > critical_z:
            continue
        
        # Magnetic/Heat source calculations
        mag_x, mag_y, mag_z = IMU.getMag() # Magnet
        mag_x, mag_y = -mag_x, -mag_y # IMU is flipped on the robot, so just flipping the values for clarity (+x is right, +y is forward)
        magnet_magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
        ir_left, ir_right = IR.value1, IR.value2 # Heat
        ir_avg = max(ir_left, ir_right)
        
        if abs(ir_avg) >= HEAT_SOURCE_MAGNITUDE or abs(magnet_magnitude) >= MAGNET_SOURCE_MAGNITUDE: # Found a source
            stop()
            print("Turning Around", right_dist, front_dist)
            turn_degrees_pid(clockwise=False, deg=180.0)
            dist_queue = form_queue(sensor_right, samples=20)
            direction = (direction - 2) % 4

            print("Then Onwards")
            move_one_cell()
            update_coordinates()

            log(turned=True, heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)
        elif right_dist > DIST_MAX and front_dist > DIST_MAX and left_dist > DIST_MAX: # At an open space
            print("Open Space Found")
            log(exit_point=True)
            break
        elif right_dist > DIST_MAX: # Turn right if clear
            start()
            time.sleep(0.5) # Go a little more forward to center
            stop()

            print("Turning Right")
            turn_degrees_pid(clockwise=True)
            dist_queue = form_queue(sensor_right, samples=20)
            direction = (direction + 1) % 4

            print("Then Onwards")
            move_one_cell()
            
            update_coordinates()
            log(turned=True, heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)
        elif front_dist < DIST_MIN: # Turn left if unable to go forward or turn right
            stop()
            print("Turning Left", right_dist, front_dist)
            turn_degrees_pid(clockwise=False)
            dist_queue = form_queue(sensor_right, samples=20)
            direction = (direction + 1) % 4

            log(turned=True, heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)
        else: # Travel one cell forward if clear
            print("Onward")
            move_one_cell()
            update_coordinates()
            log(heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)

        if run_finished:
            break

        time.sleep(0.01)
    except KeyboardInterrupt:
        stop()
        print("Code doth cease")
        break

print("Dumping...")
# dump path to a json to save it
if JSON_LOG:
    with open('maze.json', 'w') as file:
        json.dump(path, file, indent=4)
unload()

