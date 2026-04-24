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
sensor_left = UltrasonicSensor(16)
IR = IRSensor(4, 5)
IMU = IMUSensor()

JSON_LOG = True #  save path to a json, set to false if it errors
CALIBRATE = True # Calibrate gyro, set to false if it causes issues

# Testing variables (set to false during actual run)
TEST = False # Just log sensor distance without moving
TIME_BASED = True # Run for only a set amount of time

# initial variables
SPEED = 20 # Base speed of the robot
SLOW_SPEED = 5
CELL_DIST = 5 # Cell distance

DIST_MIN = 7 # Distance considered too close in cm
DIST_MAX = 30 # Max distance before wall isn't considered significant
TARGET_DIST = 12.2 # Target distance to stay from wall (dist from right sensor to right wall) (try to make slightly smaller in order to keep more right)

TURN_SLOW_THRESHOLD = 8 # How many degrees before target to start slow turning
GYRO_BIAS = 0.0 # Gets set later, for gyro error correction

MAGNET_SOURCE_MAGNITUDE = 100 # Min magnitude of a magnetic source in uT
HEAT_SOURCE_MAGNITUDE = 5 # Min magnitude of a heat source in W

# PID variables
MOTOR_CMD_MAX = 22.5
MOTOR_CMD_MIN = 15
# Turning
Kp_turn = 5
Ki_turn = 0.01
Kd_turn = 0.06
# Moving
Kp_move = 2.5
Ki_move = 0.01
Kd_move = 0.15


def stop():
    motorL.stop()
    motorR.stop()

def startL(speed):
    motorL.start(-speed)

def startR(speed):
    motorR.start(speed)

def start(speed=SPEED):
    startL(speed)
    startR(speed)
   
def turn_right(speed=SPEED):
    startL(speed)
    startR(-speed)

def turn_left(speed=SPEED):
    startL(-speed)
    startR(speed)

def get_safe_dist(sensor, default=DIST_MAX):
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
    
    for _ in tqdm(range(samples), desc="Calibrating", ascii=True):
        gx, gy, gz = IMU.getGyro()
        s += gz
        time.sleep(0.01)
        
    print("Done!\n")
    return s / samples

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
        stop()
        return
    
    stop()

def log(turned=False, heat_magnitude=0.0, magnetic_magnitude=0.0):
    path.append({
        "pos": (x, y),
        "dir": directions[direction],
        "turned": turned,
        "heat_source": abs(heat_magnitude) >= HEAT_SOURCE_MAGNITUDE,
        "magnetic_source": abs(magnet_magnitude) >= MAGNET_SOURCE_MAGNITUDE
    })
    # print(f"({x}, {y}) going {directions[direction]}")
 
def update_coordinates():
    global x, y
    if direction == 0: y += 1 # North
    elif direction == 1: x += 1 # East
    elif direction == 2: y -= 1 # South
    elif direction == 3: x -= 1 # West

def move_one_cell():
    prev_time = time.time()
    vx, vy, vz, x, y, z = 0, 0, 0, 0, 0, 0
    prev_error = 0
    integral = 0.0

    try:
        while True:
            front_dist = get_safe_dist(sensor_front)
            right_dist = get_safe_dist(sensor_right)
            left_dist = get_safe_dist(sensor_left)

            cur_time = time.time()
            dt = cur_time - prev_time
            prev_time = cur_time 

            ax, ay, az = IMU.getAccel()
            vx, vy, vz = vx + ax * dt, vy + ay * dt, vz + az * dt
            x, y, z = x + vx * dt, y + vy * dt, z + vz * dt

            # Reached distance or wall
            if  abs(y) >= CELL_DIST or front_dist < DIST_MIN:
                break

            # No wall on the right/left, just drive straight
            if True and (right_dist > DIST_MAX or left_dist > DIST_MAX):
                start(SPEED)
                prev_error = 0
            else:
                # PID to align between the two walls
                error = right_dist - TARGET_DIST
                #print(error, right_dist)

                integral += error * dt
                integral = max(min(integral, 50.0), -50) # anti-windup cap

                derivative = (error - prev_error) / max(dt, 1e-4) # using last error rather than dt formula since error could reset to 0
                prev_error = error

                adjustment = (Kp_move * error) + (Kd_move * derivative) + (Ki_move * integral)
                adjustment = max(min(adjustment, MOTOR_CMD_MAX + 10), -MOTOR_CMD_MAX - 10)
                #print("to ", adjustment)
                
                # positive adj if turning left, negative if turning right
                # TODO turning here is slightly different since its geared towards moving forward
                startL(SPEED + adjustment)
                startR(SPEED - adjustment)


            time.sleep(0.05)
    except KeyboardInterrupt:
        stop()
        return
    
    stop()


if CALIBRATE:
    GYRO_BIAS = calibrate_gyro()
else:
    GYRO_BIAS = 0

# JSON style pathing, lets upload to JSON file once code terminates
path = []

x = 0
y = 0
direction = 0 # initial facing directions (north)
directions = ['N', 'E', 'S', 'W']

cur_time = time.time()

if TEST:
    while True:
        front_dist = get_safe_dist(sensor_front)
        right_dist = get_safe_dist(sensor_right)
        left_dist = get_safe_dist(sensor_left)

        mag_x, mag_y, mag_z = IMU.getMag()
        mag_x, mag_y = -mag_x, -mag_y 

        print(f"Front: {front_dist:.2f} Right: {right_dist:.2f} Left: {left_dist:.2f} Mag X: {mag_x:.2f} Mag Y: {mag_y:.2f} Mag Z: {mag_z:.2f}")

while time.time() - cur_time < 10 or not TIME_BASED:
    try:
        front_dist = get_safe_dist(sensor_front)
        right_dist = get_safe_dist(sensor_right)
        left_dist = get_safe_dist(sensor_left)
        
        # Magnetic/Heat source calculations
        mag_x, mag_y, mag_z = IMU.getMag() # Magnet
        mag_x, mag_y = -mag_x, -mag_y # IMU is flipped on the robot, so just flipping the values for clarity (+x is right, +y is forward)
        magnet_magnitude = math.sqrt(mag_x**2 + mag_y**2 + mag_z**2)
        ir_left, ir_right = IR.value1, IR.value2 # Heat
        ir_avg = (ir_left + ir_right) / 2.0

        if right_dist > DIST_MAX: # Turn right if clear
            stop()
            print("Turning Right")
            turn_degrees_pid(clockwise=True)
          
            direction = (direction + 1) % 4
            log(turned=True, heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)
        elif front_dist < DIST_MIN: # Turn left if unable to go forward or turn right
            stop()
            print("Turning Left")
            turn_degrees_pid(clockwise=False)

            direction = (direction - 1) % 4
            log(turned=True, heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)
        else: # Travel one cell forward if clear
            print("Onward")
            move_one_cell() # TODO Comment this and uncomment bottom two lines if there are issues
            update_coordinates()
            log(heat_magnitude=ir_avg, magnetic_magnitude=magnet_magnitude)

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
