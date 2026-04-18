from buildhat import Motor
from basehat import IMUSensor, UltrasonicSensor
import time
import json

# Pathfinding is done via event logs, so when it turns it gets logged per cell
# Initial direction the robot is facing is assumed to be north

motorL = Motor('D')
motorR = Motor('A')
# TODO These numbers are prolly wrong
sensor_front = UltrasonicSensor(9)
sensor_right = UltrasonicSensor(18)
imu = IMUSensor()

JSON_LOG = True # TODO save path to a json, set to false if it errors
CALIBRATE = True # TODO Calibrate gyro, set to false if it causes issues

# initial variables
SPEED = 20 # Base speed of the robot
SLOW_SPEED = 5
CELL_TRAVEL_TIME = 1.5 # TODO How long it takes to travel 1 "cell" 

# TODO These need hella testing, especially target dist (Disable if too annoying)
DIST_MIN = 15 # Distance considered too close in cm
DIST_MAX = 25 # Max distance before wall isn't considered significant
TARGET_DIST = 20 # Target distance to stay from wall (try to make the distance between walls divided by two)

TURN_SLOW_THRESHOLD = 8 # How many degrees before target to start slow turning
GYRO_BIAS = 0.0 # Gets set later, for gyro error correction

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

# Initial gyro calibration, basically running gyro while stationary
# and tracking unwanted movement
def calibrate_gyro(samples=200):
    s = 0.0
    print(f"Calibrating gyro... Should take {0.01 * samples} seconds")
    # TODO Add tqdm if the pi allows it as an import PLEASEEEEE
    for _ in range(samples):
        gx, gy, gz = imu.getGyro()
        s += gz
        time.sleep(0.01)

    print("Done!\n")
    return s / samples

# TODO Replace with pid wall finder's function if possible, it's preferable imo
# Uncomment the function in the main loop if you decide to do so (2 lines to uncomment)
def turn_degrees(turn_func, degrees=90.0, speed=SPEED, tolerance=2.0):
    total = 0.0
    prev_time = time.time()

    turn_func(speed)
    try:
        # Main turn
        while total < (degrees - tolerance):
            # Having it slow down towards the end to get more precise
            if total > degrees - tolerance - TURN_SLOW_THRESHOLD:
                turn_func(SLOW_SPEED)

            # Change in time
            cur_time = time.time()
            dt = cur_time - prev_time
            prev_time = cur_time

            # Update rotation
            gx, gy, gz = imu.getGyro()
            # TODO absolute value may be needed, not really sure
            total += (gz - GYRO_BIAS) * dt
            time.sleep(0.005)
    except KeyboardInterrupt:
        stop()
        return

    stop()

    # Accounting for overshoot I LOVE RECURSION YESSIR
    # TODO I'd be shocked if this doesn't cause issues
    # Just comment out if too weird lol
    overshoot = abs(total - degrees)
    if overshoot > tolerance:
        # hacky but might work
        # basically passing a lambda that reverses the intended direction
        # This might slow down with a lotta recursion but with a high enough tolerance it
        # shouldn't happen, which i'm forcing by having it gradually increase
        turn_degrees(lambda x: turn_func(-x), degrees=overshoot, tolerance=tolerance*1.1)

def log(turned=False):
    path.append({
        "pos": (x, y),
        "dir": directions[direction],
        "turned": turned
    })
    print(f"({x}, {y}) going {directions[direction]}")

def update_coordinates():
    global x, y
    if direction == 0: y += 1 # North
    elif direction == 1: x += 1 # East
    elif direction == 2: y -= 1 # South
    elif direction == 3: x -= 1 # West

def move_one_cell():
    start_time = time.time()
    prev_time = time.time()

    # TODO more PID since I gotta study for the final
    # apparently for testing these values, do it like so:
    # 1. Set both to 0
    # 2. Steadily increase KP until the robot starts constantly moving in an S shape (this is the limit)
    # 3. Slowly increase KD until it stops doing this S shape
    KP = 2.0
    KD = 1.5
    MAX_ADJUSTMENT = SPEED - 5

    last_error = 0
    
    while time.time() - start_time < CELL_TRAVEL_TIME:
        front_dist = sensor_front.getDist
        right_dist = sensor_right.getDist

        # Wall
        if front_dist < DIST_MIN:
            break

        # No wall on the right, just drive straight
        if right_dist > DIST_MAX:
            start(SPEED)
            last_error = 0
        else:
            # Using proportional and derivative control
            # No integral cause nah
            cur_time = time.time()
            dt = cur_time - prev_time
            prev_time = cur_time 

            error = TARGET_DIST - right_dist
            derivative = (error - last_error) / max(dt, 1e-4) # using last error rather than dt formula since error could reset to 0
            
            adjustment = (KP * error) + (KD * derivative)
            adjustment = max(min(adjustment, MAX_ADJUSTMENT), -MAX_ADJUSTMENT)
            # positive adj if turning left, negative if turning right
            # TODO turning here is slightly different since its geared towards moving forward
            startL(SPEED + adjustment)
            startR(SPEED - adjustment)

            last_error = error
              
        time.sleep(min(CELL_TRAVEL_TIME, 0.02))

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

try:
    # Task 2
    # Uncomment 2 lines below, modify degrees and turn func
    # turn_degrees(turn_right, degrees=90.0)
    # return

    while True:
        front_dist = sensor_front.getDist
        right_dist = sensor_right.getDist

        # Task 5
        if right_dist > DIST_MAX: # Turn right if clear
            stop()
            turn_degrees(turn_right)
            # turn_degrees_pid(90.0, clockwise=True)
            direction = (direction + 1) % 4
            log(turned=True)
            
            # Travel one cell since its guaranteed
            move_one_cell() # TODO Comment this and uncomment bottom two lines if there are issues
            # start()
            # time.sleep(CELL_TRAVEL_TIME)

            update_coordinates()
            log()

        elif front_dist > DIST_MIN: # Travel one cell if clear
            move_one_cell() # TODO Comment this and uncomment bottom two lines if there are issues
            # start()
            # time.sleep(CELL_TRAVEL_TIME)

            update_coordinates()
            log()

        else: # Corner, turn left (new path not guaranteed)
            stop()
            turn_degrees(turn_left)
            # turn_degrees_pid(90.0, clockwise=True)
            direction = (direction - 1) % 4 
            log(turned=True)

        time.sleep(0.01)
except KeyboardInterrupt:
    stop()

    # dump path to a json to save it
    if JSON_LOG:
        with open('maze.json', 'w') as file:
            json.dump(path, file, indent=4)

    print("Code doth cease")
   
