from buildhat import Motor
from basehat import IMUSensor, UltrasonicSensor
import time

motorL = Motor('D')
motorR = Motor('A')
# BUG TODO These numbers are prolly wrong
sensor_front = UltrasonicSensor(9)
sensor_right = UltrasonicSensor(18)
imu = IMUSensor()

# initial variables
SPEED = 20 # Base speed of the robot
SLOW_SPEED = 5
DIST_MIN = 15 # Distance considered too close in cm
DIST_MAX = 25 # Max distance before wall isn't considered significant

TURN_SLOW_THRESHOLD = 8
CELL_TRAVEL_TIME = 1.5

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

def turn_degrees(turn_func, degrees=90.0, speed=SPEED, tolerance=2.0):
    total = 0.0
    prev_time = time.time()

    turn_func(speed)

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
        total += abs(gz) * dt
        time.sleep(0.005)

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

prev_time = time.time()
turn = 0

try:
    # Task 2
    # Uncomment 2 lines below, modify degrees and turn func
    # turn_degrees(turn_right, degrees=90.0)
    # return

    start()
    while True:
        front_dist = sensor_front.getDist
        right_dist = sensor_right.getDist

        # Task 1
        if front_dist < DIST_MIN:
            if right_dist < DIST_MAX:
                turn_degrees(turn_left)
            else:
                turn_degrees(turn_right)

        time.sleep(0.01)
except KeyboardInterrupt:
    stop()
    print("Code doth cease")
   
