from buildhat import Motor
from basehat import IMUSensor, UltrasonicSensor
import time

motorL = Motor('D')
motorR = Motor('A')
# TODO These numbers are prolly wrong
sensor_front = UltrasonicSensor(9)
sensor_right = UltrasonicSensor(18)
imu = IMUSensor()

# Initial variables
SPEED = 20 # Base speed of the robot
SLOW_SPEED = 5

# TODO These need hella testing, especially target dist (Disable if too annoying)
DIST_MIN = 15 # Distance considered too close in cm
DIST_MAX = 25 # Max distance before wall isn't considered significant
TARGET_DIST = 20 # Target distance to stay from wall

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
            total += abs(gz - GYRO_BIAS) * dt
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

# TODO set this to 0 if there are issues
GYRO_BIAS = calibrate_gyro()

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
        if right_dist > DIST_MAX: # default right turn if possible
            stop()
            turn_degrees(turn_right)
        elif front_dist < DIST_MIN: # turn left otherwise
            stop()
            turn_degrees(turn_left)
        else:
            start() # TODO idk if this rerunning constantly is a good idea

        # TODO Minor turning for adjustments, probably won't work, more theoretical
        # Uncomment if there are issues, could not work, but might also solve some problems
        # Could also track the change in distance, if it's steadily increasing, minor turn right
        # elif right_dist < DIST_MIN:
        #     turn_degrees(turn_left, degrees=3.0, speed=SLOW_SPEED, tolerance=1.0)

        time.sleep(0.01)
except KeyboardInterrupt:
    stop()
    print("Code doth cease")
   
