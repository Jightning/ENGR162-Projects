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

DIST_MIN = 15 # Distance considered too close in cm
DIST_MAX = 25 # Max distance before wall isn't considered significant

TURN_SLOW_THRESHOLD = 8 # How many degrees before target to start slow turning
GYRO_BIAS = 0.0 # Gets set later, for gyro error correction

# PID variables
MOTOR_CMD_MAX = 80
Kp=1.6
Ki=0.01
Kd=0.12

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

# Theoretical pid implementation
def turn_degrees_pid(deg, clockwise=True, tolerance=2.0, timeout=5.0):
    target = float(deg) if clockwise else -float(deg)
    prev_error = target
    heading = 0.0
    integral = 0.0

    prev_time = time.time()
    start_time = prev_time

    while True:
        cur_time = time.time()
        dt = cur_time - prev_time
        prev_time = cur_time

        gx, gy, gz = imu.getGyro()
        heading += (gz - GYRO_BIAS) * dt

        # Error calculations
        error = target - heading
        integral += error * dt
        derivative = (error - prev_error) / dt if dt > 0 else 0.0
        prev_error = error

        adjustment = Kp * error + Ki * integral + Kd * derivative

        # Clamp to ensure we don't go overboard lol
        new_speed = int(min(MOTOR_CMD_MAX, abs(adjustment)))
        # Turn based on command
        if adjustment >= 0:
            turn_right(new_speed)
        else:
            turn_left(new_speed)

        # Exit conditions
        if abs(error) <= tolerance:
            break
        if time.time() - start_time > timeout:
            break

        time.sleep(0.01)

    stop()

# TODO set this to 0 if there are issues
GYRO_BIAS = calibrate_gyro()

try:
    # Task 2
    # Uncomment 2 lines below, modify degrees and turn func
    # turn_degrees(turn_right, degrees=90.0)
    # return

    start()
    while True:
        front_dist = sensor_front.getDist()
        right_dist = sensor_right.getDist()

        # Task 1
        if right_dist > DIST_MAX: # default right turn if possible
            turn_degrees_pid(90.0, clockwise=True)

        elif front_dist < DIST_MAX: # turn left otherwise
            turn_degrees_pid(90.0, clockwise=False)


        time.sleep(0.01)
except KeyboardInterrupt:
    stop()
    print("Code doth cease")
   
