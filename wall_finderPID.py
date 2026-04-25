from buildhat import Motor
from basehat import IMUSensor, UltrasonicSensor
import time
import math

try:
    from tqdm import tqdm
except ImportError:
    print("tqdm not found. Run 'pip install tqdm' for the cool loading bar!")
    def tqdm(iterable, **kwargs): return iterable

motorL = Motor('D')
motorR = Motor('A')
sensor_front = UltrasonicSensor(26)
sensor_right = UltrasonicSensor(18)
imu = IMUSensor()

# Initial variables
SPEED = 20
DIST = 15
DEFAULT_DIST = 30.0 # Fallback distance for sensor errors

GYRO_BIAS = 0.0 

# PID variables
MOTOR_CMD_MAX = 22.5
MOTOR_CMD_MIN = 15 # Minimum speed to overcome motor friction (deadband)
Kp = 5
Ki = 0.01
Kd = 0.06

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

# 
def get_safe_dist(sensor, default=DEFAULT_DIST):
    try:
        dist = sensor.getDist
        # If the sensor returns None, use the default value
        if dist is None:
            return default
        return float(dist)
    except Exception:
        # If the sensor temporarily disconnects or throws an error, don't crash
        return default

def calibrate_gyro(samples=200):
    s = 0.0
    print(f"Calibrating gyro... Should take {0.01 * samples:.2f} seconds")
    
    for _ in tqdm(range(samples), desc="Calibrating", ascii=True):
        gx, gy, gz = imu.getGyro()
        s += gz
        time.sleep(0.01)
        
    print("Done!\n")
    return s / samples

# Changed tolerance to 1.0 degree
def turn_degrees_pid(deg, clockwise=True, tolerance=3, timeout=5.0):
    target = float(deg) if clockwise else -float(deg)
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

            gx, gy, gz = imu.getGyro()
            total += (gz - GYRO_BIAS) * dt

            error = total - target
            integral += error * dt

            # Anti-windup cap
            integral = max(min(integral, 50.0), -50.0) 
            
            derivative = (error - prev_error) / max(dt, 1e-4)
            prev_error = error

            adjustment = Kp * error + Ki * integral + Kd * derivative

            new_speed = int(min(MOTOR_CMD_MAX, abs(adjustment)))
            
            # Ensure motor receives enough power to actually move
            if new_speed > 0:
                new_speed = max(MOTOR_CMD_MIN, new_speed)

            if adjustment >= 0:
                turn_right(new_speed)
            else:
                turn_left(new_speed)

            # Exit condition for tight tolerance
            if abs(error) <= tolerance:
                break
            if time.time() - start_time > timeout:
                print("PID Timeout reached!")
                break
            print("Error: " + str(error) + "          Total: " + str(total))
        
            time.sleep(0.001)
    except KeyboardInterrupt:
        stop()
        return

    stop()

GYRO_BIAS = calibrate_gyro()

try:
    is_moving = False 

    while True:
        # Use the safe wrapper instead of calling .getDist() directly
        front_dist = get_safe_dist(sensor_front)
        right_dist = get_safe_dist(sensor_right)
        print("Front Distance: " + str(front_dist) + 8*' ' + "Right Distance: " + str(right_dist))

        if right_dist > DEFAULT_DIST and front_dist < DIST: 
            turn_degrees_pid(-90.0, clockwise=True)
            
            front_dist = get_safe_dist(sensor_front)
            # Force the robot to drive forward briefly so it clears the corner
            if front_dist > 15:
                start()
                time.sleep(0.5) # Adjust this duration depending on robot speed
                is_moving = True
            
            
        elif right_dist < DIST and front_dist < DIST:
            turn_degrees_pid(-90.0, clockwise = False)

            front_dist = get_safe_dist(sensor_front)
            if front_dist > 15:
                start()
                time.sleep(0.5) # Adjust this duration depending on robot speed
                is_moving = True
                    
        else:
            # Only send the start command once when transitioning states
            start()

        time.sleep(0.01)

except KeyboardInterrupt:
    stop()
    print("\nCode doth gracefully cease.")