

import RPi.GPIO as GPIO
import time
import threading
import board
import smbus2
import adafruit_vl53l1x

# === Motor Pins ===
PWMA = 12
AIN1 = 16
AIN2 = 26
STBY = 21

# Servo Pin
SERVO_PIN = 13

# TCA9548A Config
TCA_ADDRESS = 0x70
LEFT_CHANNEL = 2


# PID Configuration
TARGET_DISTANCE = 20  # cm
TOLERANCE = 2  # cm

# PID Gains (adjust as needed)
KP = 1.4
KI = 0.0
KD = 0.7

# Servo angle limits
STEERING_CENTER = 60
STEERING_LEFT = 35
STEERING_RIGHT = 90

# Quit flag
stop_flag = False

# PID state
previous_error = 0
integral = 0

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([PWMA, AIN1, AIN2, STBY, SERVO_PIN], GPIO.OUT)

motor_pwm = GPIO.PWM(PWMA, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
motor_pwm.start(0)
servo_pwm.start(0)

# === Motor Control ===

def standby(on):
    GPIO.output(STBY, GPIO.HIGH if on else GPIO.LOW)

def forward(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed_percent)

def stop():
    standby(False)
    motor_pwm.ChangeDutyCycle(0)

# === Servo Control ===

def set_servo_angle(angle):
    angle = max(STEERING_LEFT, min(STEERING_RIGHT, angle))
    duty = 2 + (angle / 18)
    for _ in range(10):
        servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)
    servo_pwm.ChangeDutyCycle(0)

# === TCA Channel Select ===

def select_tca_channel(channel):
    with smbus2.SMBus(1) as tca:
        tca.write_byte(TCA_ADDRESS, 1 << channel)
    time.sleep(0.1)

# === VL53L1X Init ===

def init_left_sensor():
    print("Initializing VL53L1X (Left)...")
    select_tca_channel(LEFT_CHANNEL)
    i2c = board.I2C()
    sensor = adafruit_vl53l1x.VL53L1X(i2c)
    sensor.distance_mode = 2
    sensor.timing_budget = 100
    sensor.start_ranging()
    return sensor



# === Keyboard Quit Thread ===

def input_listener():
    global stop_flag
    while not stop_flag:
        user_input = input()
        if user_input.strip().lower() == 'q':
            stop_flag = True
            print("\n🔴 'q' received. Stopping...")

# === PID Controller ===

def pid_control(error, dt):
    global previous_error, integral

    # PID Terms
    proportional = KP * error
    integral += error * dt
    derivative = (error - previous_error) / dt if dt > 0 else 0

    output = proportional + KI * integral + KD * derivative
    previous_error = error

    return output

# === Main ===

try:
    # Start input thread
    input_thread = threading.Thread(target=input_listener)
    input_thread.daemon = True
    input_thread.start()

    # Init sensor
    left_sensor = init_left_sensor()
    print("✅ Left sensor ready")
   
   

    # Start motion
    set_servo_angle(STEERING_CENTER)
    forward(60)

    last_time = time.time()

    while not stop_flag:
        select_tca_channel(LEFT_CHANNEL)
        if left_sensor.data_ready:
            distance = left_sensor.distance
            left_sensor.clear_interrupt()

            if distance is None:
                print("⚠️ Distance read failed. Skipping.")
                time.sleep(0.1)
                continue

            print(f"Left distance: {distance:.2f} cm")

           
            # PID calculation
            error = TARGET_DISTANCE - distance
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            correction = pid_control(error, dt)

            # Convert PID output to servo angle (center ± output)
            new_angle = STEERING_CENTER + correction
            set_servo_angle(new_angle)

        time.sleep(0.05)

except Exception as e:
    print(f"⚠️ Exception: {e}")

finally:
    print(" Cleaning up...")
    try:
        left_sensor.stop_ranging()
    except:
        pass
    stop()
    set_servo_angle(STEERING_CENTER)
    motor_pwm.stop()
    servo_pwm.stop()
    GPIO.cleanup()
    print("✅ All systems stopped.")
