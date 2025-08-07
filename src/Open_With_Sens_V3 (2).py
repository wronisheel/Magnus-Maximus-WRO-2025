# Gyro Straight is working
# stop no frist corner
# give us turning direction


import RPi.GPIO as GPIO
import time
import threading
import board
import smbus2
import adafruit_vl53l1x
import adafruit_bno055
import subprocess

# === Motor Pins ===
PWMA = 12
AIN1 = 16
AIN2 = 26
STBY = 21

# Servo Pin
SERVO_PIN = 13

# TCA9548A Config
TCA_ADDRESS = 0x70
BNO055_CHANNEL = 4
LEFT_CHANNEL = 2
CENTER_CHANNEL = 0
RIGHT_CHANNEL = 1
quadrant = 0
sensornomina = "neutral"
wfoll = "neutral"
distance = 0
previous_error = 0
integral = 0

# PID Configuration
TARGET_DISTANCE = 24  # Initial 0 cm
TOLERANCE = 5      # cm
KP = 1.2
KI = 0.0
KD = 0.0

# Servo angle limits
STEERING_CENTER = 64
RIGHTMAX = 93
LEFTMAX = 35
SERVO_DEVIATION_LIMIT = 30

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([PWMA, AIN1, AIN2, STBY, SERVO_PIN], GPIO.OUT)

motor_pwm = GPIO.PWM(PWMA, 1000)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
motor_pwm.start(0)
servo_pwm.start(0)

# === Basic Functions ===
def standby(on):
    GPIO.output(STBY, GPIO.HIGH if on else GPIO.LOW)

def forward(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed_percent)
    
def reverse(speed_percent):
    standby(True)
    GPIO.output(AIN1, GPIO.LOW)
    GPIO.output(AIN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(speed_percent)    

def stop():
    standby(False)
    motor_pwm.ChangeDutyCycle(0)

def set_servo_angle(angle):
    angle = max(STEERING_CENTER - SERVO_DEVIATION_LIMIT, min(STEERING_CENTER + SERVO_DEVIATION_LIMIT, angle))
    angle = max(0, min(180, angle))
    duty = 2 + (angle / 18)
    servo_pwm.ChangeDutyCycle(duty)
    time.sleep(0.05)
    #for _ in range(10):
     #   servo_pwm.ChangeDutyCycle(duty)
      #  time.sleep(0.05)
    servo_pwm.ChangeDutyCycle(0)
 
def set_servo_angley(angley):
    angley = max(LEFTMAX, min(RIGHTMAX, angley))
    duty = 2 + (angley / 18)
    for _ in range(10):
        servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)
    servo_pwm.ChangeDutyCycle(0)    

def select_tca_channel(channel):
    with smbus2.SMBus(1) as tca:
        tca.write_byte(TCA_ADDRESS, 1 << channel)
    time.sleep(0.1)

# === Sensor Initialization ===
def init_vl53l1x(channel):
    select_tca_channel(channel)
    i2c = board.I2C()
    sensor = adafruit_vl53l1x.VL53L1X(i2c)
    sensor.distance_mode = 2
    sensor.timing_budget = 100
    sensor.start_ranging()
    return sensor

def init_bno055():
    select_tca_channel(BNO055_CHANNEL)
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    time.sleep(1)
    return sensor

# === Sensor Reading ===
def get_distance(sensor, channel):
    select_tca_channel(channel)
    time.sleep(0.05)
    if sensor.data_ready:
        distance = sensor.distance
        sensor.clear_interrupt()
        if distance is None:
            distance = 400
        return distance
    return 400

def get_heading(imu_sensor):
    select_tca_channel(BNO055_CHANNEL)
    time.sleep(0.05)
    euler_data = imu_sensor.euler
    if euler_data is not None and euler_data[0] is not None:
        return round(euler_data[0])
    return None
    
##Custom MyBlocks    
def quadrant1_leftturner():
    print("quadrant1left")
    ##forward(60)
    ##time.sleep(0.5)
    set_servo_angle(33)
    forward(60)
    time.sleep(3)
    set_servo_angle(STEERING_CENTER)
    stop()
    
def quadrant1_rightturner():
    print("quadrant1right")
    ##forward(60)
    ##time.sleep(0.5)
    set_servo_angle(90)
    forward(60)
    time.sleep(3)
    set_servo_angle(STEERING_CENTER)
    stop()
    
def quadrant2_leftturner():
    print("quadrant2left")
    reverse(60)
    time.sleep(0.5)
    set_servo_angle(38)
    forward(60)
    time.sleep(3.5)
    set_servo_angle(STEERING_CENTER)
    stop()
    
def quadrant2_rightturner():
    print("quadrant2right")
    reverse(60)
    time.sleep(0.5)
    set_servo_angle(90)
    forward(60)
    time.sleep(3.5)
    set_servo_angle(STEERING_CENTER)
    stop()
    
def quadrant3_leftturner():
    print("quadrant3left")
    reverse(60)
    time.sleep(1)
    set_servo_angle(93)
    forward(60)
    time.sleep(3)
    set_servo_angle(STEERING_CENTER)
    forward(60)
    time.sleep(3)
    stop()
    
def quadrant3_rightturner():
    print("quadrant3right")
    reverse(60)
    time.sleep(1)
    set_servo_angle(35)
    forward(60)
    time.sleep(3)
    set_servo_angle(STEERING_CENTER)
    forward(60)
    time.sleep(3)
    stop()
    
def waiver():
    set_servo_angle(STEERING_CENTER)
    set_servo_angle(44)
    forward(40)
    time.sleep(2)
    set_servo_angle(84)
    forward(40)
    time.sleep(2)
    set_servo_angle(44)
    forward(40)
    time.sleep(2)
    set_servo_angle(84)
    forward(40)
    time.sleep(2)
    set_servo_angle(44)
    forward(40)
    time.sleep(2)
    set_servo_angle(84)
    forward(40)
    time.sleep(2)
    set_servo_angle(STEERING_CENTER)
    
def pid_control(error, dt):
    global previous_error, integral
    proportional = KP * error
    integral += error * dt
    derivative = (error - previous_error) / dt if dt > 0 else 0
    output = proportional + KI * integral + KD * derivative
    previous_error = error
    return output
    
    
                                           
    

# === Main Execution ===
if __name__ == '__main__':
    time.sleep(3)
    
    left_sensor = init_vl53l1x(LEFT_CHANNEL)
    print("✅ Left sensor ready")
    
    center_sensor = init_vl53l1x(CENTER_CHANNEL)
    print("✅ Center sensor ready")

    right_sensor = init_vl53l1x(RIGHT_CHANNEL)
    print("✅ Right sensor ready")

    imu_sensor = init_bno055()
    print("✅ BNO055 sensor ready")

    base_heading = get_heading(imu_sensor)
    print(f"🚗 Initial Heading: {base_heading}°")
    
    distaL = get_distance(left_sensor, LEFT_CHANNEL)
    distaR = get_distance(right_sensor, RIGHT_CHANNEL)
    distaL = round(distaL)
    distaR = round(distaR)
    print(f"Left: {distaL:.2f} cm, Right: {distaR:.2f} cm")
    
    if abs(distaL - distaR) < 10:
        quadrant == 2
        print("Q2")
    else:
        if distaL < distaR:
                sensornomina = "Left"
        
        elif distaR < distaL:
                sensornomina = "Right"
    
    print(f"SNOM: {sensornomina}")
    time.sleep(1)
    
    # Go straight until left or right distance > 120
    turndir = None
    startsection = True
    set_servo_angle(STEERING_CENTER)
    forward(60)  # Set desired speed



    while True:
        
        
        while startsection:
            distL = get_distance(left_sensor, LEFT_CHANNEL)
            distR = get_distance(right_sensor, RIGHT_CHANNEL)
            distC = get_distance(center_sensor, RIGHT_CHANNEL)
            heading = get_heading(imu_sensor)
        
            """if distL < distR:
                if distL - distR < 10 or distR - distL < 10:
                    quadrant = 2
                    print("Q2 Upper")
                else: 
                    sensornomina = "Left"
        
            elif distR < distL:
                if distL - distR < 10 or distR - distL < 10:
                    quadrant = 2
                    print("Q2 Lower")
                else: 
                    sensornomina = "Right""" 

            if heading is None:
                print("⚠️ Heading read failed.")
                continue

            # Proportional heading correction
            error = (heading - base_heading + 180) % 360 - 180
            #error = heading - base_heading
            correction = int(error * 1.5)  # Adjust gain as needed
            target_angle = STEERING_CENTER - correction
            set_servo_angle(target_angle)

            print(f"Heading: {heading}°, TargetAngle: {target_angle}°, Left: {distL:.2f} cm, Right: {distR:.2f} cm")
    
        

            if distL > 120 :
                turndir = "left"
                if sensornomina == "Right":
                    quadrant3_rightturner()
                    quadrant = 3
                elif sensornomina == "Left":
                    quadrant1_leftturner()
                    quadrant = 1
                elif quadrant == 2: 
                    quadrant2_leftturner()
                stop()    
                wfoll = "left"
                #distL = TARGET_DISTANCE
            
                print("Wall Follow Left")
                startsection = False
                break
            
            
            elif distR > 120:
                turndir = "right"
                if sensornomina == "Left":
                    quadrant3_leftturner()
                    quadrant = 3
                elif sensornomina == "Right":
                    quadrant1_rightturner()
                    quadrant = 1
                elif quadrant == 2:
                    quadrant2_rightturner()
                print("Wall Follow Right")
                stop()    
                wfoll = "right"
                #distR = TARGET_DISTANCE
                startsection = False      
                break
                
        ##return
        print("secloop")
        distL = get_distance(left_sensor, LEFT_CHANNEL)
        distR = get_distance(right_sensor, RIGHT_CHANNEL)
        print(f"Lefty: {distL:.2f} cm, Righty: {distR:.2f} cm")
        last_time = time.time()
        set_servo_angley(STEERING_CENTER)
        #waiver()
        forward(60)

        if wfoll == "left":
            #forward(60)
            
            distL = distance
            error = TARGET_DISTANCE - distance
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            correction = pid_control(error, dt)

            # Convert PID output to servo angle (center + correction)
            new_angle = STEERING_CENTER + correction
            set_servo_angley(new_angle)

            time.sleep(0.05)
            
            
        
        
        elif wfoll == "right":
            
            #forward(60)
            #set_servo_angley(STEERING_CENTER)
            distance = distR
            error = distance - TARGET_DISTANCE
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            correction = pid_control(error, dt)
            print(f"corr : {correction} ")

            # Convert PID output to servo angle (center + correction)
            new_angley = STEERING_CENTER + correction
            print(f"new_angley : {new_angley} ")
            set_servo_angley(new_angley)

        time.sleep(0.05)
            
                
            
        
   

        time.sleep(0.1)

