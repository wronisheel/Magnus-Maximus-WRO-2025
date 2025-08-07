import RPi.GPIO as GPIO
import time

servo_pin = 13  # GPIO13 (BCM numbering)

GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM frequency to 50Hz
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    if angle < 0 or angle > 180:
        print("Angle must be between 0 and 180")
        return
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        angle_input = input("Enter angle (0 to 180): ")
        if angle_input.lower() == 'exit':
            break
        set_angle(float(angle_input))
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()
