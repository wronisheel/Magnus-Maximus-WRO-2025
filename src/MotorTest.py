import RPi.GPIO as GPIO
import time

# --- Pin Assignments ---
AI2 = 26  # Direction pin 1
AI1 = 16 # Direction pin 2
PWMA = 12 # PWM pin for speed control
SB = 21

# --- Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(AI1, GPIO.OUT)
GPIO.setup(AI2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(SB , GPIO.OUT)
GPIO.output(SB, GPIO.HIGH)

# Initialize PWM at 100Hz
pwm = GPIO.PWM(PWMA, 100)
pwm.start(0)

try:
    # --- Move Forward ---
    GPIO.output(AI1, GPIO.HIGH)
    GPIO.output(AI2, GPIO.LOW)
    
    pwm.ChangeDutyCycle(75)  # Speed: 75%
    print("Motor running forward...")
    
    time.sleep(5)  # Run for 5 seconds

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    # --- Stop Motor ---
    pwm.ChangeDutyCycle(0)
    GPIO.output(AI1, GPIO.LOW)
    pwm.stop()
    GPIO.cleanup()
    print("Motor stopped and GPIO cleaned up.")
