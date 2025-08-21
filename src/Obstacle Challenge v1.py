# === Obstacle Challenge V1 ===
# Main execution script for Obstacle Challenge logic
# All reusable functions are in fe_functions.py for clarity
# V_3 = Detecting Red and Green Pillar and going to avoid it untill it goes off..
# and if no pillar detected then it stop there
# V_4 = implement fix path for avoiding pillars with gyro logic
# V_4 = Corner reverse turn with gyro added 11/08/2025
# V_5 = after reverse go striaght still pillar not find = Done
# V_5 = after reverse try to adjust robot in center of codner = Done
# V_5 = after cornering pillar is two close, robot will reverse 2 second = Done
# V_5 = currently V_5 is not working with two pillar in one section
#******
# V_6 = tyr to countung number of pillar and colour of each pillar in one section = Not Done
#******
# V_7 = dobule pillar is working, Pillar detection stoped in corner section so we can avoid false detection
# ~ Still need to update Left or Right Turning accorading target positions


import cv2
from picamera2 import Picamera2
from time import sleep
import RPi.GPIO as GPIO
import numpy as np
from fe_functionsV2 import *
import math
# ------------------------------{ function / class declarations }---------------------------------#
class Pillar:
    def __init__(self, area, dist, x, y, target):
        self.area = area #pillar area
        self.dist = dist #pillar distance from bottom middle point of screen
        self.x = x #pillar x
        self.y = y #pillar y
        self.target = target #stores either target of green pillars or target of red pillarss
        self.w = 0
        self.h = 0
   
    def setDimentions(self, w, h):
        self.w = w
        self.h = h

#takes in contours, selects a pillar and returns its information in the format of a Pillar object
def find_pillar(contours, target, p, colour):
   
    global turnDir, maxDist
    num_p = 0
   
    for cnt in contours:
        area = cv2.contourArea(cnt)
        #if areas are large enough for the specific colour pillar
        # you can set this to avoid small areas or small missleading detections
        if (area > 150 and colour == "red") or (area > 200 and colour == "green"):
           
            #get width, height, and x and y coordinates by bounding rect
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            #since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
            x += ROI3[0] + w // 2
            y += ROI3[1] + h
            #calculates the distance between the pillar and the bottom middle of the screen
            temp_dist = round(math.dist([x, y], [320, 480]), 0)
           
            #if the pillar is close enough add it to the number of pillars
            if 160 < temp_dist < 380: #180, 390
                num_p += 1
           
            #if this pillar is closer than the current pillar replace the current pillar and update its information
            if temp_dist < p.dist:
                p.area = area
                p.dist = temp_dist
                p.y = y
                p.x = x
                p.target = target
                p.setDimentions(w, h)

    return p, num_p

# === Safe shutdown handler ===
def stop_all(sig, frame):
    print("\n🛑 Emergency stop activated (Ctrl+C)")
    try:
        set_servo_angle(STEERING_CENTER)  # Center steering
        stop_car()                        # Stop DC motor
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("✅ Robot stopped safely.")
    except Exception as e:
        print(f"⚠️ Shutdown error: {e}")
    sys.exit(0)

# Attach signal handler for Ctrl+C
signal.signal(signal.SIGINT, stop_all)

if __name__ == '__main__':
    time.sleep(5)
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
   
    left_sensor = init_vl53l1x(LEFT_CHANNEL)
    time.sleep(0.1)
    center_sensor = init_vl53l1x(CENTER_CHANNEL)
    time.sleep(0.1)
    right_sensor = init_vl53l1x(RIGHT_CHANNEL)
    time.sleep(0.1)
    imu_sensor = init_bno055()
    #imu_sensor.mode = 0x08
    time.sleep(0.5)
    #print("✅ Left, Rigth, Center, Servo and BNO55 are ready")

    base_heading = get_heading(imu_sensor)
    print(f"✅ Initial Heading: {base_heading}°")
   
    lapsComplete = False
    #set the target x coordinates for each red and green pillar
    redTarget = 110
    greenTarget = 530
    #variable that keeps track of the target of the last pillar the car has passed
    lastTarget = 0
    turnDir = "none" #boolean storing the only direction the car is turning during the run
   
    stopTime = 0 # Stores a Stop time
    stopTimeNeed = 0 #determines how many number of seconds after sTime are needed to stop
   
    #Regions of Interest [x1,y1,x2,y2]
    ROI1 = [0, 175, 330, 265] #for finding left lane
    ROI2 = [330, 175, 640, 265] #for finding right lane
    ROI3 = [redTarget - 50, 120, greenTarget + 50, 345] #for finding signal pillars
    ROI4 = [200, 260, 440, 310] #for detecting blue and orange lines on mat
    ROI5 = [0, 0, 0, 0] #for turns at corners
    ROIs = [ROI1, ROI2, ROI3, ROI4, ROI5]
   
    #booleans for tracking whether car is in a left or right turn
    lTurn = False
    rTurn = False
   
    kp = 1.0 #proportional value for PD steering 0.015
    kd = 0.01 #derivative value for PD steering 0.01

   
    straightConst = 65 #angle in which car goes straight
    angle = 65 #variable for the current angle of the car
    prevAngle = angle #variable tracking the angle of the previous iteration
   
    speed = 60      #variable for initial speed of the car
    reverseSpeed = 60 #variable for speed of the car going backwards
   
    aDiff = 0 #value storing the difference of area between contours
    prevDiff = 0 #value storing the previous difference of contours for derivative steering

    error = 0 #value containing the difference between a pillar's x coordinate and its target x coordinate
    prevError = 0 #stores previous error
    maxDist = 370 #no pillar can be detected if further than this value
   
    tSignal = False #boolean detecting whether a coloured line is currently seen, makes sure that a pillar doesn't affect a turn too earl
    led_on()
    time.sleep(2) #delay 8 seconds for the ???? to be ready
   
    debug = False #boolean for debug mode
    #wait_for_start() # Wait for start button to be pressed
    time.sleep(2)  # Optional delay before starting
   
    #used to track time elapsed
    pTimer = time.time()
    start = False
    pillar_was_detected = False
   
    #used to determine whether there is a pillar directly in front of the car
    startArea = 0
    startTarget = 0
    target_heading = 0
    turnDir = "right"
    t = 1 #tracks number of turns

#--------------------------------{ main loop }---------------------------------------#
    while True:
        fps_start = time.time() #used in fps calculation
        #reset rightArea, and leftArea variables
        leftArea, rightArea, areaFront, areaFrontMagenta, tArea = 0, 0, 0, 0, 0
       
        img = picam2.capture_array() #get an image from pi camera
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab) # convert from BGR to HSV
        img_lab = cv2.GaussianBlur(img_lab, (7, 7), 0) #perform Gaussian Blur
               
        #find contours of pillars
        contours_red = find_contours(img_lab, rRed, ROI3)
        contours_green = find_contours(img_lab, rGreen, ROI3)
       
        #count number of red and green pillars
        num_pillars_g = 0
        num_pillars_r = 0
       
        #make a temporary pillar object
        temp = Pillar(0, 1000000, 0, 0, greenTarget)

        #find pillar to navigate around
        cPillar, num_pillars_g = find_pillar(contours_green, greenTarget, temp, "green")
        cPillar, num_pillars_r = find_pillar(contours_red, redTarget, cPillar, "red")

        # Draw contours
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_green, -1, (0, 255, 0), 2)
        cv2.drawContours(img[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], contours_red, -1, (0, 0, 255), 2)
       
        heading = get_heading(imu_sensor)
        if heading is None:
            time.sleep(0.05)
            continue
       
       
        if t==12 and not lapsComplete:
            lapsComplete = True
            led_off()
            stop_car()
            print("All Laps Completed...")
       
       
        #------------------------{ pillar detected }------------------------#
        if cPillar.area > 0 and cPillar.y > 350 and not lapsComplete:
            reverse(60)
            time.sleep(1)  
       
        elif cPillar.area > 0 and cPillar.y > 360 and not lapsComplete:
            pillar_was_detected = True
            if cPillar.target == greenTarget:
                print(f"Green Pillar,> area: {cPillar.area},> Y: {cPillar.y}")
                left_lrrl(imu_sensor, 45)
            elif cPillar.target == redTarget:
                print(f"Red Pillar,> area: {cPillar.area},> Y: {cPillar.y}")
                right_rllr(imu_sensor, 45)
                     
        elif cPillar.area == 0 and not lapsComplete:
            distanceC = get_distance(center_sensor, CENTER_CHANNEL)
            if distanceC is not None and distanceC < 100:
                #stop_car()
                print(f"Front Wall detected at {distanceC:.2f} cm — stopping.")
               
                target_heading = decide_target_heading(t, turnDir)
                drive_straight_until_obstacle(imu_sensor, target_heading, center_sensor, stop_distance=15)
                perform_reverse_turn_to_heading(imu_sensor, turnDir, target_heading)
                t += 1
                target_heading = decide_target_heading(t, turnDir)
                drive_straight_to_second(imu_sensor, target_heading, duration=1)
       
       
        heading = get_heading(imu_sensor)
        while heading is None:
            time.sleep(0.05)  # very short wait
            heading = get_heading(imu_sensor)
   
        target_heading = decide_target_heading(t, turnDir)
        error = (heading - target_heading + 180) % 360 - 180
        correction = kp * error
        set_servo_angle(int(STEERING_CENTER - correction))
        forward(speed)
       
        print(f">>>>>>>>>>>>Debug, TurnDir: {turnDir}, Turn: {t}, Target: {target_heading:.1f}°, Error: {error:.1f}°, ")

        # Show frame
        cv2.imshow("Red & Green Contours", img)    
           
        # Keep ~30 fps
        while int(1 / (time.time() - fps_start)) > 30:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()
    GPIO.cleanup()
