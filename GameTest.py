#!/usr/bin/python3

display_debug = True
text_debug = False
############
# Camera
############

import cv2
import numpy as np
from picamera2 import Picamera2
from gripu import WhiteBall

if display_debug: cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

process = WhiteBall()

font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.25
color = (0, 255, 0)
thickness = 1

# Analysis region of interest in the image
width =  320
height = 240
startX =   0
startY = 100
endX =   320
endY =   200

#############
# Buttons and Switch
#############
# Importing libraries

import time             # import timing
import RPi.GPIO as GPIO # import Raspberry Pi input out put

# Global Variables
# Buttons
start_pin  = 21
# Switch
switch_pin = 20

# Initialize Button and Switch
GPIO.setmode(GPIO.BCM)
# Button is input
GPIO.setup(start_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#############
# meArm
#############

from pynput.keyboard import Listener
import MeArm
import time

arm = MeArm.meArm() # takes inserted data from meArm.py aka calibration data
arm.begin(0,0x70) #

# Defense
#########
x_offset = 0     # depends on your position in the field
x_gain   = -1    # depends on the distance and camera angle
z_offset = 0     # depends on the distance and camera angle
z_gain   = 0.5   # might need to be same as x_gain but opposite sign
motor_y  = 160   # defines the position from the goal during defense

# Max positions if in middle of field
x_max = 40
x_min =-60
y_max = 180
y_min = 170
z_max = 70
z_min = 0


# Throwing
##############
ready_toThrow =  False

# Idle Position
xi =  -20 # x coordinate
yi = 155 # y coordinate
zi = -35 # z coordinate
# Pre Start
xp =  0 # x coordinate
yp = 55 # y coordinate
zp = 75 # z coordinate
# Start Position
xs = -20 # x coordinate
ys =  20 # y coordinate
zs = 20 # z coordinate
# End Position
xe =   0 # x coordinate
ye = 125 # y coordinate
ze =  100 # z coordinate

############
# Position Filtering, remove outlayers
############

pos_x = x_offset # No position filtering
pos_y = z_offset

# Main Loop
#########################################################
stop = False

while (not stop):
    
    # Read Switch
    start_state  = GPIO.input(start_pin)
    switch_state = GPIO.input(switch_pin)
    str_start    = "pushed" if start_state else "not pushed"
    str_switch   = "Defense" if switch_state else "Attack"
    if text_debug: print("Start is {} and Switch is {}".format(str_start, str_switch))

    if switch_state:
        ####################################################
        # Defense
        ####################################################

        ready_toThrow = False

        #####
        # Detect Ball
        #####
        img = picam2.capture_array()
        display_img = cv2.resize(img, ((int)(width), (int)(height)), 0, 0, cv2.INTER_CUBIC)
        # Extract ROI
        proc_img = display_img[startY:endY,startX:endX,:] 
        # Process the image to find ball
        process.process(proc_img)
        # Display ball
        if len(process.ball) > 0:
            x = process.ball[1] + startX
            y = process.ball[2] + startY
            pos_x = x # no filtering
            pos_y = y
            # pos_x.append(x)
            # pos_y.append(y)
            area = process.ball[3]
            circ = process.ball[0]  
            approx = process.ball[4]
            if display_debug: 
                hsv = cv2.cvtColor(display_img, cv2.COLOR_BGR2HSV)
                cv2.drawMarker(display_img, (x, y),  (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
                txt = "{:3n},{:3n},{:3n}".format(hsv[y,x,0], hsv[y,x,1], hsv[y,x,2])
                cv2.putText(display_img, txt, (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
                txt = "{:3n},{:3.2f},{:3n}".format(area, circ, approx)
                cv2.putText(display_img, txt, (x,y+8), font, fontScale, color, thickness, cv2.LINE_AA)

        # Display all contours found
        if len(process.filter_contours_output) > 0:
            # fix the offset of the region of interest
            for contour in process.filter_contours_output:
                # contour with new offset is created
                contour += (startX, startY)
            if display_debug: cv2.drawContours(display_img, process.filter_contours_output, -1, (0,255,0), 1)
        # Show region of interest
        if display_debug:
            cv2.rectangle(display_img, (startX, startY), (endX, endY), (255,0,0), 2)
            cv2.imshow("Camera", display_img)
        
        ####
        # Move Arm depending on Ball Location
        ####
        
        ball_x = pos_x # no filtering
        ball_y = pos_y
        motor_x = x_gain*(width/2 -ball_x) + x_offset
        motor_z = z_gain*(height  -ball_y) + z_offset

        if motor_x > x_max: motor_x = x_max
        if motor_x < x_min: motor_x = x_min
        if motor_y > y_max: motor_y = y_max
        if motor_y < y_min: motor_y = y_min
        if motor_z > z_max: motor_z = z_max
        if motor_z < z_min: motor_z = z_min
                
        arm.goDirectlyTo(motor_x, motor_y, motor_z)
        
    else:
        ####################################################
        # Attack
        ####################################################
        if ready_toThrow == False:
            arm.goDirectlyTo(arm.x,arm.y-25,arm.z)
            time.sleep(0.5)
            arm.goDirectlyTo(xi,yi,zi)
            ready_toThrow = True
        if start_state and ready_toThrow:
            # Ready ...
            arm.gotoPoint(xp,yp,zp)
            time.sleep(0.5)
            # Set ...
            arm.gotoPoint(xs,ys,zs)
            time.sleep(0.5)
            # Go!!
            arm.goDirectlyTo(xe,ye,ze)
            time.sleep(0.2)
            # Relax
            arm.gotoPoint(xi,yi,zi)

    # Check if user wantS to quit
    if display_debug:
        try:
            if (cv2.waitKey(1) & 0xFF == ord('q')) : stop = True
        except: stop = True 

# Clean up
cv2.destroyAllWindows()
GPIO.cleanup()
