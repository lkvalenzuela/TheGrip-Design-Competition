#!/usr/bin/python3
# To make this work:
# sudo apt install -y python3-picamera2

import cv2
import meArm
import atexit
import csv
import time
from picamera2 import Picamera2
from grip import WhiteBall



arm = meArm.meArm()
arm.begin(block = 0, address = 0x70) # block address of motor controller

cv2.startWindowThread()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

process = WhiteBall()

font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 0.25
color = (0, 255, 0)
thickness = 1

# writing into csv file
#f = open('BallLocatorData1.csv', 'w', newline='')
#fieldnames = ['Set Speed', 'Measured Speed']
#writer = csv.DictWriter(f,fieldnames=fieldnames)
#writer.writeheader()

# Main Loop
stop = False

while (not stop):
    img = picam2.capture_array()
    display_img = cv2.resize(img, ((int)(320), (int)(240)), 0, 0)
    
    process.process(img)

    if len(process.ball) > 0:
        x = process.ball[1] + process.startX
        y = process.ball[2] + process.startY
        hsv = cv2.cvtColor(display_img, cv2.COLOR_BGR2HSV)
        cv2.drawMarker(display_img, (x, y),  (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
        txt = "{},{},{}".format(hsv[y,x,0], hsv[y,x,1], hsv[y,x,2])
        #time.sleep(.01)
        print(x,y)
        arm.gotoPoint(x-150, y-100, 100)
    

        #cv2.putText(display_img, txt, (x,y), font, fontScale, color, thickness, cv2.LINE_AA)
        #writer.writerow({'Coords1' : x, 'Coords 2' : y})
        
        
    if len(process.filter_contours_output) > 0:
        # fix the offset of the region of interest
        x_offset, y_offset = process.startX, process.startY
        for contour in process.filter_contours_output:
            # contour with new offset is created
            contour += (x_offset, y_offset)
        cv2.drawContours(display_img, process.filter_contours_output, -1, (0,255,0), 1)

    x1=process.startX
    y1=process.startY
    x2=process.endX
    y2=process.endY
    cv2.rectangle(display_img, (x1, y1), (x2, y2), (255,0,0), 2)
    cv2.imshow("Camera", display_img)
    
    #print(x2, y2)
    
    try:
        if (cv2.waitKey(1) & 0xFF == ord('q')) or (cv2.getWindowProperty("Camera", 0) < 0): stop = True
    except: stop = True 

cv2.destroyAllWindows()

