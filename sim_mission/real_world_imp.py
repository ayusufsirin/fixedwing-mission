from cmath import cos, pi, sin

import cv2

import numpy as np

import imutils

from dronekit import connect

from pymavlink import mavutil

import time

import cv2

import numpy as np

 

def detect(frame):

    global center

 

    lower_red1 = np.array([0, 125, 75])

    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 150, 100])

    upper_red2 = np.array([180, 255, 255])

 

    while True:

       

 

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

 

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

       

        mask = cv2.add(mask1,mask2 )

 

        cv2.imshow("filtered",mask)

        cv2.waitKey(5)

 

        mask_cp = mask.copy()

 

        cnts = cv2.findContours(mask_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cnts = imutils.grab_contours(cnts)

       

        x_out = 0

        y_out = 0

        r_out = 0

       

        center = (0,0)

 

        if len(cnts) > 0:

            cnt_max = max(cnts, key=cv2.contourArea)

           

            ((x, y), radius) = cv2.minEnclosingCircle(cnt_max)

            #radii = 2*radius

            M = cv2.moments(cnt_max)

            center = (int(x),int(y))

 

            x_out = int(x)

            y_out = int(y)

            r_out = radius

           

 

            if M["m00"] > 0:

                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

               

            if radius > 10:

                cv2.circle(frame, (int(x), int(y)), int(radius),

                            (0, 255, 255), 2)

                cv2.circle(frame, center, 5, (0, 0, 255), -1),

                cv2.circle(frame, (int(x),int(y)), 5, (0, 255, 0), -1)

 

            # print('camCenter: {0}'.format(center))

           

 

        cv2.circle(frame, (int(frame.shape[1] / 2.0), int(frame.shape[0] / 2.0)), 5, (0, 0, 0), -1)

        cv2.imshow("with contour",frame)

       

        return [x_out,y_out,r_out]

vehicle = connect('127.0.0.1:14550', wait_ready=True)
cap = cv2.VideoCapture(0)

real_radius = float(input("Enter the radius of the target: "))
 

while True:

 

    _, frame = cap.read()
    frame_log = detect(frame) #0th element = X, 1st element = Y, 2nd element = radius


    cv2.imshow("frame",frame)

 

    if(frame_log[2] == 0):
        continue


    k = real_radius / frame_log[2]

 

    if ((frame_log[0]<320) & (frame_log[1]<240)):
        x_distance = (frame_log[0]-320)*k
        y_distance = (240-frame_log[1])*k

 

    elif((frame_log[0] <320) & (frame_log[1]>=240)):
        x_distance = (frame_log[0]-320)*k
        y_distance = (240-frame_log[1])*k

 

    elif((frame_log[0] >= 320) & (frame_log[1] < 240)):
        x_distance = (frame_log[0]-320)*k
        y_distance = (240-frame_log[1])*k

   

    elif((frame_log[0] >= 320) & (frame_log[1] >= 240)):
        x_distance = (frame_log[0]-320)*k
        y_distance = (240-frame_log[1])*k

 

    else:
        continue

   

    yaw_radians = vehicle.attitude.yaw
    pitch_radians = vehicle.attitude.pitch
    roll_radians = vehicle.attitude.roll

 

    lat_distance = y_distance * cos(pi/2 - yaw_radians) + x_distance * cos(yaw_radians)
    lon_distance = y_distance * sin(pi/2 - yaw_radians) - x_distance * cos(pi/2 - yaw_radians)

 

    vh_latitude = vehicle.location.global_relative_frame.lat,
    vh_longitude = vehicle.location.global_relative_frame.lon,
    vh_altitude = vehicle.location._alt

 
    lat_addition = lat_distance / 24.42 / 3600
    lon_addition = lon_distance / 31.14 / 3600

 

    tg_latitude = vh_latitude + lat_addition
    tg_longitude = vh_longitude + lon_addition

   

    if((abs(roll_radians) < 0.087266462) & (abs(pitch_radians) < 0.087266462)):
        print(tg_latitude, tg_longitude) # modify and add waypoint if printed values are correct

   
    if cv2.waitKey(1) == ord("q"):

        break

   

cap.release()

cv2.destroyAllWindows()
