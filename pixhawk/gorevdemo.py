from imutils.video import VideoStream
from imutils.video import FPS
from dronekit import connect
from pymavlink import mavutil
import math
import time
import cv2
import imutils
import numpy as np
import argparse

targetGPSX = 0
targetGPSY = 0
targetCamX = 0
targetCamY = 0
lockedPosX = 0
lockedPosY = 0

releasePosX = 0
releasePosY = 0

# cv2 real-time video
vs = VideoStream(src=0).start()
time.sleep(2.0)
fps = FPS().start()
camWidth = 400
camHeight = 400
aovX = 50  # degree pi cam
aovY = 50

planeGPSX = 0
planeGPSY = 0
planeAltitude = 0
airspeed = 10  # m/s
planeRoll = 0
planeYaw = 0

distanceFromTarget = 0  # cm
dropDistance = 5  # cm

# Make connection
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
baud_rate = 115200

# --- Now that we have started the SITL and we have the connection string (basically the ip and  udp port)...
print(">>>> Connecting with the UAV <<<")

# - wait_ready flag hold the program until all the parameters are been read (=, not .)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

# - Version and attributes
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s' % vehicle.version)

# - Does the firmware support the companion pc to set the attitude?
print('Supports set attitude from companion: %s' % vehicle.capabilities.set_attitude_target_local_ned)


def compute_target_pos():
    global planeAltitude, planeGPSX, planeGPSY, targetGPSX, targetGPSY
    planeAltitude = vehicle.location._alt
    planeGPSX = vehicle.location.global_frame.lat
    planeGPSY = vehicle.location.global_frame.lon

    targetGPSX = planeGPSX + targetCamX / 400 * planeAltitude
    targetGPSY = planeGPSY + targetCamY / 400 * planeAltitude
    print(
        "Plane  GPS X: {:3.8f}".format(planeGPSX), "Y: {:3.8f}".format(planeGPSY), "Alt: {:3.8f}".format(planeAltitude))
    print("Offset     X: {:3.8f}".format(math.atan(math.tan(aovX) * (2 * targetCamX / camWidth))),
          "Y: {:3.8f}".format(math.atan(math.tan(aovY) * (2 * targetCamY / camHeight))))
    print("Target Cam X: {:3.8f}".format(targetCamX), "Y: {:3.8f}".format(targetCamY))
    print("Target GPS X: {:3.8f}".format(targetGPSX), "Y: {:3.8f}".format(targetGPSY))


def compute_release_pos():
    compute_drop_distance()
    releasePosX = 0
    releasePosY = 0
    print("Release Pos: (", releasePosX, ", ", releasePosY, ")")

    if releasePosX == planeGPSX and releasePosY == planeGPSY:
        print("Release the ball !!!")
        drop_the_ball()


def compute_drop_distance():
    global airspeed, planeAltitude, dropDistance

    airspeed = vehicle.airspeed
    planeAltitude = vehicle.location._alt
    gravity = 9.81
    dropTime = math.sqrt(2 * planeAltitude / gravity)
    dropDistance = airspeed * dropTime * 100
    print("Altitude: ", planeAltitude, "Airspeed: ", airspeed, "Drop distance: ", dropDistance)


def drop_the_ball():
    print('>>>> Sended 1 <<<')
    # send command to vehicle
    vehicle.send_mavlink(vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        7,  # servo number
        1900,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0))
    print('>>>> Dropped <<<')
    
    time.sleep(3)
    
    print('>>>> Sended 1 <<<')
    # send command to vehicle
    vehicle.send_mavlink(vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,  # confirmation
        7,  # servo number
        1100,  # servo position between 1000 and 2000
        0, 0, 0, 0, 0))
    print('>>>> Closed <<<')

# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    # frame = imutils.resize(frame, width=400)

    # Split the H channel in HSV, and get the red range
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    h[h < 175] = 0
    h[h > 180] = 0

    # normalize, do the open-morp-op
    normed = cv2.normalize(h, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
    opened = cv2.morphologyEx(normed, cv2.MORPH_OPEN, kernel)
    res = np.hstack((h, normed, opened))
    cv2.imwrite("tmp1.png", res)

    _, contours, _ = cv2.findContours(opened, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # print(len(contours))

   
    dst = frame.copy()
    areas = np.array([0])
    cnts = []
    for cnt in contours:
        # area of cnt
        area = cv2.contourArea(cnt)
        np.append(areas, area)
        cnts.append(cnt)

        print(cnt)

        if area > 1000:
            # bounding circle
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(dst, center, radius, (0, 255, 0), 2)
            cv2.drawContours(dst, [cnt], 0, (0, 255, 0), 3)
            drop_the_ball()


    # show the output frame
    cv2.imshow("images", np.hstack([frame, dst]))
    cv2.imshow("opened", opened)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # update the FPS counter
    fps.update()

    # stop the timer and display FPS information
    fps.stop()
    # print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    # print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

