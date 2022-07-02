from __future__ import print_function
from dronekit import *
from pymavlink import mavutil
import time
import cv2
import numpy as np
import imutils


def set_velocity(velocity_x, velocity_y, iha):
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    iha.send_mavlink(msg)


def goto_position_target_global_int(aLocation,iha):
    msg = iha.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    iha.send_mavlink(msg)
    

def arm_and_takeoff(yukseklik,iha):
    if iha.armed == True:
        return
        
    while iha.is_armable == False:
        print("Arm durumu sorgulaniyor...")
        time.sleep(1)
    iha.mode = "GUIDED"
    while iha.mode != "GUIDED":
        print("GUIDED moda gecis yapiliyor")
        time.sleep(1)
    print("GUIDED moda gecis yapildi")
    iha.armed=True
    while iha.armed == False:
        print("IHA arm oluyor")
        time.sleep(1)
    print("IHA arm oldu.\n")
    iha.simple_takeoff(yukseklik)
    while iha.location.global_relative_frame.alt < yukseklik*0.96:
        print("TAKEOFF : Su anki yukseklik =", iha.location.global_relative_frame.alt)
        time.sleep(1)
    print("TAKEOFF : SON yukseklik =", iha.location.global_relative_frame.alt)    
    print("TAKEOFF : Islem basariyla gerceklesti\n")


def get_current_location(iha):
    return (
        iha.location.global_relative_frame.lat, 
        iha.location.global_relative_frame.lon
        )


def get_current_location_global(iha):
    return LocationGlobalRelative(
        iha.location.global_relative_frame.lat, 
        iha.location.global_relative_frame.lon, 
        iha.location._alt
        )


def detect(frame, vehicle):
    global center

    lower_red1 = np.array([0, 150, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 100])
    upper_red2 = np.array([180, 255, 255])

    index = 0

    while True:
        frame = imutils.resize(frame, width=600)

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        center = (0,0)
        vehicle_params = None

        if len(cnts) > 0:
            cnt_max = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cnt_max)
            M = cv2.moments(cnt_max)
            center = (int(x),int(y))

            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1),
                cv2.circle(frame, (int(x),int(y)), 5, (0, 255, 0), -1)

            # print('camCenter: {0}'.format(center))
            # queue.put(center)

            if index % 10 == 0:
                vehicle_params = {
                    "y_angle": vehicle.attitude.pitch,
                    "x_angle": vehicle.attitude.roll,
                    "plane_lat": vehicle.location.global_frame.lat,
                    "plane_lon": vehicle.location.global_frame.lon,
                    "plane_alt": vehicle.location._alt,
                    "center": center
                }

            index += 1

            cv2.circle(frame, (int(frame.shape[1] / 2.0), int(frame.shape[0] / 2.0)), 5, (0, 0, 0), -1)
        return frame, vehicle_params


def compute_target_pos(vehicle_params):
    camWidth = 500
    camHeight = 500
    cam_ratio = 0.16

    targetCamX = vehicle_params['center'][0]
    targetCamY = vehicle_params['center'][1]

    y_angle = vehicle_params['y_angle']
    x_angle = vehicle_params['x_angle']
    plane_lat = vehicle_params['plane_lat']
    plane_lon = vehicle_params['plane_lon']
    plane_alt = vehicle_params['plane_alt']


    cam_center_x = camWidth / 2
    cam_center_y = camHeight / 2
    cam_lat = (cam_center_x - targetCamX) / math.cos(x_angle)
    cam_lon = (cam_center_y - targetCamY) / math.cos(y_angle)
    offset_x = cam_lat * cam_ratio + plane_alt * math.tan(x_angle)
    offset_y = cam_lon * cam_ratio + plane_alt * math.tan(y_angle)
    
    target_lat = plane_lat + offset_x / 111
    target_lon = plane_lon + offset_y / 111
    # print("Plane  GPS X: {:3.8f}".format(planeGPSX), "Y: {:3.8f}".format(planeGPSY), "Alt: {:3.8f}".format(planeAltitude))
    # print(
    #     "Offset     X: {:3.8f}".format(math.atan(math.tan(aovX) * (2 * targetCamX / camWidth))),
    #     "Y: {:3.8f}".format(math.atan(math.tan(aovY) * (2 * targetCamY / camHeight)))
    #     )
    # print("Target Cam X: {:3.8f}".format(targetCamX), "Y: {:3.8f}".format(targetCamY))
    # print("Target GPS X: {:3.8f}".format(targetGPSX), "Y: {:3.8f}".format(targetGPSY))

    print(
        'x_angle: ', x_angle, 
        '\ny_angle: ', y_angle, 
        '\nplane_lat: ', plane_lat, 
        '\nplane_lon: ', plane_lon, 
        '\nplane_alt: ', plane_alt,
        '\ntarget: ', (target_lat, target_lon)
    )

    return (target_lat, target_lon)