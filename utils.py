from __future__ import print_function
import argparse
from dronekit import *
from pymavlink import mavutil
import time
import cv2
import numpy as np
import imutils


def connect_uav():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    vehicle = connect(connection_string, wait_ready=True)

    return vehicle


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

        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        center = (0, 0)
        vehicle_params = None

        if len(cnts) > 0:
            cnt_max = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(cnt_max)
            M = cv2.moments(cnt_max)
            center = (int(x), int(y))

            if M["m00"] > 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1),
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)

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

            cv2.circle(
                frame, (int(frame.shape[1] / 2.0), int(frame.shape[0] / 2.0)), 5, (0, 0, 0), -1)
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

    target_lat = plane_lat + offset_x / 10**9
    target_lon = plane_lon + offset_y / 10**9

    print(
        'targetCam: ', (targetCamX, targetCamY),
        '\noffset_x: ', offset_x,
        '\noffset_y: ', offset_y,
        '\nx_angle: ', x_angle,
        '\ny_angle: ', y_angle,
        '\nplane_lat: ', plane_lat,
        '\nplane_lon: ', plane_lon,
        '\nplane_alt: ', plane_alt,
        '\ntarget: ', (target_lat, target_lon)
    )

    return (target_lat, target_lon)


def target_finder(pos_map):
    target_poses = []

    for vehicle_params in pos_map:
        target_poses.append(compute_target_pos(vehicle_params))

    lat_sum, lon_sum = (0, 0)
    for pos in target_poses:
        lat_sum = lat_sum + pos[0]
        lon_sum = lon_sum + pos[1]

    lat_avg = lat_sum / len(target_poses)
    lon_avg = lon_sum / len(target_poses)

    return lat_avg, lon_avg