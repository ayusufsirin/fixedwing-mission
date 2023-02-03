import time
import cv2
from dronekit import Command
from pymavlink import mavutil

import utils

WP_IDX_TARGET_1 = 10 - 1  # index is Mission Planner id minus one
WP_IDX_TARGET_2 = 19 - 1
WP_TARGET_ALT = 10
DET_THRESH_FRAME_NUM = 50

vehicle = None
wps = []  # waypoints


def main():
    print('##### CONNECTING VEHICLE #####')
    vehicle = utils.connect_uav()
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s' % vehicle.version)
    print(vehicle.mode.__str__())
    print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

    print("##### GETTING THE WAYPOINTS #####")
    cmds = vehicle.commands

    cmds.download()
    cmds.wait_ready()

    for cmd in cmds:
        wps.append(cmd)

    print(wps[WP_IDX_TARGET_1])

    print('##### TARGET DETECTION STARTED #####')
    pos_map = []
    target_gps_pos = None

    vid = cv2.VideoCapture(0)

    while True:
        ret, frame = vid.read()
        frame, vehicle_params = utils.detect(frame=frame, vehicle=vehicle)
        
        cv2.imshow('frame', frame)

        if vehicle_params is not None:
            pos_map.append(vehicle_params)

        if len(pos_map) > DET_THRESH_FRAME_NUM:
            target_gps_pos = utils.target_finder(pos_map)
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()

    print('##### TARGET DETECTED #####')
    print('Target position: ', target_gps_pos)

    print('##### SETTING NEW WAYPOINTS #####')
    wps[WP_IDX_TARGET_1] = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
        target_gps_pos[0], target_gps_pos[1], WP_TARGET_ALT
    )

    wps[WP_IDX_TARGET_2] = Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
        target_gps_pos[0], target_gps_pos[1], WP_TARGET_ALT
    )

    print('##### UPLOADING NEW WAYPOINTS #####')
    cmds.clear()

    for wp in wps:
        cmds.add(wp)

    cmds.upload()
    cmds.wait_ready()
    print('##### NEW WAYPOINTS ARE UPLOADED #####')

    vehicle.close()
    print('##### CLOSING VEHICLE OBJECT #####')


if __name__ == '__main__':
    main()
