import time
import cv2
from dronekit import Command, VehicleMode, connect
from pymavlink import mavutil

import utils

WP_IDX_TARGET_1 = 10 - 1  # index is Mission Planner id minus one
WP_IDX_TARGET_2 = 19 - 1
WP_TARGET_ALT = 10
DET_THRESH_FRAME_NUM = 50

vehicle = None
wps = []  # waypoints


def main(connection_string):
    print('##### CONNECTING VEHICLE #####')
    vehicle = connect(connection_string)
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s' % vehicle.version)
    print(vehicle.mode.__str__())
    print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

    # print("##### GETTING THE WAYPOINTS #####")
    # cmds = vehicle.commands

    # cmds.download()
    # cmds.wait_ready()

    # for cmd in cmds:
    #     wps.append(cmd)

    print('##### TARGET DETECTION STARTED #####')
    frame_map = {}
    pos_list = []
    target_gps_pos = None

    vid = cv2.VideoCapture(0)

    while True:
        ret, frame = vid.read()
        frame, vehicle_params = utils.detect(frame=frame, vehicle=vehicle)

        if vehicle_params is not None:
            frame_map[f'{time.time()}_{vehicle_params}.jpg'] = frame
            pos_list.append(vehicle_params)

        if len(pos_list) > DET_THRESH_FRAME_NUM:
            target_gps_pos = utils.target_finder(pos_list)
            break

    vid.release()

    print('##### TARGET DETECTED #####')
    print('Target position: ', target_gps_pos)
    
    for filename, frame in frame_map:
        cv2.imwrite(filename, frame)

    # print('##### SETTING NEW WAYPOINTS #####')
    # wps[WP_IDX_TARGET_1] = Command(
    #     0, 0, 0,
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
    #     target_gps_pos[0], target_gps_pos[1], WP_TARGET_ALT
    # )

    # wps[WP_IDX_TARGET_2] = Command(
    #     0, 0, 0,
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
    #     target_gps_pos[0], target_gps_pos[1], WP_TARGET_ALT
    # )

    # print('##### UPLOADING NEW WAYPOINTS #####')
    # cmds.clear()

    # for wp in wps:
    #     cmds.add(wp)

    # cmds.upload()
    # print('##### NEW WAYPOINTS ARE UPLOADED #####')
    
    while True:
        time.sleep(3)
        print('At WP: ', vehicle.commands.next)


if __name__ == '__main__':
    main('COM23')