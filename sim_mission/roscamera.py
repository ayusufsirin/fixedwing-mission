#!/usr/bin/env python2
#!/usr/bin/env python3

from multiprocessing import Manager, Queue
from multiprocessing.process import Process
from tkinter.messagebox import NO
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import roslib
from sensor_msgs.msg import Image

from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import time

import utils

roslib.load_manifest('roscamera')

vehicle = None

global_pos_map = []
target_gps_pos = None

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image, vehicle_params = utils.detect(cv_image, vehicle)
            
            if vehicle_params is not None:
                global_pos_map.append(vehicle_params)
                print(len(global_pos_map))

            
            if len(global_pos_map) > 50:
                self.target_finder()
            
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Kamera", cv_image)
        cv2.waitKey(3)

    def target_finder(self):
        target_poses = []
        for vehicle_params in global_pos_map:
            target_poses.append(utils.compute_target_pos(vehicle_params))

        lat_sum, lon_sum = (0, 0)
        for pos in target_poses:
            lat_sum = lat_sum + pos[0]
            lon_sum = lon_sum + pos[1]

        lat_avg = lat_sum / len(target_poses)
        lon_avg = lon_sum / len(target_poses)

        global target_gps_pos
        target_gps_pos = (lat_avg, lon_avg)
        
        print(target_gps_pos)
        rospy.signal_shutdown("Target Found")

        # raise Exception("Target Found: %" % target_gps_pos)


def main():
    global vehicle
    print('##### CONNECTING VEHICLE #####')
    vehicle = connect('127.0.0.1:14550', wait_ready=True)
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s' % vehicle.version)
    print(vehicle.mode.__str__())
    print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

    print('##### ARMING #####')
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.wait_ready('mode')
    vehicle.arm(wait=True)

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print('##### TAKING OFF #####')
    vehicle.simple_takeoff(10)  # Different in plane !!!
    vehicle.groundspeed = 1

    print('##### SETTING MISSION #####')
    WAYPOINTS = [
        (-35.3628811, 149.1651609, 10),  # wp1
        (-35.3636448, 149.1651607, 10),  # wp2   
        (-35.3629598, 149.1651097, 10),  # wp3
        (-35.3629905, 149.1651088, 10),  # wp4
        (-35.3635325, 149.1651095, 10),  # wp5
        # (-35.3628811, 149.1651609, 10),  # wp1
        # (-35.3636448, 149.1651607, 10),  # wp2   
        # (-35.3629598, 149.1651097, 10),  # wp3
        # (-35.3629905, 149.1651088, 10),  # wp4
        # (-35.3635325, 149.1651095, 10),  # wp5
    ]
    # WAYPOINTS = [
    #     # (-35.36336723, 149.16511541, 5) # TARGET POS
    #       (-35.5253992886674, 149.12460445221475)    
    #     (-35.3633695, 149.1651198, 5)
    # ]
    # -35.36353352 149.16512011

    # (-35.84774165661214, 149.21752812729568)
    # (-35.35797153581433, 148.84267474291096)
    # (-35.36280677701517, 148.97665146687967)
    # (-35.531232697377646, 149.0875114714931)
    #  -35.36364473 149.16515863
    #  -35.36364459 149.16515838
    # (-35.367236445197776, 149.167986936256)

    cmds = vehicle.commands

    # clear previous mission
    cmds.download()
    cmds.wait_ready()
    wps = []
    for cmd in cmds:
        wps.append(cmd)
    print('Current Mission:', wps)

    cmds.clear()
    cmds.upload()
    wps = []
    for cmd in cmds:
        wps.append(cmd)
    print('Cleared Mission:', wps)

    # setting first waypoint: TAKEOFF
    lat, lon, alt = WAYPOINTS[0]
    cmds.add(Command( 
            0, 0, 0, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, 0, 0, 
            lat, lon, alt
        ))

    # setting waypoints
    for lat, lon, alt in WAYPOINTS:
        cmds.add(Command( 
                    0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                    lat, lon, alt
                ))

    # uploading mission
    print(' ## Uploading new mission... ##\n')
    for cmd in cmds:
        print('     -> %s' % cmd)

    cmds.upload()
    print(' ## Uploaded ##\n')

    print('##### STARTING MISSION #####')
    vehicle.mode = VehicleMode("AUTO")

    while vehicle.mode != 'AUTO':
        time.sleep(1)

    print(vehicle.mode.__str__())
    print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

    print('##### MISSION INITIALIZED #####')

    print("Initializing ROS Camera ...")
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
    
    print('### GOING TO TARGET ###')
    vehicle.mode = VehicleMode("GUIDED")

    while vehicle.mode != 'GUIDED':
        time.sleep(1)

    WAYPOINTS = [
        (target_gps_pos[0], target_gps_pos[1], 10)
        # (-35.36336723, 149.16511541, 5)
    ]

    cmds = vehicle.commands

    # clear previous mission
    cmds.download()
    cmds.wait_ready()
    wps = []
    for cmd in cmds:
        wps.append(cmd)
    print('Current Mission:', wps)

    cmds.clear()
    cmds.upload()
    wps = []
    for cmd in cmds:
        wps.append(cmd)
    print('Cleared Mission:', wps)

    # setting first waypoint: TAKEOFF
    # lat, lon, alt = WAYPOINTS[0]
    # cmds.add(Command( 
    #         0, 0, 0, 
    #         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
    #         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
    #         0, 0, 0, 0, 0, 0, 
    #         lat, lon, alt
    #     ))

    # setting waypoints
    for lat, lon, alt in WAYPOINTS:
        cmds.add(Command( 
                    0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                    lat, lon, alt
                ))

    # uploading mission
    print(' ## Uploading new mission... ##\n')
    for cmd in cmds:
        print('     -> %s' % cmd)

    cmds.upload()
    print(' ## Uploaded ##\n')

    print('##### STARTING MISSION #####')
    vehicle.mode = VehicleMode("AUTO")

    while vehicle.mode != 'AUTO':
        time.sleep(1)

    print(vehicle.mode.__str__())
    print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

    print('##### TARGET MISSION INITIALIZED #####')  


if __name__ == "__main__":
    main()

