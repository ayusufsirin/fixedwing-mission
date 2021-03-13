# coding=utf-8
# Hatice Karahan
# Dronekit kütüphanesi ile otonom uçuş


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException, Command
import time
import math
import argparse
from pymavlink import mavutil


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

# MAIN EXECUTABLE #


vehicle = connect_uav()

speed_type = 1
speed = 15
throttle_setting = 0

# List of commands
cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7069803, 32.7520519, 15)
cmd2 = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0, 0, speed_type, speed, throttle_setting, 0, 0, 0, 0)
cmd3 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7069390, 32.7529854, 15)
cmd4 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7073105, 32.7536666, 15)
cmd5 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7078057, 32.7534735, 15)
cmd6 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7081070, 32.7529800, 15)
cmd7 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7080657, 32.7521485, 15)
cmd8 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                       0, 0, 0, 0, 0, 39.7078016, 32.7517086, 15)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready

cmds.clear

cmds.add(cmd1)
cmds.add(cmd2)
cmds.add(cmd3)
cmds.add(cmd4)
cmds.add(cmd5)
cmds.add(cmd6)
cmds.add(cmd7)
cmds.add(cmd8)

vehicle.commands.upload()
print('Yuklendi')

while True:
    if vehicle.mode == 'AUTO':

#         vehicle.wait_ready('autopilot_version')
        print('Autopilot version: %s' % vehicle.version)
        print('Position: %s' % vehicle.location.global_relative_frame)
        print('Attitude: %s' % vehicle.attitude)
        print('Velocity: %s' % vehicle.velocity)
        print('Is the vehicle armable: %s' % vehicle.is_armable)
        print('Groundspeed: %s', vehicle.groundspeed)
        print('Mode: %s' % vehicle.mode.name)
        print('Armed: %s' % vehicle.armed)
        print('EKF Ok: %s' % vehicle.ekf_ok)
        print('GPS: %s' % vehicle.gps_0)

        time.sleep(30)
        
        vehicle.close()
        print ("Vehicle object closed")
        

#         if vehicle.sitl is not None:
#             vehicle.sitl.stop()
    else:
        print ("AUTO modda degil")
        time.sleep(1)
