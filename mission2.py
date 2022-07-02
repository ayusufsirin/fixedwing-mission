from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import time


vehicle = connect('127.0.0.1:14550', wait_ready=True, timeout=100) 
vehicle.wait_ready('autopilot_version')
print('Autopilot version: %s' % vehicle.version)
print('Mode: %s' % vehicle.mode)
print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_ready('mode')
vehicle.arm(wait=True)

print('Mode: %s' % vehicle.mode)
print('Arm/Armable: %s/%s' % (vehicle.armed, vehicle.is_armable))

vehicle.simple_takeoff(10)

# cmds = vehicle.commands

# cmds.clear()
# cmds.upload()

# cmds.download()
# cmds.wait_ready()


# missionlist=[]
# for cmd in cmds:
#     missionlist.append(cmd)
#     print(cmd)

# cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0, 0)
# cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10, 10, 10)
# cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 20, 20, 20)
# cmds.add(cmd1)
# cmds.add(cmd2)
# cmds.add(cmd3)
# cmds.upload()

# cmds.download()
# cmds.wait_ready()

# for cmd in cmds:
#     missionlist.append(cmd)

# print(missionlist)

# vehicle.mode = VehicleMode("AUTO")
# print(vehicle.mode)
