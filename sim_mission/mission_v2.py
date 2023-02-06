from time import sleep
from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import time


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

print('##### SETTING MISSION #####')
WAYPOINTS = [
    (-35.3628811, 149.1651609, 10),  # wp1
    (-35.3636448, 149.1651607, 5),  # wp2   
    (-35.3629598, 149.1651097, 5),  # wp3
    (-35.3629905, 149.1651088, 10),  # wp4
    (-35.3635325, 149.1651095, 15),  # wp5
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

