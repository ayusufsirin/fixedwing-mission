from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil
import time



vehicle = connect('127.0.0.1:14550', wait_ready=True)





# print('Mode: %s' % vehicle.mode)

cmds = vehicle.commands
cmds.clear()
cmds.upload()

cmds.download()
cmds.wait_ready()

cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0, 0)
cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10, 10, 10)
cmd3=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 20, 10, 10)

# print(cmd1)
# print(cmd2)
# print(cmd3)


cmds.add(cmd1)
# cmds.add(cmd2)
# cmds.add(cmd3)

cmds.upload()

vehicle.mode = VehicleMode("AUTO")
print(vehicle.mode)

print(cmds)




# cmds.download()
# cmds.wait_ready()



