import time
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

autopilot = mavutil.mavlink_connection('/dev/serial0')

msg = None

# wait for autopilot connection
while msg is None:
    msg = autopilot.recv_msg()

print msg

autopilot.mav.command_long_send(
    0, 0,  # target_system, target_component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    0,  # confirmation
    6,  # servo number
    1000,  # servo position between 1000 and 2000
    0, 0, 0, 0, 0)  # param 3 ~ 7 not used

time.sleep(2)

autopilot.mav.command_long_send(
    autopilot.target_system, autopilot.target_component,  # target_system, target_component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,  # confirmation
    6,  # servo number
    2000,  # servo position between 1000 and 2000
    0, 0, 0, 0, 0)  # param 3 ~ 7 not used
