from dronekit import connect
from pymavlink import mavutil
import argparse
import time

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
# connection_string = "/dev/serial/by-id/usb-ArduPilot_fmuv2_27003F000651373236393336-if00"
baud_rate = 115200

# --- Now that we have started the SITL and we have the connection string (basically the ip and  udp port)...
print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud=baud_rate,
                  wait_ready=True)  # - wait_ready flag hold the program untill all the parameters are been read (=, not .)
print(">>>> Connected <<<")

print('>>>> Sended 1 <<<')
# send command to vehicle
vehicle.send_mavlink(vehicle.message_factory.command_long_encode(
    0, 0,  # target_system, target_component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    0,  # confirmation
    7,  # servo number
    1100,  # servo position between 1000 and 2000
    0, 0, 0, 0, 0))
print('>>>> Dropped <<<')

time.sleep(3)

print('>>>> Sended 2 <<<')
vehicle.send_mavlink(vehicle.message_factory.command_long_encode(
    0, 0,  # target_system, target_component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    0,  # confirmation
    7,  # servo number
    1900,  # servo position between 1000 and 2000
    0, 0, 0, 0, 0))
print('>>>> Closed <<<')

