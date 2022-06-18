from vincenty import vincenty
from ctypes import util
from dronekit import connect
from utils import *

# connection_string = '/dev/serial/by-id/usb-ArduPilot_fmuv2_27003F000651373236393336-if00'
connection_string = "127.0.0.1:14550"
baud_rate = 115200

WAYPOINTS = [
    (-35.3628811, 149.1651609),  # wp1
    (-35.3636448, 149.1651607),  # wp2   
    (-35.3629598, 149.1651097),  # wp3
    (-35.3629905, 149.1651088),  # wp4
    (-35.3635325, 149.1651095),  # wp5
]


def go_waypoint(vehicle, index, alt):
    location = WAYPOINTS[index]
    global_location = LocationGlobalRelative(location[0], location[1], alt)

    while not 1000 * vincenty(get_current_location(vehicle), location) < 1:
        goto_position_target_global_int(
            iha=vehicle,
            aLocation=global_location
            )

        print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(get_current_location(vehicle), location)))
        print("HIZ = ", vehicle.groundspeed)

        if not 1000 * vincenty(get_current_location(vehicle), location) < 1:
            time.sleep(1)

    print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(get_current_location(vehicle), location)))


def main():
    print(">>>> Connecting with the UAV <<<")

    vehicle = connect(
            connection_string, 
            # baud=baud_rate, 
            wait_ready=True,
            timeout=100
        )

    print(">>>> UAV is ready <<<")
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s' % vehicle.version)

    vehicle.groundspeed = 8

    arm_and_takeoff(iha=vehicle, yukseklik=10)

    go_waypoint(vehicle=vehicle, index=1, alt=5)
    go_waypoint(vehicle=vehicle, index=2, alt=5)
    go_waypoint(vehicle=vehicle, index=3, alt=5)
    go_waypoint(vehicle=vehicle, index=4, alt=5)



if __name__ == "__main__":
    main()