# coding=utf-8
# Hatice Karahan
# Dronekit kütüphanesi ile otonom uçuş versiyon2


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


def get_distance_meters(targetLocation, currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5


def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
    vehicle.simple_goto(targetLocation)
    while vehicle.mode.name == 'AUTO':
        print('Vehicle Groundspeed: %s' % vehicle.groundspeed)
        print('Vehicle Airspeed: %s' % vehicle.airspeed)
        currentDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
        print("Current distance: %d" % currentDistance)
        print("distanceToTargetLocation: %d" % distanceToTargetLocation)
        print("currentDistance < distanceToTargetLocation*.20: %s" % currentDistance < distanceToTargetLocation*.20)
        if currentDistance < distanceToTargetLocation*.20:
            print ("Reached target waypoint")
            time.sleep(2)
            break
        time.sleep(1)
    return None


# MAIN EXECUTABLE #

vehicle = connect_uav()

wp1 = LocationGlobalRelative(39.7069803, 32.7520519, 15)
wp2 = LocationGlobalRelative(39.7069390, 32.7529854, 15)
wp3 = LocationGlobalRelative(39.7073105, 32.7536666, 15)
wp4 = LocationGlobalRelative(39.7078057, 32.7534735, 15)
wp5 = LocationGlobalRelative(39.7081070, 32.7529800, 15)
wp6 = LocationGlobalRelative(39.7080657, 32.7521485, 15)
wp7 = LocationGlobalRelative(39.7078016, 32.7517086, 15)


while True:
    if vehicle.mode == 'AUTO':
        vehicle.wait_ready('autopilot_version')
        print('Autopilot version: %s' % vehicle.version)
        print('Position: %s' % vehicle.location.global_relative_frame)
        print('Attitude: %s' % vehicle.attitude)
        print('Velocity: %s' % vehicle.velocity)
        print('Last Heartbeat: %s' % vehicle.last_heartbeat)
        print('Is the vehicle armable: %s' % vehicle.is_armable)
        print('Groundspeed: %s', vehicle.groundspeed)
        print('Mode: %s' % vehicle.mode.name)
        print('Armed: %s' % vehicle.armed)
        print('EKF Ok: %s' % vehicle.ekf_ok)
        print('GPS: %s' % vehicle.gps_0)

        vehicle.groundspeed = 10
        print('Vehicle Groundspeed: %s' % vehicle.groundspeed)
        print('Vehicle Airspeed: %s' % vehicle.airspeed)

        goto(wp1)
        goto(wp2)
        goto(wp3)
        goto(wp4)
        goto(wp5)
        goto(wp6)
        goto(wp7)

        print('Vehicle Groundspeed: %s' % vehicle.groundspeed)
        print('Vehicle Airspeed: %s' % vehicle.airspeed)

        vehicle.close()
        print ("Vehicle object closed")

        if vehicle.sitl is not None:
            vehicle.sitl.stop()
    else:
        print ("AUTO modda degil")
        time.sleep(1)
