from __future__ import print_function
from vincenty import vincenty
from utils import *
from datetime import datetime


WAYPOINTS = [
    (-35.3628811, 149.1651609),  # wp1
    (-35.3636448, 149.1651607),  # wp2   
    (-35.3629598, 149.1651097),  # wp3
    (-35.3629905, 149.1651088),  # wp4
    (-35.3635325, 149.1651095),  # wp5
]


def go_waypoint(vehicle, waypoint, alt):
    location = waypoint
    global_location = LocationGlobalRelative(location[0], location[1], alt)

    while not 1000 * vincenty(get_current_location(vehicle), location) < 1:
        goto_position_target_global_int(
            iha=vehicle,
            aLocation=global_location
            )

        # print("\nHedefe ilerleniyor...\nHedefe uzaklik {:.2f}" .format(1000*vincenty(get_current_location(vehicle), location)))
        # print("HIZ = ", vehicle.groundspeed)

        if not 1000 * vincenty(get_current_location(vehicle), location) < 1:
            time.sleep(1)

    # print("\niha hedefe vardi.\nHedefe uzaklik:  {:.2f}".format(1000*vincenty(get_current_location(vehicle), location)))


def main(vehicle, target_pos):
    vehicle.groundspeed = 8

    arm_and_takeoff(iha=vehicle, yukseklik=10)

    while True:
        (x, y) = target_pos

        print(WAYPOINTS[1], x.value, y.value)
        go_waypoint(vehicle=vehicle, waypoint=WAYPOINTS[0], alt=15)
        print("WP1")
        go_waypoint(vehicle=vehicle, waypoint=WAYPOINTS[1], alt=15)
        print("WP2")
        go_waypoint(vehicle=vehicle, waypoint=WAYPOINTS[2], alt=15)
        print("WP3")
        # go_waypoint(vehicle=vehicle, index=3, alt=15)
        # go_waypoint(vehicle=vehicle, index=4, alt=15)
        
    go_waypoint(vehicle=vehicle, waypoint=target_pos, alt=15)
    print("Mission completed !!!, Current Position: ", get_current_location_global(vehicle))
    

# def run(waypoints, is_target_found, target_pos):
    # BaseManager.register('Vehicle', Vehicle)
    # manager = BaseManager()
    # manager.start()
    # inst = manager.Vehicle()
    # x_angle = vehicle.attitude.pitch
    # y_angle = vehicle.attitude.roll
    # plane_lat = vehicle.location.global_frame.lat
    # plane_lon = vehicle.location.global_frame.lon
    # plane_alt = vehicle.location._alt
    # print(
    #     'x_angle: ', x_angle, 
    #     '\ny_angle: ', y_angle, 
    #     '\nplane_lat: ', plane_lat, 
    #     '\nplane_lon: ', plane_lon, 
    #     '\nplane_alt: ', plane_alt
    # )

    # manager = Manager()
    # return_dict = manager.dict()
    
    # t1 = Process(target=target_pos_finder, args=(queue, return_dict))
    # t2 = Process(target=run, args=(WAYPOINTS, False, "",))
    
    # t1.daemon = True
    # t1.start()
    # t2.start()

    # t1.join()
    # t2.join()

    # if return_dict.values['target_pos']:
    # print(return_dict.values())

    # run(vehicle, WAYPOINTS, False, "")
    # pass


def target_pos_finder(vehicle, sharedQueue, return_dict):
    cam_poses = []
    gps_poses = []

    with open('/home/dad/FixedWing/mission/logs.txt', 'a+') as f:
        print("Log file opened")

        while len(gps_poses) < 20:
            target_cam_pos = sharedQueue.get()
            cam_poses.append(target_cam_pos)
            print(target_cam_pos)

            iha_gps_pos = get_current_location_global(iha=vehicle)
            target_gps_pos = compute_target_pos(vehicle=vehicle, cam=target_cam_pos)
            gps_poses.append(target_gps_pos)
            
            log = '{0} :: target_cam_pos: {1}, iha_gps_pos: {2}, target_gps_pos: {3}'.format(
                datetime.now(), 
                target_cam_pos, 
                iha_gps_pos,
                target_gps_pos
                )

            # print(log)
            print(log, file=f)

        x_sum, y_sum = (0, 0)
        for pos in gps_poses:
            x_sum = x_sum + pos[0]
            y_sum = y_sum + pos[1]

        x_avg = x_sum / len(gps_poses)
        y_avg = y_sum / len(gps_poses)

        target_gps_pos = (x_avg, y_avg)

        print("Target Locked At: ", target_gps_pos)
        return_dict['target_pos'] = target_gps_pos

    return True
