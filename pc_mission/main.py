import os
import time
import argparse
from dronekit import connect, VehicleMode
from multiprocessing import Process

import mission

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--port')
args = parser.parse_args()

serial_port = args.port
connection_string_1 = "127.0.0.1:14550"
connection_string_2 = "127.0.0.1:14551"

original_wps = []


def setup_proxy():
    os.system(
        f'mavproxy.py --master={serial_port} --out {connection_string_1} --out {connection_string_2} --daemon')


if __name__ == '__main__':
    p_proxy = Process(target=setup_proxy)
    p_mission = Process(target=mission.main, args=(connection_string_2,))

    p_proxy.start()
    
    vehicle = connect(connection_string_1)

    # Backup original waypoints
    print('##### BACKUP ORIGINAL WAYPOINTS #####')
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    for cmd in cmds:
        original_wps.append(cmd)

    # Vehicle mode FSM
    while True:
        if vehicle.mode != VehicleMode('AUTO'):
            # Reset mission
            if p_mission.is_alive():
                print('##### RESET MISSION #####')
                p_mission.kill()

                # Restore original waypoints
                cmds.clear()

                print('##### RESTORE ORIGINAL WAYPOINTS #####')
                for wp in original_wps:
                    cmds.add(wp)

                cmds.upload()

            p_mission = Process(target=mission.main, args=(connection_string_2,))

            print('Change mode to AUTO |', end='\r')
            time.sleep(0.5)
            print('Change mode to AUTO /', end='\r')
            time.sleep(0.5)
            print('Change mode to AUTO -', end='\r')
            time.sleep(0.5)
            print('Change mode to AUTO \\', end='\r')
            time.sleep(0.5)
        elif not p_mission.is_alive():
            print('Mission started.')
            p_mission.start()
