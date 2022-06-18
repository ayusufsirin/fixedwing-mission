from dronekit import *
from pymavlink import mavutil
import time


def set_velocity(velocity_x, velocity_y, iha):
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    iha.send_mavlink(msg)


def goto_position_target_global_int(aLocation,iha):
    msg = iha.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        int(aLocation.lat*1e7), # lat_int - X Position in WGS84 frame in 1e7 * meters
        int(aLocation.lon*1e7), # lon_int - Y Position in WGS84 frame in 1e7 * meters
        int(aLocation.alt), # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    iha.send_mavlink(msg)
    

def arm_and_takeoff(yukseklik,iha):
    if iha.armed == True:
        return
        
    while iha.is_armable == False:
        print("Arm durumu sorgulaniyor...")
        time.sleep(1)
    iha.mode = "GUIDED"
    while iha.mode != "GUIDED":
        print("GUIDED moda gecis yapiliyor")
        time.sleep(1)
    print("GUIDED moda gecis yapildi")
    iha.armed=True
    while iha.armed == False:
        print("IHA arm oluyor")
        time.sleep(1)
    print("IHA arm oldu.\n")
    iha.simple_takeoff(yukseklik)
    while iha.location.global_relative_frame.alt < yukseklik*0.96:
        print("TAKEOFF : Su anki yukseklik =", iha.location.global_relative_frame.alt)
        time.sleep(1)
    print("TAKEOFF : SON yukseklik =", iha.location.global_relative_frame.alt)    
    print("TAKEOFF : Islem basariyla gerceklesti\n")


def get_current_location(iha):
    return (
        iha.location.global_relative_frame.lat, 
        iha.location.global_relative_frame.lon
        )


def get_current_location_global(yukseklik,iha):
    return LocationGlobalRelative(
        iha.location.global_relative_frame.lat, 
        iha.location.global_relative_frame.lon, yukseklik
        )
