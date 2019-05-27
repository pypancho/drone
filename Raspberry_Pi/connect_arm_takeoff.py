from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import datetime
import math

connection_string = '/dev/ttyS0' 
baud_rate = 921600
aTargetAltitude = 1

def conncect_drone(connection_string, baud_rate):
    
    #--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...
    
    print(">>>> Connecting with the UAV <<<")
    vehicle = connect(connection_string, baud= baud_rate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are been read (=, not .)
    print('vehicle is connected.')
    #-- Read information from the autopilot:
    #- Version and attributes
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s'%vehicle.version)
    
    #- Does the firmware support the companion pc to set the attitude?
    print('Supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)
    
    #- Read the actual position
    print('Position: %s'% vehicle.location.global_relative_frame)
    
    #- Read the actual attitude roll, pitch, yaw
    print('Attitude: %s'% vehicle.attitude)
    
    #- Read the actual velocity (m/s)
    print('Velocity: %s'%vehicle.velocity) #- North, east, down
    
    #- When did we receive the last heartbeat
    print('Last Heartbeat: %s'%vehicle.last_heartbeat)
    
    #- Is the vehicle good to Arm?
    print('Is the vehicle armable: %s'%vehicle.is_armable)
    
    #- Which is the total ground speed?   Note: this is settable
    print('Groundspeed: %s'% vehicle.groundspeed) #(%)
    
    #- What is the actual flight mode?    Note: this is settable
    print('Mode: %s'% vehicle.mode.name)
    
    #- Is the vehicle armed               Note: this is settable
    print('Armed: %s'%vehicle.armed)
    
    #- Is thestate estimation filter ok?
    print('EKF Ok: %s'%vehicle.ekf_ok)
    return vehicle




def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    #print(datetime.datetime.now())
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        #print(datetime.datetime.now())
        time.sleep(1)
    print ("After Initialising, Battery: %s" % vehicle.battery)
	
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        #print(datetime.datetime.now())
        time.sleep(1)
    print ("After Arming, Battery: %s" % vehicle.battery)
	
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    print(datetime.datetime.now())
    print ("Taking off, Battery: %s" % vehicle.battery)

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    print(datetime.datetime.now())
    while True:
        #print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        print ("Taking off, Battery: %s" % vehicle.battery)
        print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

vehicle = conncect_drone(connection_string, baud_rate)
arm_and_takeoff(aTargetAltitude)

print('Landing')
vehicle.mode = VehicleMode("LAND")