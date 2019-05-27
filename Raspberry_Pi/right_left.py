from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

print ("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        #- Read the actual position
        print('Position: %s'% vehicle.location.global_relative_frame)
        time.sleep(1)


takeoff_alt = 10
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.is_armable:
    time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print('Waiting for arming...')
    time.sleep(1)
vehicle.simple_takeoff(takeoff_alt) # Take off to target altitude
while True:
    print('Altitude: %d' %  vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= takeoff_alt * 0.95:
        print('REACHED TARGET ALTITUDE')
        break
    time.sleep(1)

# This is the command to move the copter 5 m/s forward for 10 sec.
print('Forward 5m/s for 10s')
velocity_x = 0
velocity_y = 5
velocity_z = 0
duration = 10
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

# backwards at 5 m/s for 10 sec.
print('backwards 5m/s for 10s')
velocity_x = 0
velocity_y = -5
velocity_z = 0
duration = 10
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

print('Landing')
vehicle.mode = VehicleMode("LAND")