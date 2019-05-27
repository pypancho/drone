from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

'''
print ("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
vehicle = connect(connection_string, wait_ready=True)
'''


connection_string = '/dev/ttyS0' 
baud_rate = 921600
vehicle = connect(connection_string, baud= baud_rate, wait_ready=True)


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

def condition_yaw(heading, direction = -1, relative=True):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees, 
        0,          # param 2, yaw speed deg/s, no need
        direction,   # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


takeoff_alt = 1


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

# This is the command to yaw 30 countclockwise.
print('Yaw 30 degrees CCW')
heading = 30
condition_yaw(heading = heading, direction = -1, relative=True)

# This is the command to move the copter 0.5 m/s forward for 6 sec.
print('Forward 0.5m/s for 6s')
velocity_x = 0.5
velocity_y = 0
velocity_z = 0
duration = 10
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

# This is the command to yaw 30 clockwise.
print('Yaw 30 degrees CW')
heading = 30
condition_yaw(heading = heading, direction = 1, relative=True)


# backwards at 0.5 m/s for 6 sec.
print('backwards 0.5m/s for 6s')
velocity_x = 0
velocity_y = 0.5
velocity_z = 0
duration = 10
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

print('Landing')
vehicle.mode = VehicleMode("LAND")