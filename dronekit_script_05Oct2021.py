#!/usr/bin/env python

# Import DroneKit-Python
from dronekit import connect,  VehicleMode, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math, matplotlib, threading, requests
import matplotlib.pyplot as plt
import sys
import re


################################################################################################
# Settings
################################################################################################

connection_string       = '127.0.0.1:14540' # default
MAV_MODE_AUTO   = 4
# https://github.com/PX4/PX4-Autopilot/blob/master/Tools/mavlink_px4.py

# Parse connection argument
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
parser.add_argument("-m", "--mission", action='store', help="mission file path")
parser.add_argument("-f", "--faults", action='store', help="faults file path")
parser.add_argument("-type", "--typerun", action='store', help="type of run, 0 for Golden, 1 for Faulty")
args = parser.parse_args()

if args.connect:
    connection_string = args.connect

#Plot Data
start_time = time.time()
times = []
distances_east = []
distances_north = []
distances_down = []
has_finished = False
takeoff_time = 0



################################################################################################
# Init
################################################################################################

def print_with_flush(msg):
    print(msg)
    sys.stdout.flush()



def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)

def update_values():
    t = threading.Timer(2.0, update_values)
    t.start()
    if(has_finished == True):
        t.cancel()
    if(vehicle.armed == True):
        times.append(time.time() - start_time)
        distances_east.append(vehicle.location.local_frame.east)
        distances_north.append(vehicle.location.local_frame.north)
        distances_down.append(vehicle.location.local_frame.down)
        #print "Updating values"

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)


def readmission(aFileName):
    """
    Load a mission from a file into a list. 
    This function is used by upload_mission().
    """
    print_with_flush("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    home = vehicle.location.global_relative_frame
    missionread = False
    with open(aFileName) as f:
        cur_wp = 0
        for i, line in enumerate(f):
            linearray=re.split('    |\r\n',line)
            print_with_flush(linearray)
	    if len(linearray) <= 2:
                if linearray[0]!='':
		    takeoff_time = int(linearray[0])
            else:
		    ln_target_system = int(linearray[0])
		    ln_target_component = int(linearray[1])
		    ln_seq = int(linearray[2])
		    ln_frame = int(linearray[3])
		    if ln_frame == 0:
		        ln_frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
		    ln_current = int(linearray[5])
		    ln_autocontinue = int(linearray[6])
		    ln_param1 = float(linearray[7])
		    ln_param2 = float(linearray[8])
		    ln_param3 = float(linearray[9])
		    ln_param4 = float(linearray[10])
		    ln_paramx = float(linearray[11])
		    ln_paramy = float(linearray[12])
		    ln_paramz = max(float(linearray[13]),5)
		    ln_command = int(linearray[4])
		    print_with_flush(ln_command)
		    if ln_command == 0:
		        ln_command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
		        wp = get_location_offset_meters(home, ln_paramx, ln_paramy, ln_paramz);
		        cur_wp = get_location_offset_meters(home, ln_paramx, ln_paramy, ln_paramz);
		    elif ln_command == 1:
		        ln_command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
		        #print_with_flush(cur_wp)
		        wp = get_location_offset_meters(cur_wp, ln_paramx, ln_paramy, ln_paramz);
		        #print_with_flush(wp)
		        cur_wp = wp
		    elif ln_command == 2:
		        ln_command = mavutil.mavlink.MAV_CMD_NAV_LAND
		        wp = get_location_offset_meters(home, ln_paramx, ln_paramy, ln_paramz);
			missionread = True
		    cmd = Command(ln_target_system, ln_target_component, ln_seq, ln_frame, ln_command, ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, wp.lat, wp.lon, wp.alt)
		    missionlist.append(cmd)
		    if missionread:
			break
		    
		   
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    print_with_flush("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print_with_flush(' Clear mission ')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print_with_flush(' Upload mission ')
    cmds.upload()




################################################################################################
# Connect to the vehicle
################################################################################################


print_with_flush("Connecting...")
vehicle = connect(connection_string, wait_ready=True)



################################################################################################
# Listeners
################################################################################################

home_position_set = False

#Create a message listener for home position fix
@vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    home_position_set = True

#@vehicle.on_attribute('armed')
#def arming_listener(self, name, value):
#    if(value == False):
#        print_with_flush("Vehicle disarmed")

#        has_finished = True
#        time.sleep(1)

        #print_with_flush("Drawing run plot")
        #plt.plot(times, distances_north, 'r', label='X coordinate')
        #plt.plot(times, distances_east, 'g', label='Y coordinate')
        #plt.plot(times, distances_down, 'b', label='Z coordinate')

        #plt.legend(loc=2)
        #plt.xlabel('Time (s)')
        #plt.ylabel('GPS Coordinates')
        #if(args.typerun == '0'):
        #    plt.title("Golden Run")
        #    plt.savefig('drone_golden_run.png')
        #    print_with_flush("Drawn golden run plot")
        #elif(args.typerun == '1'):
        #    plt.title("Faulty Run")
        #    plt.savefig('drone_faulty_run.png')
        #    print_with_flush("Drawn faulty run plot")

       
     

################################################################################################
# Start mission example
################################################################################################
 
update_values()

# wait for a home position lock
while not home_position_set:
    print_with_flush("Waiting for home position...")
    time.sleep(1)

# Display basic vehicle state
print_with_flush(" Type: %s" % vehicle._vehicle_type)
print_with_flush(" Armed: %s" % vehicle.armed)
print_with_flush(" System status: %s" % vehicle.system_status.state)
print_with_flush(" GPS: %s" % vehicle.gps_0)
print_with_flush(" Alt: %s" % vehicle.location.global_relative_frame.alt)

# wait
time.sleep(takeoff_time)


 

# Change to AUTO mode
print_with_flush("Change to MAV_MODE_AUTO mode...")
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)

#vehicle.home_location = vehicle.location.global_frame


# send and save the file including faults to the UAV that is target of fault injection



# Upload mission
print_with_flush("Uploading mission")
upload_mission(args.mission)
time.sleep(2)

# Arm vehicle
vehicle.armed = True

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    if vehicle.commands.next > nextwaypoint:
        display_seq = vehicle.commands.next+1
        print_with_flush("Moving to waypoint %s" % display_seq)
        nextwaypoint = vehicle.commands.next
    time.sleep(1)

# wait for the vehicle to land
while vehicle.commands.next > 0:
    time.sleep(1)

#print("Setting LAND mode...")
#vehicle.mode = VehicleMode("LAND")

# Disarm vehicle
print_with_flush("Disarming the vehicle")
vehicle.armed = False
time.sleep(5)

print_with_flush("Vehicle disarmed")
has_finished = True
time.sleep(1)


# Close vehicle object before exiting script
print_with_flush("Close vehicle object")
vehicle.close()
time.sleep(2)

print_with_flush("Exit")
sys.exit()
stop()


