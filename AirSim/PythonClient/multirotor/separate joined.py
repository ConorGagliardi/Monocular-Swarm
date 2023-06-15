import setup_path 
import airsim

import numpy as np
import pprint
import time
import traceback

"""
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",a
  "ClockSpeed": 1,

  "Vehicles": {
    "A": {
      "VehicleType": "SimpleFlight",
      "X": 1,
      "Y": -1,
      "Z": -1,
      "yaw": 180
    },
    "B": {
      "VehicleType": "SimpleFlight",
      "X": 3,
      "Y": -3,
      "Z": -2,
      "yaw": 180
    },
    "C": {
      "VehicleType": "SimpleFlight",
      "X": 5,
      "Y": -5,
      "Z": -3,
      "yaw": 180
    }
  }
}

"""


def enable_control(client, vehicle_names: list) -> None:
	for vehicle in vehicle_names:	
		client.enableApiControl(True, vehicle)
		client.armDisarm(True, vehicle)


def disable_control(client, vehicle_names: list) -> None:
	for vehicle in vehicle_names:                                                      
		client.armDisarm(False, vehicle)
		client.enableApiControl(False, vehicle)


def takeoff(client, vehicle_names: list) -> None:
	"""
	   Make all vehicles takeoff, one at a time and return the
	   pointer for the last vehicle takeoff to ensure we wait for
	   all drones  
	"""
	vehicle_pointers = []
	for vehicle in vehicle_names:
		vehicle_pointers.append(client.takeoffAsync(vehicle_name=vehicle))
	# All of this happens asynchronously. Hold the program until the last vehicle
	# finishes taking off.
	return vehicle_pointers[-1]

# ====================================================================================================== #
# Start of main process
# ====================================================================================================== #

# Generate a set of drones based upon a given number input and number of swarms.
# Convention: Capital Letter = Drone Swarm Number = Number of drone in that swarm
# Ex: A1, A2, A3, etc.
# Load list of parameters into the system -> Some sort of class module to set all of these for me.

# Load vehicle names as a list for easy iteration.
# TO DO: This will be drawn from the parameters file loading (Rules sheet)
vehicle_names = ["A", "B", "C"]
vehicle_offsets = {"A": [1, -1, 1], "B": [3, -3, 2], "C": [5, -5, 3]}
time_step = 5 # seconds
final_separation_distance = 10 # meters

# We want a matrix to track who can communicate with who!
# It should be a nxn matrix, with each drone tracking itself and the matrix looks like
#            drone_1 drone_2 ... drone n
# drone_1    true    false   ... true
# drone_2    false   true    ... true
# drone_n    false   false   ... true  
communications_tracker = np.zeros((len(vehicle_names),len(vehicle_names)), dtype=bool)

# We mimic the memory bank of a drone, tracking the relative positions.
# It should be a n-length vector, with each drone tracking itself and the matrix looks like

# drone_1 drone_2 ... drone n
# [x,y,z] [x,y,z] ... [x,y,z]

position_tracker = np.zeros((len(vehicle_names)), dtype=list)

try:
	client = airsim.MultirotorClient()
	print(client)
	client.confirmConnection()
	print('Connection Confirmed')

	enable_control(client, vehicle_names)

	airsim.wait_key('Press any key to takeoff')
	last_vehicle_pointer = takeoff(client, vehicle_names)
	# We wait until the last drone is off the ground
	last_vehicle_pointer.join()

	airsim.wait_key('Press any key to rendevous the drones!')
	
	vehicle_name = 'A'
	client.moveToPositionAsync(vehicle_offsets[vehicle_name][0] + 5, vehicle_offsets[vehicle_name][0] + 0, vehicle_offsets[vehicle_name][0] + 2, 2, vehicle_name=vehicle_name)
	time.sleep(0.1)
	vehicle_name = 'B'
	client.moveToPositionAsync(vehicle_offsets[vehicle_name][0] - 2, vehicle_offsets[vehicle_name][0] + 1, vehicle_offsets[vehicle_name][0] + 0, 2, vehicle_name=vehicle_name)
	time.sleep(0.1)
	vehicle_name = 'C'
	client.moveToPositionAsync(vehicle_offsets[vehicle_name][0] + 3, vehicle_offsets[vehicle_name][0] + 2, vehicle_offsets[vehicle_name][0] + 0, 2, vehicle_name=vehicle_name)
	# time.sleep(0.1)

	airsim.wait_key('Press any key to reset to original state')
	client.reset()
except Exception:
	traceback.print_exc()
finally:
	if client:
		client.reset()
		disable_control(client, vehicle_names)
	print("Finished!")