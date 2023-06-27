import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
from pynput import keyboard
from threading import Thread
import time

import simplemoves as sm


# Use below in settings.json with Blocks environment
"""
{
	"SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 8, "Y": 0, "Z": -2
		}

    }
}
"""

# # connect to the AirSim simulator
# client = airsim.MultirotorClient()
# client.confirmConnection()
# client.enableApiControl(True, "Drone1")
# client.enableApiControl(True, "Drone2")
# client.armDisarm(True, "Drone1")
# client.armDisarm(True, "Drone2")

# airsim.wait_key('Press any key to takeoff')
# f1 = client.takeoffAsync(vehicle_name="Drone1")
# f2 = client.takeoffAsync(vehicle_name="Drone2")
# f1.join()
# f2.join()


# airsim.wait_key('Press any key to move drones')

# d1 = "Drone1"
# d2 = "Drone2"

# def f():
#     airsim.wait_key('f')

# sm.move_up(client, d1, 1)

# print("up")
# f()
# sm.move_up(client, d2, 2)


# f()
# print("up")
# sm.move_up(client, d1, 1)

# # f()
# # print("right")
# # sm.move_right(client, d2, 1)

# # f()
# # print("left")
# # sm.move_left(client, d2, 2)

# # f()
# # print("down")
# # sm.move_down(client, d2, 1)

# f()
# print("forward")
# sm.move_forward(client, d2, 1)

# f()
# print("backward")
# sm.move_backward(client, d2, 1)

class DroneController:
	def __init__(self, client, drone_name):
		self.client = client
		self.drone_name = drone_name
		self.current_key = None
		self.breakout = False  

		self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
		self.listener.start()

		self.movement_thread = Thread(target=self.movement_loop)
		self.movement_thread.start()

	def on_press(self, key):
		self.current_key = key
		if key == keyboard.Key.esc:  #PRESS ESCAPE TO END PROG
			self.breakout = True 

	def on_release(self, key):
		self.current_key = None

	def movement_loop(self):

		distance = 1

		while True:
			if self.breakout:  
				break
			if isinstance(self.current_key, keyboard.KeyCode):  # Check if the key is a regular character
				if self.current_key.char == 'w':
					print("forward")
					sm.move_forward(self.client, self.drone_name, distance)
				elif self.current_key.char == 's':
					print("backward")
					sm.move_backward(self.client, self.drone_name, distance)
				elif self.current_key.char == 'a':
					print("left turn")
					sm.rotate_left(self.client, self.drone_name)
				elif self.current_key.char == 'd':
					print("right turn")
					sm.rotate_right(self.client, self.drone_name)
			elif isinstance(self.current_key, keyboard.Key):  # Check if the key is a special key
				if self.current_key == keyboard.Key.up:
					print("up")
					sm.move_up(self.client, self.drone_name, distance)
				elif self.current_key == keyboard.Key.down:
					print("down")
					sm.move_down(self.client, self.drone_name, distance)
				elif self.current_key == keyboard.Key.left:
					print("left")
					sm.move_left(self.client, self.drone_name, distance)
				elif self.current_key == keyboard.Key.right:
					print("right")
					sm.move_right(self.client, self.drone_name, distance)

			
			time.sleep(0.1)

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
# client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.takeoffAsync(vehicle_name="Drone2")
# client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")

# drone1_controller = DroneController(client, "Drone1")
drone2_controller = DroneController(client, "Drone2")