#EX) move_up(client, "Drone1", 1, 5)  # Move Drone1 up by 1 meter at 5 m/s
import airsim
import cv2
import numpy as np
import os
import pprint
import setup_path 
import tempfile
import time

def move_up(client, drone_name, distance, speed=1):
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    current_pos = drone_state.kinematics_estimated.position
    client.moveToPositionAsync(current_pos.x_val, current_pos.y_val, current_pos.z_val - distance, speed, vehicle_name=drone_name).join()

# def move_down(client, drone_name, distance, speed=1):
#     drone_state = client.getMultirotorState(vehicle_name=drone_name)
#     current_pos = drone_state.kinematics_estimated.position
#     client.moveToPositionAsync(current_pos.x_val, current_pos.y_val, current_pos.z_val + distance, speed, vehicle_name=drone_name).join()

def move_down(client, drone_name, distance, speed=1):
    #USE LANDING FOR DOWN MOVEMENT INSTEAD OF MANUAL CONTROL BECAUSE Thats just how
    #it has to be :)
    landing = client.landAsync(vehicle_name=drone_name)
    time.sleep(distance / speed)
    client.hoverAsync(vehicle_name=drone_name)
    landing.join()

def move_forward(client, drone_name, distance, speed=1):
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    current_pos = drone_state.kinematics_estimated.position
    client.moveToPositionAsync(current_pos.x_val + distance, current_pos.y_val, current_pos.z_val, speed, vehicle_name=drone_name).join()

def move_backward(client, drone_name, distance, speed=1):
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    current_pos = drone_state.kinematics_estimated.position
    client.moveToPositionAsync(current_pos.x_val - distance, current_pos.y_val, current_pos.z_val, speed, vehicle_name=drone_name).join()

def move_right(client, drone_name, distance, speed=1):
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    current_pos = drone_state.kinematics_estimated.position
    client.moveToPositionAsync(current_pos.x_val, current_pos.y_val + distance, current_pos.z_val, speed, vehicle_name=drone_name).join()

def move_left(client, drone_name, distance, speed=1):
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    current_pos = drone_state.kinematics_estimated.position
    client.moveToPositionAsync(current_pos.x_val, current_pos.y_val - distance, current_pos.z_val, speed, vehicle_name=drone_name).join()
