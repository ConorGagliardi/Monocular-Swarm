# Monocular-Swarm

Overview: Uses microsoft airsim for testing monocular swarming techniques. Then attempting to use code abstraction to implement the same algorithms for
both microsoft airsim as well as physical DJI Tello EDU drones.

## Simulation:
So far primary edits are here: AirSim/PythonClient/multirotor

Tracking program "testball.py"
and controller "drone_controller.py" and "simplemoves.py"

## DJI Tello Implementation:
Mostly unchanged testball.py will be used for tello, with a modified drone_controller that
is able to control a dji tello edu drone.
