import setup_path
import airsim
import pprint
import time

client = airsim.MultirotorClient()
def connect(client, vehicle_name):
    #client = airsim.MultirotorClient()
    client.simPause(True)
    client.confirmConnection()
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)
    client.takeoffAsync(vehicle_name=vehicle_name)

    position = airsim.Vector3r(0 , 0, -10)
    heading = airsim.utils.to_quaternion(0, 0, 0)
    pose = airsim.Pose(position, heading)
    client.simSetVehiclePose(pose, False, vehicle_name=vehicle_name)

    client.simPause(False)
    client.moveByVelocityZAsync(0, 0, -10, 1, vehicle_name=vehicle_name).join()

#Connect the drones
connect(client,'Drone1')

#Set the waypoint list

waypoints_drone1 = [[0,-2,-10],[0,0,-10],[0,-2,-10], [0,0,-10]]

#Fly the drones for i in range (4): client.moveToPositionAsync(waypoints_drone1[i][0],waypoints_drone1[i][1],waypoints_drone1[i][2],1,vehicle_name='Drone1').join()

state1 = client.simGetGroundTruthKinematics(vehicle_name="Drone1").position
s = pprint.pformat(state1)
print("state: %s" % s)
client.enableApiControl(False, "Drone1")