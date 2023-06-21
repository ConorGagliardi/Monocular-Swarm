import setup_path 
from collections import deque
from imutils.video import VideoStream
import airsim
import cv2
import numpy as np
import argparse
import pprint
import imutils

import simplemoves as sm





ap = argparse.ArgumentParser()
ap.add_argument("-s", "--size", type=float, default=0.25,
                help="ratio of tracker box to camera size")
ap.add_argument("-p", "--position", type=float, nargs=2, default=[0.0, 0.0],
                help="X Y coords (0 , 0) is center. can be -1 thru 1")
ap.add_argument("-d", "--drone", type=str, default="Drone1",
                help="which drone to control")
ap.add_argument("-v", "--video",
                help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())


drone = args["drone"]

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, drone)
client.armDisarm(True, drone)

# set camera name and image type to request images and detections
camera_name = "0"
image_type = airsim.ImageType.Scene


while True:
    rawImage = client.simGetImage(camera_name, image_type, drone)
   
    if not rawImage:
        continue
    png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)

    

    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)

    pts = deque(maxlen=args["buffer"])

    if png is None:
        break

    
    blurred = cv2.GaussianBlur(png, (11, 11), 0)
    cv2.imshow("blurred", blurred)


    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    cv2.imshow("mask", mask)

    

    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    position = (0,0)
    
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        print(x,y)
        try:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        except:
            print("tried to divide by zero OOPS")
        print("center coords: ")
        print(center)

        print("Detected Radius: ")
        print(radius)

        if radius > 100:
            break

        position=center
        
        if radius > 0.5:
            cv2.circle(png, center, int(radius), (128, 0, 255), -1)

            cv2.circle(png, center, 2,
                       (0, 255, 255), 1)
            
            cv2.circle(png, center, 6,
                       (0, 255, 255), 1)
            
    #establish tracking boundary
    color = (255, 255, 255)  # color of the grid lines

    height, width, _ = png.shape  # Get image size

    center_box_scale = args["size"] #size of box relative to camera size
    center_box_position_offset = tuple(args["position"]) #X and Y of the tracking box center (-1 thru 1)

    center_box_size = (int(width * center_box_scale), int(height * center_box_scale))  
    center_box_position = (int((width - center_box_size[0]) * 0.5 * (1 + center_box_position_offset[0])), 
                       int((height - center_box_size[1]) * 0.5 * (1 + center_box_position_offset[1])))  # top left corner of the center box

    
    # draw vertical lines
    cv2.line(png, (center_box_position[0], 0), (center_box_position[0], height), color, 1)
    cv2.line(png, (center_box_position[0] + center_box_size[0], 0), (center_box_position[0] + center_box_size[0], height), color, 1)

    # draw horizontal lines
    cv2.line(png, (0, center_box_position[1]), (width, center_box_position[1]), color, 1)
    cv2.line(png, (0, center_box_position[1] + center_box_size[1]), (width, center_box_position[1] + center_box_size[1]), color, 1)


    left = center_box_position[0]
    right = center_box_position[0] + center_box_size[0]
    upper = center_box_position[1]
    lower = center_box_position[1] + center_box_size[1]

    status = ""
    
    line_color_left = (255, 255, 255)
    line_color_right = (255, 255, 255)
    line_color_upper = (255, 255, 255)
    line_color_lower = (255, 255, 255)

    if position:
        

        x, y = position
        if y < upper:
            status = 'Above'
            line_color_upper = (0, 0, 255)

            #attempt to remediate
            sm.move_up(client, drone, 1, 1)
            
        elif y > lower:
            status = 'Below'
            line_color_lower = (0, 0, 255)

            #attempt to remediate
            sm.move_down(client, drone, 1, 1)
            
        elif x < left:
            status = 'Left'
            line_color_left = (0, 0, 255)

            #attempt to remediate
            sm.move_left(client, drone, 1, 1)
            
        elif x > right:
            status = 'Right'
            line_color_right = (0, 0, 255) 

            #attempt to remediate
            sm.move_right(client, drone, 1, 1)

        elif radius < 2:
            status = 'too far'

            #attempt to remediate
            sm.move_forward(client, drone, 1, 1)

        elif radius > 5:
            status = 'too close'

            sm.move_backward(client, drone, 1, 1)

            
        else:
            status = 'Center'
    else:
        status = 'No Detection'

    # change line colors if it is beyond the threshold
    cv2.line(png, (left, 0), (left, height), line_color_left, 1)
    cv2.line(png, (right, 0), (right, height), line_color_right, 1)
    cv2.line(png, (0, upper), (width, upper), line_color_upper, 1)
    cv2.line(png, (0, lower), (width, lower), line_color_lower, 1)

    # display text of the position
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(png, status, (10, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            
    # pts.appendleft(center)

    # for i in range(1, len(pts)):
    #     if pts[i - 1] is None or pts[i] is None:
    #         continue
    #     thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
    #     cv2.line(png, pts[i - 1], pts[i], (0, 0, 255), thickness)

    scaler = 2.5
    width = int(png.shape[1] * scaler)
    height = int(png.shape[0] * scaler)
    resized = cv2.resize(png, (width, height), interpolation= cv2.INTER_AREA)
    cv2.imshow("AirSim", resized)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == ord('c'):
        client.simClearDetectionMeshNames(camera_name, image_type)
    elif cv2.waitKey(1) & 0xFF == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name, image_type, "Sphere*")


    
    


cv2.destroyAllWindows() 
