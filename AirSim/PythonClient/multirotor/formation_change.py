from collections import deque
from imutils.video import VideoStream
import airsim
import threading
import cv2
import numpy as np
import argparse
import time
import imutils
from pynput import keyboard

import simplemoves as sm


class QuadController:
    def __init__(self, args):
        self.args = args
        self.drone = args["drone"]

        self.client1 = airsim.MultirotorClient()
        # self.client1.confirmConnection()
        # self.client1.enableApiControl(True, self.drone)
        # self.client1.armDisarm(True, self.drone)

        self.client2 = airsim.MultirotorClient()
        self.client2.confirmConnection()
        self.client2.enableApiControl(True, self.drone)

        sm.move_up(self.client2, self.drone, 1, 1)

        self.client2.armDisarm(True, self.drone)

        self.camera_name = "0"
        self.image_type = airsim.ImageType.Scene

        self.pts = deque(maxlen=self.args["buffer"])
        self.bounding_box_xy = self.args["position"]

        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)

        self.gridLines = (255, 255, 255)
        self.gridTriggeredLines = (0, 0, 255)

        self.line_color_left = (255, 255, 255)
        self.line_color_right = (255, 255, 255)
        self.line_color_upper = (255, 255, 255)
        self.line_color_lower = (255, 255, 255)

        self.status = ""

        self.formation = "TRIANGLE"
        self.breakout = False
        self.current_key = None

        self.radius = 0
        self.min_radius = 7
        self.max_radius = 12
        self.cent_position = (0,0)
        self.prj_position = (0,0)

        self.upper = 0
        self.right = 0
        self.left = 0
        self.lower = 0

        self.running = True

    def start_video(self):
        self.video_thread = threading.Thread(target=self.img_tracker, args=(self.client1,))
        self.video_thread.start()

    def start_movements(self):
        self.command_thread = threading.Thread(target=self.detect_movements)
        self.command_thread.start()
        
        self.order_thread = threading.Thread(target=self.send_movements, args=(self.client2,))
        self.order_thread.start()

    def start_listener(self):
        self.key_listener = threading.Thread(target=self.key_listener)
        self.key_listener.start()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        self.current_key = key
        if key == keyboard.Key.esc:  #PRESS ESCAPE TO EXIT KEY RECORDER
            self.breakout = True
            self.running = False

    def on_release(self, key):
        self.current_key = None

    def key_listener(self):
        while self.running:
            if self.breakout: 
                self.stop()
                break
            if isinstance(self.current_key, keyboard.KeyCode):
                if self.current_key.char == 'l':
                    print("LINE FORMATION ACTIVATE")
                    self.formation = "LINE"
                    time.sleep(5.0)
                    
                elif self.current_key.char == 't':
                    print("TRIANGLE FORMATION ACTIVATE")
                    self.formation = "TRIANGLE"
                    time.sleep(5.0)

            time.sleep(0.1)


    def img_tracker(self, client):
        while self.running:

            if self.breakout:
                break

            rawImage = client.simGetImage(self.camera_name, self.image_type, self.drone)


            if not rawImage:
                time.sleep(0.1)
                continue

            png_raw = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)

            scaler = 1
            width = int(png_raw.shape[1] * scaler)
            height = int(png_raw.shape[0] * scaler)
            png = cv2.resize(png_raw, (width, height), interpolation= cv2.INTER_AREA)
            

            blurred = cv2.GaussianBlur(png, (11, 11), 0)

            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)

                #print(x,y)

                try:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                except:
                    print("tried to divide by zero OOPS")
                    center = (png.shape[1] // 2, png.shape[0] // 2)

                #print("center coords: ")
                #print(center)

                #print("Detected Radius: ")
                #print(radius)

                if radius > 3:
                    cv2.circle(png, center, int(radius), (128, 0, 255), -1)

                    cv2.circle(png, center, self.min_radius, (0, 255, 255), 2)
                    cv2.circle(png, center, self.max_radius, (0, 255, 255), 2)

                self.cent_position = center
                self.pts.appendleft(center)
                self.radius = radius

            past_points = np.array(self.pts)

            skip = 3
            scalar = 8 / radius

            if len(past_points) > skip:

                vec_list = np.diff(past_points[::skip], axis = 0) * scalar
                    
                avg_velocity = np.rint(np.nanmean(vec_list, axis = 0))


                # print("AVERGAGE VEOLOCITY")
                # print(avg_velocity)

                try:

                    pt_2 = (int(center[0]-avg_velocity[0]), int(center[1]-avg_velocity[1]))

                    self.prj_position = pt_2

                    cv2.arrowedLine(png, center, pt_2, (255, 255, 25), 2)
                except:
                    print("found NaN")
                

            #establish tracking boundary
            color = (255, 255, 255)  # color of the grid lines


            height, width, _ = png.shape  # Get image size

            center_box_scale = self.args["size"] # size of box relative to camera size
            center_box_position_offset = tuple(self.bounding_box_xy) # X and Y of the tracking box center (-1 thru 1)

            center_box_size = (int(width * center_box_scale), int(height * center_box_scale))  
            center_box_position = (int((width - center_box_size[0]) * 0.5 * (1 + center_box_position_offset[0])), 
                            int((height - center_box_size[1]) * 0.5 * (1 + center_box_position_offset[1])))  # top left corner of the center box

            # vertical lines
            cv2.line(png, (center_box_position[0], 0), (center_box_position[0], height), color, 1)
            cv2.line(png, (center_box_position[0] + center_box_size[0], 0), (center_box_position[0] + center_box_size[0], height), color, 1)

            # horizon
            cv2.line(png, (0, center_box_position[1]), (width, center_box_position[1]), color, 1)
            cv2.line(png, (0, center_box_position[1] + center_box_size[1]), (width, center_box_position[1] + center_box_size[1]), color, 1)


            self.left = center_box_position[0]
            self.right = center_box_position[0] + center_box_size[0]
            self.upper = center_box_position[1]
            self.lower = center_box_position[1] + center_box_size[1]

            # change line colors if it is beyond the threshold
            cv2.line(png, (self.left, 0), (self.left, height), self.line_color_left, 1)
            cv2.line(png, (self.right, 0), (self.right, height), self.line_color_right, 1)
            cv2.line(png, (0, self.upper), (width, self.upper), self.line_color_upper, 1)
            cv2.line(png, (0, self.lower), (width, self.lower), self.line_color_lower, 1)


            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(png, self.status, (10, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            scaler = 1
            width = int(png.shape[1] * scaler)
            height = int(png.shape[0] * scaler)
            resized = cv2.resize(png, (width, height), interpolation= cv2.INTER_AREA)
            cv2.imshow("AirSim", resized)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
            
            
    def change_form(self, client):
        #hardcoding formation change as proof of concept

        #agent 1 back left triangle drone
        if self.args["agent"] == 1:

            #if changing to line
            if self.formation == "LINE":
                self.bounding_box_xy = (0,0)
                sm.move_right(client, self.drone, 1.5, 1.5)
                time.sleep(1)

            if self.formation == "TRIANGLE":
                self.bounding_box_xy = self.args["position"]
                sm.move_left(client, self.drone, 1.5, 1.5)
                time.sleep(1)


        #agent 2 back right triangle drone
        if self.args["agent"] == 2:

            #if changing to line
            if self.formation == "LINE":
                self.bounding_box_xy = (0, 0)
                sm.move_backward(client, self.drone, 3, 3)
                time.sleep(1)
                sm.move_left(client, self.drone, 1.5, 1.5)

            if self.formation == "TRIANGLE":
                self.bounding_box_xy = self.args["position"]
                sm.move_right(client, self.drone, 1.5, 1.5)
                time.sleep(1)
                sm.move_forward(client, self.drone, 3.5, 3.5)


    #meant to only change the line colors for visual clarity
    def detect_movements(self):

        while self.running:
            
            if self.breakout:
                break
            
            if self.cent_position is not None:
                x, y = self.cent_position
                if self.prj_position is not None:
                    #print('Using projected position!')
                    x, y = self.prj_position
                if x != 0 and y != 0:

                    if self.radius < self.min_radius:
                        self.status = 'too far'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)

                    elif self.radius > self.max_radius:
                        self.status = 'too close'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)

                    elif y < self.upper:
                        self.status = 'Moving Up'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (0, 0, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)

                        
                    elif y > self.lower:
                        self.status = 'Moving Down'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (0, 0, 255)
                        time.sleep(0.1)

                        
                    elif x < self.left:
                        self.status = 'Moving Left'
                        self.line_color_left = (0, 0, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)

                        
                    elif x > self.right:
                        self.status = 'Moving Right'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (0, 0, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)

                        
                    else:
                        self.status = 'Center'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)
                        time.sleep(0.1)
                else:
                    self.status = 'No Detection'
                    self.line_color_left = (255, 255, 255)
                    self.line_color_right = (255, 255, 255)
                    self.line_color_upper = (255, 255, 255)
                    self.line_color_lower = (255, 255, 255)
                    time.sleep(0.1)

    def send_movements(self, client):
        speed = 1
        distance = 1

        curr_form = "TRIANGLE"


        while self.running:

            if self.breakout:
                break

            if self.cent_position is not None:
                if self.status is not None:

                    if self.status == 'Moving Up':
                        print('up')

                        #attempt to remediate
                        sm.move_up(client, self.drone, distance, speed)
                        
                    elif self.status == 'Moving Down':
                        print('down')

                        #attempt to remediate
                        sm.move_down(client, self.drone, distance, speed)
                        
                    elif self.status == 'Moving Left':
                        print('left')

                        #attempt to remediate
                        sm.move_left(client, self.drone, distance, speed)
                        
                    elif self.status == 'Moving Right':
                        print('right')

                        #attempt to remediate
                        sm.move_right(client, self.drone, 1, speed)
                    
                    elif self.status == 'too far':
                        print('forward')

                        #attempt to remediate
                        sm.move_forward(client, self.drone, distance, speed)

                    elif self.status == 'too close':
                        print('reverse!')

                        #attempt to remediate
                        sm.move_backward(client, self.drone, distance, speed)


                        
                    else:
                        
                        if curr_form != self.formation and self.status == 'Center':
                            curr_form = self.formation
                            self.change_form(client)
                        time.sleep(0.1)
                else:
                    time.sleep(0.1)

        

    def stop(self):
        self.running = False
        if threading.current_thread() != self.command_thread:
            self.command_thread.join()
        if threading.current_thread() != self.order_thread:
            self.order_thread.join()
        if threading.current_thread() != self.video_thread:
            self.video_thread.join()
        if threading.current_thread() != self.listener:
            self.listener.join()
        if threading.current_thread() != self.key_listener:
            self.key_listener.join()
        cv2.destroyAllWindows()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--size", type=float, default=0.25,
                    help="ratio of tracker box to camera size")
    ap.add_argument("-p", "--position", type=float, nargs=2, default=[0.0, 0.0],
                    help="X Y coords (0 , 0) is center. can be -1 thru 1")
    ap.add_argument("-d", "--drone", type=str, default="Drone1",
                    help="which drone to control")
    ap.add_argument("-b", "--buffer", type=int, default=20,
                    help="max buffer size")
    ap.add_argument("-a", "--agent", type=int, default=1,
                    help="agent number for position in formations")
    args = vars(ap.parse_args())

    quad = QuadController(args)

    quad.start_video()

    quad.start_movements()

    quad.start_listener()



    #quad.stop()


if __name__ == '__main__':
    main()