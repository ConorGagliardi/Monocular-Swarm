from collections import deque
from djitellopy import Tello
import threading
import cv2
import numpy as np
import argparse
import time
import imutils

import simplemoves as sm


class QuadController:
    def __init__(self, args):
        self.args = args

        drone = Tello()
        drone.connect()


        drone.streamon()

        self.drone = drone

        self.pts = deque(maxlen=self.args["buffer"])

        self.greenLower = (29, 86, 6)
        self.greenUpper = (64, 255, 255)

        self.gridLines = (255, 255, 255)
        self.gridTriggeredLines = (0, 0, 255)

        self.line_color_left = (255, 255, 255)
        self.line_color_right = (255, 255, 255)
        self.line_color_upper = (255, 255, 255)
        self.line_color_lower = (255, 255, 255)

        self.status = ""

        self.radius = 0
        self.min_radius = 12
        self.max_radius = 30
        self.position = (0,0)
        self.prj_position = (0,0)

        self.upper = 0
        self.right = 0
        self.left = 0
        self.lower = 0

        self.running = True

        self.drone.takeoff()

        

    def start_video(self):
        self.video_thread = threading.Thread(target=self.img_tracker, args=(self.drone,))
        self.video_thread.start()
        # pass

    def start_movements(self):
        self.command_thread = threading.Thread(target=self.detect_movements)
        self.command_thread.start()

        self.order_thread = threading.Thread(target=self.send_movements)
        self.order_thread.start()

    def img_tracker(self):
        while self.running:
            frame_read = self.drone.get_frame_read()

            if not frame_read:
                time.sleep(0.1)
                continue

            raw_png = frame_read.frame


            width = int(raw_png.shape[1] * 0.7)
            height = int(raw_png.shape[0] * 0.7)
            png = cv2.resize(raw_png, (width, height), interpolation= cv2.INTER_AREA)
            

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

                print(x,y)

                try:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                except:
                    print("tried to divide by zero OOPS")
                    center = (png.shape[1] // 2, png.shape[0] // 2)

                # print("center coords: ")
                # print(center)

                # print("Detected Radius: ")
                # print(radius)

                if radius > 5:
                    cv2.circle(png, center, int(radius), (128, 0, 255), -1)

                    cv2.circle(png, center, self.min_radius, (0, 255, 255), 2)
                    cv2.circle(png, center, self.max_radius, (0, 255, 255), 2)

                self.position = center
                self.pts.appendleft(center)
                self.radius = radius

            past_points = np.array(self.pts)

            skip = 3
            scalar = 3

            if len(past_points) > skip:

                vec_list = np.diff(past_points[::skip], axis = 0) * scalar
                    
                avg_velocity = np.rint(np.nanmean(vec_list, axis = 0))


                # print("AVERGAGE VEOLOCITY")
                # print(avg_velocity)

                try:

                    pt_2 = (int(center[0]-avg_velocity[0]), int(center[1]-avg_velocity[1]))

                    # self.prj_position = pt_2

                    cv2.arrowedLine(png, center, pt_2, (255, 255, 25), 2)
                except:
                    print("found NaN")
                
            if len(past_points) > skip:
                vec_list = np.diff(past_points[::skip], axis = 0) * scalar
                            
                # weight vector with highest weight first
                weights = np.flip(np.arange(len(vec_list)) + 1)

                #print(weights)

                avg_velocity = np.rint(np.average(vec_list, axis = 0, weights=weights))

                # print("AVERGAGE VEOLOCITY")
                #print(avg_velocity)

                try:
                    pt_2 = (int(center[0]-avg_velocity[0]), int(center[1]-avg_velocity[1]))

                    self.prj_position = pt_2

                    cv2.arrowedLine(png, center, pt_2, (25, 255, 25), 2)
                except:
                    print("found NaN")



            #establish tracking boundary
            color = (255, 255, 255)  # color of the grid lines


            height, width, _ = png.shape  # Get image size

            center_box_scale = self.args["size"] # size of box relative to camera size
            center_box_position_offset = tuple(self.args["position"]) # X and Y of the tracking box center (-1 thru 1)

            center_box_size = (int(width * center_box_scale), int(height * center_box_scale))  
            center_box_position = (int((width - center_box_size[0]) * 0.5 * (1 + center_box_position_offset[0])), 
                            int((height - center_box_size[1]) * 0.5 * (1 + center_box_position_offset[1])))  # top left corner

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
            
            
    def detect_movements(self):

        while self.running:
            
            

            if self.position is not None:
                x, y = self.position
                if self.prj_position is not None:
                    print('Using projected position!')
                    x, y = self.prj_position
                if x != 0 and y != 0:

                    if self.radius < self.min_radius:
                        self.status = 'too far'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)

                    elif self.radius > self.max_radius:
                        self.status = 'too close'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)

                    elif y < self.upper:
                        self.status = 'Moving Up'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (0, 0, 255)
                        self.line_color_lower = (255, 255, 255)

                        
                    elif y > self.lower:
                        self.status = 'Moving Down'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (0, 0, 255)

                        
                    elif x < self.left:
                        self.status = 'Moving Left'
                        self.line_color_left = (0, 0, 255)
                        self.line_color_right = (255, 255, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)

                        
                    elif x > self.right:
                        self.status = 'Moving Right'
                        self.line_color_left = (255, 255, 255)
                        self.line_color_right = (0, 0, 255)
                        self.line_color_upper = (255, 255, 255)
                        self.line_color_lower = (255, 255, 255)

                        
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

        
    def send_movements(self):
        distance = 1
        while self.running:
            if self.position is not None:
                if self.status is not None:

                    if self.status == 'too far':
                        print('forward')
                        sm.move_forward(self.drone, distance)

                    elif self.status == 'too close':
                        print('reverse!')
                        sm.move_backward(self.drone, distance)

                    elif self.status == 'Moving Up':
                        print('up')
                        sm.move_up(self.drone, distance)
                        
                    elif self.status == 'Moving Down':
                        print('down')
                        sm.move_down(self.drone, distance)
                        
                    elif self.status == 'Moving Left':
                        print('left')
                        sm.move_left(self.drone, distance)
                        
                    elif self.status == 'Moving Right':
                        print('right')
                        sm.move_right(self.drone, 1)


                        
                    else:
                        
                        time.sleep(0.1)
                else:
                    
                    time.sleep(0.1)

    def stop(self):
        self.running = False
        self.drone.land()
        if threading.current_thread() != self.command_thread:
            self.command_thread.join()
        if threading.current_thread() != self.order_thread:
            self.order_thread.join()
        # if threading.current_thread() != self.video_thread:
        #     self.video_thread.join()
        cv2.destroyAllWindows()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--size", type=float, default=0.25,
                    help="ratio of tracker box to camera size")
    ap.add_argument("-p", "--position", type=float, nargs=2, default=[0.0, 0.0],
                    help="X Y coords (0 , 0) is center. can be -1 thru 1")
    ap.add_argument("-d", "--drone", type=str, default="Drone1",
                    help="which drone to control")
    ap.add_argument("-b", "--buffer", type=int, default=30,
                    help="max buffer size")
    args = vars(ap.parse_args())

    quad = QuadController(args)

    quad.start_movements()

    quad.img_tracker()



    #quad.stop()


if __name__ == '__main__':
    main()
