from collections import deque
from djitellopy import Tello
import threading
import cv2
import numpy as np
import argparse
import time
import imutils
from pynput import keyboard
from ultralytics import YOLO

import simplemoves as sm


class QuadController:
    def __init__(self, args):
        self.args = args

        drone = Tello()
        drone.connect()


        drone.streamon()

        self.drone = drone

        self.pts = deque(maxlen=self.args["buffer"])
        self.bounding_box_xy = self.args["position"]

        self.detector = YOLO('drone_detector_v2_finetuned.pt')

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
        self.two_blobs = False

        self.radius = 0
        self.min_radius = 40
        self.max_radius = 70
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

    def img_tracker(self):
        while self.running:

            if self.breakout:
                break


            frame_read = self.drone.get_frame_read()

            if not frame_read:
                time.sleep(0.1)
                continue

            png_raw = frame_read.frame

            png_raw = cv2.cvtColor(png_raw, cv2.COLOR_RGB2BGR)
            
            width = int(png_raw.shape[1] * 0.7)
            height = int(png_raw.shape[0] * 0.7)
            png = cv2.resize(png_raw, (width, height), interpolation=cv2.INTER_AREA)
            

            center = None

            top = None
            left = None 
            right = None
            bot = None

            mid_vertical = None
            mid_horizontal = None


            results = self.detector.track(png, persist=True)

            annotated_frame = results[0].plot()

            cv2.imshow("YOLOv8 Inference", annotated_frame)

            if results[0].boxes is not None and results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
                ids = results[0].boxes.id.cpu().numpy().astype(int)

                found_first = False
                for box, id in zip(boxes, ids):
                    if not found_first:
                        top = box[0]
                        left = box[1]
                        right = box[3]
                        bot = box[2]
                    
                        mid_horizontal = (box[0] + box[2]) // 2
                        mid_vertical = (box[1] + box[3]) // 2
                        center = (mid_horizontal , mid_vertical)

                        found_first = True

                    cv2.rectangle(png, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
                    cv2.putText(
                        png,
                        f"Id {id}",
                        (box[0], box[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2,
                    )


            


                radius = ((right - mid_horizontal) + (bot - mid_vertical)) // 2

                print(radius)


                if radius > 20:
                        cv2.circle(png, center, int(radius / 3), (128, 0, 255), -1)

                        cv2.circle(png, center, int(self.min_radius / 3), (0, 255, 255), 2)
                        cv2.circle(png, center, int(self.max_radius / 3), (0, 255, 255), 2)

                

                    #print("center coords: ")
                    #print(center)

                    #print("Detected Radius: ")
                    #print(radius)


                self.cent_position = center
                self.pts.appendleft(center)
                self.radius = radius

                past_points = np.array(self.pts)

                skip = 3
                scalar = 2
                #scalar = (radius + 10) / radius

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


    def change_form(self):
        #hardcoding formation change as proof of concept

        #agent 1 back left triangle drone
        if self.args["agent"] == 1:

            #if changing to line
            if self.formation == "LINE":
                self.bounding_box_xy = (0,0)
                sm.move_right(self.drone, 2)
                time.sleep(1)

            if self.formation == "TRIANGLE":
                self.bounding_box_xy = self.args["position"]
                sm.move_left(self.drone, 2)
                time.sleep(1)


        #agent 2 back right triangle drone
        if self.args["agent"] == 2:

            #if changing to line
            if self.formation == "LINE":
                self.bounding_box_xy = (0, 0)
                sm.move_backward(self.drone, 4)
                time.sleep(2)
                if self.two_blobs:
                    sm.move_left(self.drone, 2)

            if self.formation == "TRIANGLE":
                self.bounding_box_xy = self.args["position"]
                sm.move_right(self.drone, 2)
                time.sleep(1)
                sm.move_forward(self.drone, 4)
            
            
    def detect_movements(self):

        while self.running:
            
            if self.breakout:
                break

            if self.position is not None:
                x, y = self.position
                if self.prj_position is not None:
                    print('Using projected position!')
                    x, y = self.prj_position
                if x != 0 and y != 0:

                    

                    if y < self.upper:
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
                    
                    elif self.radius < self.min_radius:
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

        curr_form = "TRIANGLE"


        while self.running:

            if self.breakout:
                break

            if self.position is not None:
                if self.status is not None:

                    if self.status == 'Moving Up':
                        print('up')
                        sm.move_up(self.drone, distance)
                        time.sleep(0.1)
                        
                    elif self.status == 'Moving Down':
                        print('down')
                        sm.move_down(self.drone, distance)
                        time.sleep(0.1)
                        
                    elif self.status == 'Moving Left':
                        print('left')
                        sm.move_left(self.drone, distance)
                        time.sleep(0.1)
                        
                    elif self.status == 'Moving Right':
                        print('right')
                        sm.move_right(self.drone, distance)
                        time.sleep(0.1)
                    
                    elif self.status == 'too far':
                        print('forward')
                        sm.move_forward(self.drone, distance)
                        time.sleep(0.1)

                    elif self.status == 'too close':
                        print('reverse!')
                        sm.move_backward(self.drone, distance)
                        time.sleep(0.1)


                        
                    else:

                        if curr_form != self.formation and self.status == 'Center':
                            curr_form = self.formation
                            self.change_form()
                        
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
    ap.add_argument("-b", "--buffer", type=int, default=30,
                    help="max buffer size")
    ap.add_argument("-a", "--agent", type=int, default=1,
                    help="agent number for position in formations")
    args = vars(ap.parse_args())

    quad = QuadController(args)

    quad.start_movements()

    quad.start_listener()

    quad.img_tracker()

    quad.drone.land()

    quad.stop()


if __name__ == '__main__':
    main()