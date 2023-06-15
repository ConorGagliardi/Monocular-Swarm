from djitellopy import Tello
from pynput import keyboard
from threading import Thread
from simplemoves import *
import time

class TelloDroneController:
    def __init__(self, drone):
        self.drone = drone
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
        while True:
            if self.breakout:  
                break
            if isinstance(self.current_key, keyboard.KeyCode):  # Check if the key is a regular character
                if self.current_key.char == 'w':
                    print("forward")
                    move_forward(self.drone, 1)
                elif self.current_key.char == 's':
                    print("backward")
                    move_backward(self.drone, 1)
            elif isinstance(self.current_key, keyboard.Key):  # Check if the key is a special key
                if self.current_key == keyboard.Key.up:
                    print("up")
                    move_up(self.drone, 1)
                elif self.current_key == keyboard.Key.down:
                    print("down")
                    move_down(self.drone, 1)
                elif self.current_key == keyboard.Key.left:
                    print("left")
                    move_left(self.drone, 1)
                elif self.current_key == keyboard.Key.right:
                    print("right")
                    move_right(self.drone, 1)
            time.sleep(0.1)

drone = Tello()
drone.connect()
drone.takeoff()

tello_controller = TelloDroneController(drone)
