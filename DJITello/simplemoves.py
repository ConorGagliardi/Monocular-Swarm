def move_up(drone, distance):
    drone.move_up(distance * 100)  # Tello measures distance in cm

def move_down(drone, distance):
    drone.move_down(distance * 100)

def move_forward(drone, distance):
    drone.move_forward(distance * 100)  # Tello measures distance in cm

def move_backward(drone, distance):
    drone.move_back(distance * 100)  # Tello measures distance in cm

def move_right(drone, distance):
    drone.move_right(distance * 100)  # Tello measures distance in cm

def move_left(drone, distance):
    drone.move_left(distance * 100)  # Tello measures distance in cm
