## change height to fit line in middle frame first, once height is calibrated move onto regular loop
## try to change turns based on percentages instead of arbitrary.

import numpy as np
from djitellopy import tello
import cv2
import time



me = tello.Tello()

me.connect()

print(me.get_battery())

me.streamon()

time.sleep(2)

me.takeoff()

me.move_up(2 * 20)

# cap = cv2.VideoCapture(0)

hsvVals = [0, 60, 113, 129, 255, 223]
sensors = 3
threshold = 0.3
width, height = 480, 360
senstivity = 3  # if number is high less sensitive
weights = [-25, -10, 0, 10, 25]
fSpeed = 8
curve = 0

def thresholding(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([hsvVals[0], hsvVals[1], hsvVals[2]])
    upper = np.array([hsvVals[3], hsvVals[4], hsvVals[5]])
    mask = cv2.inRange(hsv, lower, upper)
    #mask = cv2.bitwise_not(mask)

    return mask

def getContours(imgThres, img):
    cx = 0
    cy = 0
    l1 = 0
    l2 = 0
    h = 0

    contours, hieracrhy = cv2.findContours(imgThres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        biggest = max(contours, key=cv2.contourArea)

        x, y, w, h = cv2.boundingRect(biggest)

        cx = x + w // 2
        cy = y + h // 2

        cv2.drawContours(img, biggest, -1, (255, 0, 255), 7)

        cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

        l1 = x
        l2 = x + w


    return cx, cy, l1, l2, h

def getSensorOutput(imgThres, sensors):

    imgs = np.hsplit(imgThres, sensors)

    totalPixels = (imgThres.shape[1] // sensors) * imgThres.shape[0]

    senOut = []

    for x, im in enumerate(imgs):
        pixelCount = cv2.countNonZero(im)
        if pixelCount > threshold * totalPixels:
            senOut.append(1)
        else:
            senOut.append(0)

        # cv2.imshow(str(x), im)

    # print(senOut)
    return senOut

def sendCommands(senOut, cx):

    global curve
    global previous_moves 


    ## TRANSLATION
    lr = (cx - width // 2) // senstivity
    lr = int(np.clip(lr, -10, 10))
    if 2 > lr > -2: lr = 0

    ## Rotation
    if   senOut == [1, 0, 0]: curve = weights[0]
    elif senOut == [1, 1, 0]: curve = weights[1]
    elif senOut == [0, 1, 0]: curve = weights[2]
    elif senOut == [0, 1, 1]: curve = weights[3]
    elif senOut == [0, 0, 1]: curve = weights[4]
    elif senOut == [0, 0, 0]: curve = weights[2]
    elif senOut == [1, 1, 1]: curve = weights[2]
    elif senOut == [1, 0, 1]: curve = weights[2]
    me.send_rc_control(lr, fSpeed, 0, curve)


def calibrate():
    time_buffer = 0
    calibrated = False

    while not calibrated:
        # capture frame and process image
        img = me.get_frame_read().frame
        img = cv2.resize(img, (width, height))
        img = cv2.flip(img, 0)

        imgThres = thresholding(img)

        cx, cy, l1, l2, h = getContours(imgThres, img)


        buffer_val = 20
        outer_size = 60

        boundary = (imgThres.shape[1] // sensors) - outer_size

        boundary2 = imgThres.shape[1] - (imgThres.shape[1] // sensors) + outer_size


        calc_width = l2 - l1

        max_need_width = boundary2 - boundary

        min_need_width = (boundary2 - buffer_val) - (boundary + buffer_val)

        cv2.line(img, (boundary, 0), (boundary, h), (0, 255, 255), 1) #guidelines left
        cv2.line(img, (boundary + buffer_val, 0), (boundary + buffer_val, h), (0, 255, 255), 1)

        cv2.line(img, (boundary2 - buffer_val, 0), (boundary2 - buffer_val, h), (0, 255, 255), 1) #guidelines right
        cv2.line(img, (boundary2, 0), (boundary2, h), (0, 255, 255), 1)


        cv2.line(img, (l1, 0), (l1, h), (0, 0, 255), 2) #detected lines

        cv2.line(img, (l2, 0), (l2, h), (0, 0, 255), 2)

        cv2.imshow("Output", img)
        cv2.imshow("threshold", imgThres)



        if calc_width > max_need_width:
            print('move up')
            me.send_rc_control(0, 0, 10, 0) 
            time.sleep(0.1) # move up
            time_buffer = 0

        elif calc_width < min_need_width:
            print('move down')
            me.send_rc_control(0, 0, -10, 0)  # move down
            time_buffer = 0
            time.sleep(0.1)

        elif calc_width > min_need_width and calc_width < max_need_width:
            print('good height now lets go!')
            time_buffer += 1



        # if line has been in center of frame and the right size for 30 frames, we're calibrated
        print(time_buffer)
        if time_buffer >= 200:
            calibrated = True
            break

        cv2.waitKey(1)




calibrate()

print('we calibrated')

while True:

    img = me.get_frame_read().frame
    img = cv2.resize(img, (width, height))
    img = cv2.flip(img, 0)
    imgThres = thresholding(img)
    cx, cy, l1, l2, h = getContours(imgThres, img)  ## For Translation
    senOut = getSensorOutput(imgThres, sensors)  ## Rotation
    sendCommands(senOut, cx)
    cv2.imshow("Output", img)
    cv2.imshow("Path", imgThres)
    cv2.waitKey(1)
