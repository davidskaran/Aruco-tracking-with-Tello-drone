from djitellopy import Tello
from arucoPoseEstimation import *
from PID import *
import cv2 as cv
import time

myDrone = Tello()
myDrone.connect()


def initializeTello():
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone


# Extracting frame by frame from tello drone
def telloGetFrame(myDrone, w=360, h=240):
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame
    img = cv.resize(myFrame, (w, h))
    return img


# For manually controlling the Tello drone
def manual_key_control():  # funksjonen kjøres i main
    key = cv.waitKey(1) & 0xFF
    if key == ord("w"):
        myDrone.move_forward(30)
    elif key == ord("s"):
        myDrone.move_back(30)
    elif key == ord("a"):
        myDrone.move_left(30)
    elif key == ord("d"):
        myDrone.move_right(30)
    elif key == ord("x"):
        myDrone.rotate_clockwise(30)
    elif key == ord("z"):
        myDrone.rotate_counter_clockwise(30)
    elif key == ord("r"):
        myDrone.move_up(30)
    elif key == ord("f"):
        myDrone.move_down(30)
    elif key == ord("t"):
        myDrone.takeoff()
        tIsPressed = True
