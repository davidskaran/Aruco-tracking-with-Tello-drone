from utils import *
import numpy as np


# Kp, ki, ke
pid_x_dir = [15, 1, 0]
pid_y_dir = [15, 0, 0]
pid_z_dir = [8, 3, 0]
# For when you want to use angle rotation, take the rotation from one single axis for either x or y, and keep the aruco code in a fixed direction

previousError_x_dir = 0  # saved from previous error registered.
previousError_y_dir = 0
previousError_z_dir = 0

# This code takes reading from a sensor, like distance, and returns the error value
def PID_reg_x_dir(myDrone, sensorReading, pid, previousError, reference):

    error = (
        sensorReading - reference
    )  # zero means in this example that the reference is when x y or z is zero

    speed = pid[0] * error + pid[1] * (error - previousError)

    speed = int(np.clip(speed, -100, 100))

    # yaw_velocity rotates with the z axis
    myDrone.yaw_velocity = speed
    myDrone.send_rc_control(
        myDrone.left_right_velocity,
        myDrone.for_back_velocity,
        myDrone.up_down_velocity,
        myDrone.yaw_velocity,
    )
    return error


def PID_reg_y_dir(myDrone, sensorReading, pid, previousError, reference):

    error = -(
        sensorReading - reference
    )  # zero means in this example that the reference is when x y or z is zero
    speed = pid[0] * error + pid[1] * (error - previousError)

    speed = int(np.clip(speed, -100, 100))

    # yaw_velocity rotates with the z axis
    myDrone.up_down_velocity = speed
    myDrone.send_rc_control(
        myDrone.left_right_velocity,
        myDrone.for_back_velocity,
        myDrone.up_down_velocity,
        myDrone.yaw_velocity,
    )
    return error


def PID_reg_z_dir(myDrone, sensorReading, pid, previousError, reference):

    error = (
        sensorReading - reference
    )  # zero means in this example that the reference is when x y or z is zero
    speed = pid[0] * error + pid[1] * (error - previousError)

    speed = int(np.clip(speed, -100, 100))

    if sensorReading == 0:
        speed = 0

    # yaw_velocity rotates with the z axis
    myDrone.for_back_velocity = speed
    myDrone.send_rc_control(
        myDrone.left_right_velocity,
        myDrone.for_back_velocity,
        myDrone.up_down_velocity,
        myDrone.yaw_velocity,
    )
    return error
