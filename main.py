from utils import *  # Import all the files and library necessary
from PID import *

w, h = 1280, 720  # HVIS POSE ER FEIL, KAN DET VÆRE MED DIMENSJONSFEIL HER

# Initializes center frame of where aruco code should be placed
"""center_x = w / 2
center_y = h / 2"""
# Run script from utils. This is a main sequence to initialize and start the drone
myDrone = initializeTello()

# Loop iteration as long as q is not pressed (ends the program)
while True:
    # Get picture input from drone and display it
    img = telloGetFrame(myDrone, w, h)

    # Estimate pose from aruco codes with related text
    output = pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)

    # Run manual key control. T is for takeoff
    manual_key_control()

    previousError_x_dir = PID_reg_x_dir(
        myDrone, trans_vector[0], pid_x_dir, previousError_x_dir, 0
    )
    previousError_y_dir = PID_reg_y_dir(
        myDrone, trans_vector[1], pid_y_dir, previousError_y_dir, 0
    )
    previousError_z_dir = PID_reg_z_dir(
        myDrone, trans_vector[2], pid_z_dir, previousError_z_dir, 8
    )

    # Display picture (output)
    cv.imshow("Estimated Pose", output)

    # stop program and land drone if q is pressed
    if cv.waitKey(1) & 0xFF == ord("q"):
        cv.destroyAllWindows()
        myDrone.land()
        break
