import numpy as np
import cv2 as cv
import sys
import time

# Save transformation, rotation and aruco ID globally. The values will be stored as a 1x3 matrix
trans_vector = [0, 0, 0]
rot_vector = [0, 0, 0]
aruco_id_read = 0

# Differnt aruco types and markers we have available
ARUCO_DICT = {
    "DICT_4X4_50": cv.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11,
}

# Find corners from image, and display the ID the aruco code has
def aruco_display(corners, ids, rejected, image):

    if (
        len(corners) > 0
    ):  #  If the code has found a corner(aruco code) then this code will run

        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv.putText(
                image,
                str(markerID),
                (topLeft[0], topLeft[1] - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            aruco_id_read = markerID
            # print("[Inference] ArUco marker ID: {}".format(markerID))
    return image


# Find pose for detected markers and display the 3D pose visually
def pose_estimation(
    frame, aruco_dict_type, matrix_coefficients, distortion_coefficients
):

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.aruco_dict = cv.aruco.Dictionary_get(
        aruco_dict_type
    )  # What type of aruco marker do we want to detect
    parameters = cv.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv.aruco.detectMarkers(
        gray,
        cv.aruco_dict,
        parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients,
    )
    if len(corners) > 0:
        for i in range(0, len(ids)):

            rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(
                corners[i], 0.02, matrix_coefficients, distortion_coefficients
            )

            cv.aruco.drawDetectedMarkers(frame, corners)

            cv.aruco.drawAxis(
                frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01
            )
            gainVal = 50
            trans_vector[0] = round(tvec[0][0][0] * gainVal, 3)
            trans_vector[1] = round(tvec[0][0][1] * gainVal, 3)
            trans_vector[2] = round(tvec[0][0][2] * gainVal, 3)

            rot_vector[0] = round(rvec[0][0][0] * gainVal, 3)
            rot_vector[1] = round(rvec[0][0][1] * gainVal, 3)
            rot_vector[2] = round(rvec[0][0][2] * gainVal, 3)

            aruco_display(corners, ids, None, frame)
    else:
        trans_vector[0] = 0
        trans_vector[1] = 0
        trans_vector[2] = 0

        rot_vector[0] = 0
        rot_vector[1] = 0
        rot_vector[2] = 0

    tvec_str = "Positional vectors: x=%4.3f,   y=%4.3f,   z=%4.3f" % (
        trans_vector[0],
        trans_vector[1],
        trans_vector[2],
    )
    rvec_str = " Rotational vectors: x=%4.3f,   y=%4.3f,   z=%4.3f" % (
        rot_vector[0],
        rot_vector[1],
        rot_vector[2],
    )
    cv.putText(
        frame,
        tvec_str,
        (0, 20),
        cv.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 100),
        2,
    )
    cv.putText(
        frame,
        rvec_str,
        (0, 40),
        cv.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 100),
        2,
    )
    return frame


aruco_type = "DICT_4X4_100"
arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
arucoParams = cv.aruco.DetectorParameters_create()


# Values from calibration
"""intrinsic_camera = np.array(
    [[1831.13105, 0, 1292.64501], [0, 1832.67544, 1010.07140], [0, 0, 1]]
)
distortion = np.array([-0.01928008, 0.15817319, -0.00395639, -0.00095826])"""


# Values from a webpage
"""
intrinsic_camera = np.array(
    [[921.170702, 0, 459.904354], [0, 919.018377, 351.238301], [0, 0, 1]]
)
distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647])
"""

# Values from youtube tutorial
intrinsic_camera = np.array(
    [[933.15867, 0, 657.59], [0, 933.1586, 400.36993], [0, 0, 1]]
)

distortion = np.array([-0.43948, 0.18514, 0, 0])
