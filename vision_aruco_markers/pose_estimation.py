'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
'''


import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
import imutils
from scipy.spatial.distance import pdist


# Used for pose estimation of the aruc markers using the frame of the video
def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients,count):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    tvec - translation vector
    rvec - rotation vector
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    #
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)

    # Defining lists of trans and rotation vector
    tvecs=[]
    rvecs=[]

    # If ArUcos were detect and its not none thus create arrays for translation and rotation vector
    if ids is not None:
        tvecs=np.zeros((len(ids),4),dtype=np.float32)
        rvecs=np.zeros((len(ids),4),dtype=np.float32)


    # If markers are detected
    if len(corners) > 0:
        # Loop through the identified marker ids
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)

            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)

            # Storing the tvec and rvecs
            if ids is not None:
                # Adding the translation vectors into a cumulative array with their corresponding ID
                tvecs[i,:] = np.insert(tvec,0,ids[i])
                # Adding the rot vectors into a cumulative array with their corresponding ID
                rvecs[i,:] = np.insert(rvec,0,ids[i])


            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


    return frame, tvecs,rvecs

# Sets up the dict. for aruco marker detection
def setup_aruco_detection():
        ap = argparse.ArgumentParser()
        ap.add_argument("-k", "--K_Matrix", required=False, help="Path to calibration matrix (numpy file)")
        ap.add_argument("-d", "--D_Coeff", required=False, help="Path to distortion coefficients (numpy file)")
        ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
        args = vars(ap.parse_args())

        aruco_dict_type = ARUCO_DICT["DICT_5X5_100"]
        calibration_matrix_path = 'calibration_matrix.npy'
        distortion_coefficients_path = 'distortion_coefficients.npy'


        if ARUCO_DICT.get(args["type"], None) is None:
            print(f"ArUCo tag type '{args['type']}' is not supported")
            sys.exit(0)



        k = np.load(calibration_matrix_path)
        d = np.load(distortion_coefficients_path)

        return k,d,aruco_dict_type

# Used to calculate the displacement from the origin marker
# i.e. moves markers to the ref. Origin ref frame
def calculate_dist_origin(poses,count):
    new_poses=0
    origin_ID=10
    if np.shape(poses)[0]>=2:
        origin=poses[poses[:,0]==origin_ID]


        new_poses=poses-origin
        new_poses[:,0]=poses[:,0]

    return new_poses

# Used to calculate the orientation from the origin
def calculate_orientation_origin(RPY,count):
    new_RPY=0
    origin_ID=10
    if np.shape(RPY)[0]>=2:
        origin=RPY[RPY[:,0]==origin_ID]


        new_RPY=RPY-origin
        new_RPY[:,0]=RPY[:,0]

    return new_RPY

if __name__ == '__main__':

    k,d,aruco_dict_type=setup_aruco_detection()

    video = cv2.VideoCapture(2)
    time.sleep(2.0)

    # Counting frames
    count=0


    while True:

        ret, frame = video.read()
        frame = imutils.resize(frame, width=1000)
        if not ret:
            break

        output,tvecs,rvecs = pose_esitmation(frame, aruco_dict_type, k, d,count)


        cv2.imshow('Estimated Pose', output)

        # Calculaing Pose (XYZ) and Orientation (Roll,Pitch,Yaw) based on origin marker
        new_poses=calculate_dist_origin(tvecs,count)
        new_RPY=calculate_orientation_origin(rvecs,count)

        # Update Print Statement every 15 frames (aprox every 0.5 sec)
        if count%15==0:
        #     print("      ID          X           Y           Z")
        #     print(tvecs,'\n')
        #     print("          ID      Roll        Pitch       Yaw")
        #     print(rvecs)
            print("XYZ:\n",new_poses)
            print("RPY:\n",new_RPY)

        # Frame Counter incremented
        count=count+1

        # Releasing current frame
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
