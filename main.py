import time
import imutils
import math
import depthai as dai
# from pynput import keyboard
import numpy as np
import cv2
from cv2 import aruco
from TCPLink import TCP_init, send, receive
from OAKdLink import oakd_init, set_oakd_props, aruco_init
from Controller import aruco_control, frame_counter, max_x


def main():
    print("[INFO] Program Starting!")
    steer = 0
    # positive steer = clockwise
    speed = 0
    # positive Speed = move ahead (push the cart)
    # Units:
    #   Speed : 110 units = 1m / s
    #   steer: 40 units = 30 deg / s

    print("[INFO] Setting up oak-d cam ...")
    pipeline, camRGB, xoutVideo = oakd_init()
    set_oakd_props(camRGB, xoutVideo)

    print("[INFO] Setting up Aruco dictionary and camera coefficients ...")
    camera_matrix, distortion_coeffs, arucoDict, arucoParams = aruco_init()

    time.sleep(2.0)  # Necessary !!!

    print("[INFO] Initializing TCP connection ...")
    # s = TCP_init()  # Socket object

    fc = 0  # Frame counter

    # Main loop
    with dai.Device(pipeline) as device:  # used with OAK-D camera
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)  # establish queue
        # start = timeit.default_timer()  # Enable for debugging
        while True:
            # with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
            #     pass

            videoIn = video.get()  # OAK-D cam
            frame = videoIn.getCvFrame()  # OAK-D cam
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(gray,
                                                               arucoDict,
                                                               parameters=arucoParams,
                                                               cameraMatrix=camera_matrix,
                                                               distCoeff=distortion_coeffs)
            tx, ty, tz, norm_x = np.Inf, np.Inf, np.Inf, np.Inf  # initializing position values
            rx, ry, rz = np.Inf, np.Inf, np.Inf  # initializing rotation values
            # draw borders
            if len(corners) > 0:  # add remote control boolean
                fc = frame_counter(fc, inc=True, dec=False)

                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    0.258,
                    # 0.053,
                    # 0.07,
                    camera_matrix,
                    distortion_coeffs)

                # Draw the detected marker and its axis
                aruco.drawDetectedMarkers(frame, corners, ids)
                aruco.drawAxis(frame, camera_matrix, distortion_coeffs, rvecs, tvecs, 0.01)

                # Extract information of the position of the detected marker for interpretation
                tvecs = np.squeeze(tvecs)
                tx, ty, tz = tvecs[0] * 100, tvecs[1] * 100, tvecs[2] * 100
                maximum_x = max_x(81, tz)
                norm_x = tx / maximum_x * 10

                rvecs = np.squeeze(rvecs)
                rx, ry, rz = obtain_angles(rvecs)
                
                speed, steer = aruco_control(tz, 400, 90, 60, norm_x)

                # print("distances: [x: {:.2f},".format(tx), "y: {:.2f}, ".format(ty), "z: {:.2f}] ".format(tz),
                #       ", norm x position: {:.3f}".format(tx / maximum_x * 10))  # Why multiply by 10?
                # Add text to the image
            else:
                fc = frame_counter(fc, inc=False, dec=True)
                if fc == 0:
                    speed, steer = 0, 0

            # Resize the frame (This is safe, because we already did the processing)
            frame = imutils.resize(frame, 800)

            # Enable for debugging
            frame = cv2.flip(frame, 1)

            # Put text on the frame to display it
            cv2.putText(frame, "Position: x:%.2f, y:%.2f, z:%.2f, x_percentage:%3f" % (tx, ty, tz, norm_x),
                        (0, 100), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                        color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)
            if rz <0: rz+=360
            cv2.putText(frame, "Rotation: x:%.2f, y:%.2f, z:%.2f" % (rx,ry,rz),
                        (0, 200), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1,
                        color=(0, 255, 255), thickness=2, lineType=cv2.LINE_AA)

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

            # send(s, speed, steer)
            # receive(s, debug=True)

        # listener.join
        
def obtain_angles(rvec):
    # https://github.com/tizianofiorenzani/how_do_drones_work/blob/master/opencv/aruco_pose_estimation.py
    #-- Obtain the rotation matrix tag->camera
    R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc    = R_ct.T
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0

    #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
    roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

    #-- Print the marker's attitude respect to camera frame
    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                        math.degrees(yaw_marker))
    return math.degrees(roll_marker),math.degrees(pitch_marker), math.degrees(yaw_marker)


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])



if __name__ == '__main__':
    main()
