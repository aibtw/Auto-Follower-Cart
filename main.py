import time
import imutils
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
    s = TCP_init()  # Socket object

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
            tx, ty, tz, norm_x = np.Inf, np.Inf, np.Inf, np.Inf  # initializing
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

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

            send(s, speed, steer)
            receive(s, debug=True)

        # listener.join


if __name__ == '__main__':
    main()
