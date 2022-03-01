import argparse
import sys
import socket
import time
import imutils
from imutils.video import VideoStream
from pynput import keyboard
import numpy as np
from numpy.core import uint16
import cv2
from cv2 import aruco

steer = 0
# positive steer = clockwise

speed = 0
# positive Speed = move ahead (push the cart)

# Units:
# Speed : 110 units = 1m / s
# steer: 40 units = 30 deg / s


def main():
    print("[INFO] Program Starting!")
    # initialize the video stream and allow the camera sensor to warm up
    global speed
    global steer
    print("[INFO] starting video stream...")
    cap = cv2.VideoCapture(1)
    time.sleep(2.0)

    # Get coeffecients and camera matrix from yaml calibration file
    cv_file = cv2.FileStorage("Calibration\calibration_chessboard.yaml", cv2.FileStorage_READ)
    camera_matrix = cv_file.getNode('K').mat()
    distortion_coeffs = cv_file.getNode('D').mat()
    cv_file.release()

    # define the type of aruco marker to detect
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()

    # IP and PORT of the ESP server
    TCP_IP, TCP_PORT = "192.168.4.1", 8000

    # Receive and send buffer information
    Read_Buffer_size_bytes = 22

    # Establish connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    s.settimeout(20)
    print("Connected successfully")
    print(s.getsockname())

    # Main loop
    while True:
        with keyboard.Listener(
                on_press=on_press,
                on_release=on_release) as listener:
            time.sleep(0.2)

        # grab the frame from the threaded video stream and resize it to have a maximum width of 1000 pixels
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=500)
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
                                                           arucoDict,
                                                           parameters=arucoParams,
                                                           cameraMatrix=camera_matrix,
                                                           distCoeff=distortion_coeffs)
        # draw borders
        if len(corners) > 0:
            speed = 25
            aruco.drawDetectedMarkers(frame, corners, ids)
            corners = np.squeeze(corners)
            print(corners)
            center = (corners[0][0] + corners[0][1]) / 2
            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                0.043,
                camera_matrix,
                distortion_coeffs)
            print(tvecs)
            aruco.drawAxis(frame, camera_matrix, distortion_coeffs, rvecs, tvecs, 0.01)
        else:
            speed = 0
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

        send(s)
        receive(s, debug=False)
        # TODO: Determine whether recv function accepts only power-of-2 values (i.e. 32) or it can accept 22 bytes

        listener.join


def send(s):
    # Send Commands
    start = uint16(43981)  # Start (2 bytes)
    steerp = uint16(steer)  # Steer (2 bytes)
    speedp = uint16(speed)  # Speed (2 bytes)
    chkSum = (start ^ steerp) ^ speedp
    chkSum = uint16(chkSum)  # Error detection bytes (2 bytes)
    write_buffer = np.array([start, steerp, speedp, chkSum])
    write_buffer = write_buffer.tobytes()  # Group them into one command as 8 byes
    s.send(write_buffer)  # Send command


def receive(s, debug=False):
    feedback = s.recv(22)
    if len(feedback) >= 22:
        # print("message from server:", feedback.decode("UTF-8"))   # For debugging, if sent message is UTF-8
        # print("message from server:", feedback)                   # For debugging, if sent message is numeric
        header = feedback[0:2]
        cmd1 = feedback[2:4]  # Current steer
        cmd2 = feedback[4:6]  # Current speed
        spdR = feedback[6:8]  # Motor speed R
        spdL = feedback[8:10]  # Motor speed L
        cntR = feedback[10:12]  # Wheels encoder R
        cntL = feedback[12:14]  # Wheels encoder L
        batV = feedback[14:16]  # Battery voltage
        temp = feedback[16:18]  # Temperature
        cmdLED = feedback[18:20]  # LED
        chkSum = feedback[20:22]  # Error detection
        if debug:
            # TODO: Cast all the byte-type variables into the appropriate type (int16-uint16)
            print(f"speed: {int.from_bytes(cmd2, byteorder='little')} | temp: {int.from_bytes(temp, byteorder='little')} | "
                  f"voltage: {int.from_bytes(batV, byteorder='little')}")
            print(f"temp: {int.from_bytes(temp, byteorder='little')}")
            print(f"voltage: {int.from_bytes(batV, byteorder='little')}")


def on_press(key):
    global speed
    global steer
    if key == keyboard.Key.up:
        speed = speed+1
        # print('up: ' + str(speed))
        if speed > 50:
            speed = 50
    if key == keyboard.Key.down:
        speed = speed-1
        # print('down: ' + str(speed))
        if speed < -50:
            speed = -50
    if key == keyboard.Key.left:
        steer = steer-1
        # print('left: ' + str(steer))
        if steer < -50:
            steer = -50
    if key == keyboard.Key.right:
        steer = steer+1
        # print('right: ' + str(steer))
        if steer > 50:
            steer = 50


def on_release(key):
    global speed
    global steer
    # print('{0} released'.format(key))
    if key == keyboard.Key.up:
        speed = 0
    if key == keyboard.Key.down:
        speed = 0
    if key == keyboard.Key.right:
        steer = 0
    if key == keyboard.Key.left:
        steer = 0


def detector():
    pass

if __name__ == '__main__':
    main()






