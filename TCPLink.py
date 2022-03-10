import socket
import time
import imutils
import depthai as dai
#from pynput import keyboard
import numpy as np
from numpy.core import uint16
import cv2
from cv2 import aruco
import timeit

steer = 0
# positive steer = clockwise

speed = 0
# positive Speed = move ahead (push the cart)

# Units:
# Speed : 110 units = 1m / s
# steer: 40 units = 30 deg / s


def main():
    print("[INFO] Program Starting!")

    # global steer and speed variables
    global speed
    global steer

    # Set up OAK-D cam
    print("[INFO] Setting up oak-d cam ...")
    pipeline, camRGB, xoutVideo = oakd_init()
    set_oakd_props(camRGB, xoutVideo)

    print("[INFO] Setting up Aruco dictionary and camera coefficients ...")
    camera_matrix, distortion_coeffs, arucoDict, arucoParams = aruco_init()

    time.sleep(2.0)  # Necessary !!!

    print("[INFO] Initializing TCP connection ...")
    s = TCP_init()
    start = 0
    # Main loop
    with dai.Device(pipeline) as device:  # used with OAK-D camera
        video = device.getOutputQueue(name="video", maxSize=1, blocking=False)  # establish queue
        while True:
            videoIn = video.get()   # OAK-D cam
            frame = videoIn.getCvFrame()    # OAK-D cam
            # with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
            #     pass

            # detect ArUco markers in the input frame
            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame,
                                                               arucoDict,
                                                               parameters=arucoParams,
                                                               cameraMatrix=camera_matrix,
                                                               distCoeff=distortion_coeffs)
            # draw borders
            if len(corners) > 0:        #add remote control boolean
                start = timeit.default_timer()

                aruco.drawDetectedMarkers(frame, corners, ids)
                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    0.07,
                    camera_matrix,
                    distortion_coeffs)
                aruco.drawAxis(frame, camera_matrix, distortion_coeffs, rvecs, tvecs, 0.01)
                tvecs = np.squeeze(tvecs)
                tx,ty,tz = tvecs[0]*100, tvecs[1]*100, tvecs[2]*100
                maximum_x = max_x(81, tz)
                norm_x = tx/maximum_x*10
                print("distances: [x: {:.2f},".format(tx),
                      "y: {:.2f}, ".format(ty),
                      "z: {:.2f}] ".format(tz),
                      ", norm x position: {:.3f}".format(tx/maximum_x*10))  # Why multiply by 10?
                aruco_control(tz, 300, 90, 60, norm_x)
            else:
                stop = timeit.default_timer()
                print('Time: ', stop - start)
                if stop - start > 0.5:
                    speed = 0
                    steer = 0
                else:
                    pass

            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break

            send(s)
            receive(s, debug=False)
            # if cv2.waitKey(256):
                # cv2.destroyAllWindows()

            # TODO: Determine whether recv function accepts only power-of-2 values (i.e. 32) or it can accept 22 bytes

        # listener.join


def aruco_control(dist, max_threshold, forward_threshold, back_threshold, x):
    global speed
    global steer
    if forward_threshold < dist < max_threshold:
        move_forward()
    elif dist < back_threshold:
        move_backward()
    else:
        speed = 0

    if x >= 0.25:
        steer = 20
    elif x <= -0.25:
        steer = -20
    else:
        steer = 0
    print("current speed: " + str(speed))


def move_forward():
    global speed
    speed = 45


def move_backward():
    global speed
    speed = -20


def get_user_speed(dist, t):
    pass


def aruco_init():
    # Get coefficients and camera matrix from yaml calibration file
    cv_file = cv2.FileStorage("calibration_chessboard.yaml", cv2.FileStorage_READ)
    camera_matrix = cv_file.getNode('K').mat()
    distortion_coeffs = cv_file.getNode('D').mat()
    cv_file.release()
    # define the type of aruco marker to detect
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    return camera_matrix, distortion_coeffs, arucoDict, arucoParams


def TCP_init():
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
    return s


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


# def on_press(key):
#     global speed
#     global steer
#     if key == keyboard.Key.up:
#         speed = speed+1
#         # print('up: ' + str(speed))
#         if speed > 50:
#             speed = 50
#     if key == keyboard.Key.down:
#         speed = speed-1
#         # print('down: ' + str(speed))
#         if speed < -50:
#             speed = -50
#     if key == keyboard.Key.left:
#         steer = steer-1
#         # print('left: ' + str(steer))
#         if steer < -50:
#             steer = -50
#     if key == keyboard.Key.right:
#         steer = steer+1
#         # print('right: ' + str(steer))
#         if steer > 50:
#             steer = 50


# def on_release(key):
#     global speed
#     global steer
#     # print('{0} released'.format(key))
#     if key == keyboard.Key.up:
#         speed = 0
#     if key == keyboard.Key.down:
#         speed = 0
#     if key == keyboard.Key.right:
#         steer = 0
#     if key == keyboard.Key.left:
#         steer = 0


def oakd_init():
    pipeline = dai.Pipeline()  # pipeline object, only one is needed.
    camRGB = pipeline.create(dai.node.ColorCamera)  # this is the node for the RGB camera (the middle camera)
    xoutVideo = pipeline.create(dai.node.XLinkOut)

    return pipeline, camRGB, xoutVideo


def set_oakd_props(camRGB, xoutVideo):
    # Use your connection to stream stuff.
    xoutVideo.setStreamName("video")
    # properties for the camera
    camRGB.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRGB.setVideoSize(1920,1080)
    xoutVideo.input.setBlocking(False)
    xoutVideo.input.setQueueSize(1)
    # linking the cam node with the pipeline
    camRGB.video.link(xoutVideo.input)


# applying Pythagorean theory
def max_x(angle, adjacent):
    tang = adjacent / np.cos(angle * np.pi / 180)
    opposite = np.sqrt((tang ** 2) - (adjacent ** 2))
    return opposite


if __name__ == '__main__':
    main()






