import depthai as dai
import cv2
from cv2 import aruco


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
    # camRGB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
    # camRGB.setFps(fps=120)
    camRGB.setVideoSize(1920, 1080)
    xoutVideo.input.setBlocking(False)
    xoutVideo.input.setQueueSize(1)
    # linking the cam node with the pipeline
    camRGB.video.link(xoutVideo.input)
    # camRGB.autofocus(dai.CameraControl.AutoFocusMode.OFF)
    # dai.CameraControl.AutoFocusMode.OFF
    ctrl = dai.CameraControl()
    ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
    ctrl.setAutoFocusTrigger()



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
