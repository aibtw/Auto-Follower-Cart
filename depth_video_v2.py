# -*- coding: utf-8 -*-
"""
Created on Sun Apr 24 17:25:24 2022

@author: Fahd
"""

import cv2
import depthai as dai
import numpy as np


def getFrame(queue):
    # Get frame from queue
    frame = queue.get()
    # Convert frame to OpenCV format and return
    return frame.getCvFrame()


def getMonoCamera(pipeline, isLeft):
    # Configure mono camera
    mono = pipeline.createMonoCamera()

    # Set Camera Resolution
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    if isLeft:
        # Get left camera
        mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else:
        # Get right camera
        mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono


def getStereoPair(pipeline, monoLeft, monoRight):
    # Configure stereo pair for depth estimation
    stereo = pipeline.createStereoDepth()
    # Checks occluded pixels and marks them as invalid
    stereo.setLeftRightCheck(True)

    # Configure left and right cameras to work as a stereo pair
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    return stereo


def setter(pipeline, stereo):
    # Set XlinkOut for disparity, rectifiedLeft, and rectifiedRight
    xoutDisp = pipeline.createXLinkOut()
    xoutDisp.setStreamName("disparity")

    xoutRectifiedLeft = pipeline.createXLinkOut()
    xoutRectifiedLeft.setStreamName("rectifiedLeft")

    xoutRectifiedRight = pipeline.createXLinkOut()
    xoutRectifiedRight.setStreamName("rectifiedRight")

    stereo.disparity.link(xoutDisp.input)

    stereo.rectifiedLeft.link(xoutRectifiedLeft.input)
    stereo.rectifiedRight.link(xoutRectifiedRight.input)

    return xoutDisp, xoutRectifiedRight, xoutRectifiedLeft


def get_queues(device):
    disparityQueue = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
    rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
    rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)

    return disparityQueue, rectifiedRightQueue, rectifiedLeftQueue


def get_map(disparityQueue, disparityMultiplier):
    # Get disparity map
    disparity = getFrame(disparityQueue)
    disparity = (disparity * disparityMultiplier).astype(np.uint8)
    ret, threshed = cv2.threshold(disparity, 100, 255, cv2.THRESH_BINARY)
    # print(np.shape(disparity))
    # REGION OF INTRESET
    section = threshed[0:400, 120:500]
    ROI = np.average(section)

    return ROI, section


def depth_init(pipeline):
    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft=True)
    monoRight = getMonoCamera(pipeline, isLeft=False)

    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)

    return monoRight, monoLeft, stereo
