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
  else :
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



if __name__ == '__main__':

   
    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)

    # Combine left and right cameras to form a stereo pair
    stereo = getStereoPair(pipeline, monoLeft, monoRight)

    
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
    
    # Pipeline is defined, now we can connect to the device

    with dai.Device(pipeline) as device:

        
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        disparityQueue = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)
        rectifiedLeftQueue = device.getOutputQueue(name="rectifiedLeft", maxSize=1, blocking=False)
        rectifiedRightQueue = device.getOutputQueue(name="rectifiedRight", maxSize=1, blocking=False)


        # Calculate a multiplier for colormapping disparity map
        disparityMultiplier = 255 / stereo.getMaxDisparity()
        
    

        while True:
            
            # Get disparity map
            disparity = getFrame(disparityQueue)
            disparity = (disparity * disparityMultiplier).astype(np.uint8)
            
            ret,threshed= cv2.threshold(disparity,160,255,cv2.THRESH_BINARY)
            
            cv2.imshow("threshed", threshed)
            cv2.imshow("Disparity", disparity)
            #cv2.imshow("D in gray",gray)

            # Check for keyboard input
            key = cv2.waitKey(1)
            if key == ord('q'):
                # Quit when q is pressed
                cv2.destroyAllWindows()
                break
           