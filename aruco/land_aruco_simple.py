# land_aruco_simple.py

import depthai as dai
import cv2
import numpy as np
import time

# Initialize the OAK-1 pipeline
pipeline = dai.Pipeline()

# Create a color camera node and set its properties
cam_rgb = pipeline.createColorCamera()
# cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setIspScale(10,10)
cam_rgb.setFps(30)

# Create an XLink output node for the video stream
xout = pipeline.createXLinkOut()
xout.setStreamName("video")
# cam_rgb.video.link(xout.input)

# Preview
# xout_preview = pipeline.create(dai.node.XLinkOut)
# xout_preview.setStreamName("preview")
cam_rgb.preview.link(xout.input)

# Define the ArUco dictionary and parameters
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()
