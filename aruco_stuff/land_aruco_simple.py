# land_aruco_simple.py

import depthai as dai
import cv2
import numpy as np
import time
from pymavlink import mavutil

# Initialization ---------------------------------------------------------------
# Initialize MAVLink connection
master = mavutil.mavlink_connection("/dev/serial0", baud=921600)
master.wait_heartbeat()

# OAK Initialization
pipeline = dai.Pipeline()

cam_rgb = pipeline.createColorCamera()
# cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setIspScale(10,10)
cam_rgb.setFps(30)

xout = pipeline.createXLinkOut()		# create output node for video stream
xout.setStreamName("video")
cam_rgb.preview.link(xout.input)		# preview

# Aruco Initialization
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()

# Variables --------------------------------------------------------------------
# Variables for drone control
last_seen = time.time()		# time since last aruco detection
HOVER_TIME = 2  # Time in seconds to hover if marker isn't seen

# Main Loop --------------------------------------------------------------------
# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    # Initialize cv display timer
    cv_time = time.time()

    while True:
        # Loop timer
        start_time = time.time()

        # Get a frame from the OAK-1 camera
        frame_data = video.get()
        frame = frame_data.getCvFrame()

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)		# draw markers

        if ids is not None and 0 in ids:		# for aruco tag id=0
            last_seen = time.time()
            marker_center = np.mean(corners[0][0], axis=0)
            frame_center = [frame.shape[1] // 2, frame.shape[0] // 2]
            offset = marker_center - frame_center

            # Draw offset line
            cv2.line(frame, tuple(map(int, frame_center)), tuple(map(int, marker_center)), (0, 255, 0), 2)

            # TODO: Convert offset to drone movement commands and send via MAVLink
            # For example:
            # master.mav.command_long_send(
            #     master.target_system, master.target_component,
            #     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
            #     offset[0], 0, 0, 0, 0, 0, 0)
            
            # If the marker is close enough to the center, start descending
            if np.linalg.norm(offset) < 30:  # Adjust the threshold as needed
                # TODO: Send a command to reduce altitude slowly
                pass

        # Display the frame
        if time.time() - cv_time > 0.1:
            cv2.imshow("OAK-1 ArUco Detection", frame)
            cv_time = time.time()
        # cv2.imshow("video", frame)

    	# Calculate sampling rate
        process_time = time.time() - start_time
        print(1/process_time)

        if cv2.waitKey(1) == 27:  # Exit when 'ESC' is pressed
            break

cv2.destroyAllWindows()
device.close()