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
# xout_preview = pipeline.create(dai.node.XLinkOut)
# xout_preview.setStreamName("preview")
cam_rgb.video.link(xout.input)
# cam_rgb.preview.link(xout_preview.input)

# # Start the device with the created pipeline
# device = dai.Device(pipeline)
# video_queue = device.getOutputQueue(name="video", maxSize=8, blocking=False)

# Define the ArUco dictionary and parameters
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.minMarkerPerimeterRate = 0.05  # Minimum marker size as a fraction of image width
aruco_params.maxMarkerPerimeterRate = 0.50  # Maximum marker size as a fraction of image width

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue('video')
    # preview = device.getOutputQueue('preview')

    # Initialize cv display timer
    cv_time = time.time()

    while True:
        start_time = time.time()

        # Get a frame from the OAK-1 camera
        frame_data = video.get()
        #previewFrame = preview.get()
        frame = frame_data.getCvFrame()
        print(frame.shape)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        # Filter out markers that aren't ID 0
        if ids is not None:
            valid_indices = [i for i, id_ in enumerate(ids) if id_ == 0]
            ids = ids[valid_indices]
            corners = [corners[i] for i in valid_indices]

        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the frame
        # Get BGR frame from NV12 encoded video frame to show with opencv
        if time.time() - cv_time > 0.1:
            cv2.imshow("OAK-1 ArUco Detection", frame)
            cv_time = time.time()
        # Show 'preview' frame as is (already in correct format, no copy is made)
        #cv2.imshow("preview", previewFrame.getFrame())

        process_time = time.time() - start_time
        print(process_time)

        if cv2.waitKey(1) == 27:  # Exit when 'ESC' is pressed
            break

cv2.destroyAllWindows()
device.close()
