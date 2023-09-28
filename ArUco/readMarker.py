import depthai as dai
import cv2
import numpy as np

# Initialize the OAK-1 pipeline
pipeline = dai.Pipeline()

# Create a color camera node and set its properties
cam_rgb = pipeline.createColorCamera()
# cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

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
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue('video')
    # preview = device.getOutputQueue('preview')

    while True:
        # Get a frame from the OAK-1 camera
        frame_data = video.get()
        #previewFrame = preview.get()
        frame = frame_data.getCvFrame()

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the frame
        #cv2.imshow("OAK-1 ArUco Detection", frame)
        # Get BGR frame from NV12 encoded video frame to show with opencv
        cv2.imshow("video", frame)
        # Show 'preview' frame as is (already in correct format, no copy is made)
        #cv2.imshow("preview", previewFrame.getFrame())

        if cv2.waitKey(1) == 27:  # Exit when 'ESC' is pressed
            break

cv2.destroyAllWindows()
device.close()
