# import jetson.inference
# import jetson.utils
import cv2
import depthai as dai
import numpy as np
import json

net = None
camera = None
device = None
video = None

# Aruco Initialization
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()

def initialize_detector():
	global net, camera
	# net = jetson.inference.detectNet("ssd-mobilenet-v2")
	# camera = jetson.utils.videoSource("csi://0")      # '/dev/video0' for V4L2
	# OAK Initialization
	pipeline = dai.Pipeline()
	device = dai.Device(pipeline)

	camera = pipeline.createColorCamera()
	camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
	camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
	camera.setIspScale(10,10)
	camera.setFps(30)

	xout = pipeline.createXLinkOut()		# create output node for video stream
	xout.setStreamName("video")
	camera.preview.link(xout.input)		# preview

	# Camera properties
	with open("calibration/oak-1_cal.json", "r") as file:
		calib_data = json.load(file)
		camera_matrix = np.array(calib_data['cameraData'][0][1]['intrinsicMatrix'])
		dist_coeffs = np.array(calib_data['cameraData'][0][1]['distortionCoeff'])

	print("Camera initialized")
	return video

def get_image_size():
	return camera.GetWidth(), camera.GetHeight()

def close_camera():
	camera.Close()

def get_detections(vid):
	aruco_detections = []
	# img = camera.Capture()
	# Get a frame from the OAK-1 camera
	frame_data = vid.get()
	frame = frame_data.getCvFrame()
	# detections = net.Detect(img)
	corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
	# for detection in detections:
	# 	if detection.ClassID == 1: #remove unwanted classes
	# 		aruco_detections.append(detection)

	fps = net.GetNetworkFPS()

	# return aruco_detections, fps, jetson.utils.cudaToNumpy(img)
	return corners, ids, frame

