import cv2
from datetime import datetime
from imutils.video import VideoStream
import time

def find_available_cameras(max_cameras=10):
    available_cameras = []
    for i in range(max_cameras):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

cameras = find_available_cameras()
print("Available Cameras:", cameras)
