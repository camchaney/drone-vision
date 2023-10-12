"""

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull

sudo python setup.py build
sudo python setup.py install

Be sure the RASPI CAMERA driver is correctly acivated -> type the following
modprobe bcm2835-v4l2 


"""
# from os import sys, path
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import depthai as dai
import time
import math
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2
import json

# Argument Parsing
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '/dev/serial0')
parser.add_argument('--baud', default = '921600')
args = parser.parse_args()
    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print(f"{dlat}, {dlon}", dLat, dLon)

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav =-y_cam
    y_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        
#--------------------------------------------------
#-------------- CONNECTION  
#--------------------------------------------------    
# Connect to the vehicle
print('Connecting...')
vehicle = connect(args.connect,baud=args.baud,wait_ready=True)  

# OAK Initialization
pipeline = dai.Pipeline()

cam_rgb = pipeline.createColorCamera()
# cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setIspScale(10,10)
cam_rgb.setFps(30)

xout = pipeline.createXLinkOut()        # create output node for video stream
xout.setStreamName("video")
cam_rgb.preview.link(xout.input)        # preview

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 0
marker_size     = 10 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0



#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
# Camera properties
with open("../calibration/oak-1_cal.json", "r") as file:
    calib_data = json.load(file)
    camera_matrix = np.array(calib_data['cameraData'][0][1]['intrinsicMatrix'])
    dist_coeffs = np.array(calib_data['cameraData'][0][1]['distortionCoeff'])                                   
# aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size, show_video=False, 
#                 camera_matrix=camera_matrix, camera_distortion=dist_coeffs)
# Aruco Initialization
# aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters_create()
                
                
time_0 = time.time()

with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    while True:
        # Get a frame from the OAK-1 camera
        frame_data = video.get()
        frame = frame_data.getCvFrame()            

        # marker_found, x_cm, y_cm, z_cm = aruco_tracker.track(loop=False)
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        # if marker_found:
        if ids is not None and 0 in ids:
            marker_center = np.mean(corners[0][0], axis=0)
            frame_center = [frame.shape[1] // 2, frame.shape[0] // 2]
            #offset = marker_center - frame_center
            x_cm, y_cm = camera_to_uav(marker_center[0], markercenter[1])
            uav_location = vehicle.location.global_relative_frame
            
            #-- If high altitude, use baro rather than visual
            if uav_location.alt >= 5.0:
                print 
                z_cm = uav_location.alt*100.0
                
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)

            
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                # print ""
                print( " ")
                print("Altitude = %.0fcm"%z_cm)
                print("Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg))
                
                north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                print("Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg))
                
                marker_lat, marker_lon = get_location_metres(uav_location, north*0.01, east*0.01)  
                #-- If angle is good, descend
                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print("Low error: descending")
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
                else:
                    location_marker = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                    
                vehicle.simple_goto(location_marker)
                print("UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon))
                print("Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon))
                
            #--- Command to land
            if z_cm <= land_alt_cm:
                if vehicle.mode == "GUIDED":
                    print(" -->>COMMANDING TO LAND<<")
                    vehicle.mode = "LAND"

            if cv2.waitKey(1) == 27:  # Exit when 'ESC' is pressed
                break

