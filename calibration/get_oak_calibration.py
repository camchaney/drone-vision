import depthai as dai

# Initialize device
device = dai.Device()

# Fetch the EEPROM calibration data
eeprom = device.readCalibration()

# Specify the camera socket we're interested in
camera_socket = dai.CameraBoardSocket.CAM_A

# Check if calibration data is available for the specified camera
if eeprom.isCameraCalibrationValid(camera_socket):
    # Get the intrinsic calibration parameters
    intrinsics_rgb = eeprom.getCameraIntrinsics(camera_socket)

    # Print the calibration parameters
    print("Camera Matrix:\n", intrinsics_rgb.cameraMatrix)
    print("Distortion Coefficients:\n", intrinsics_rgb.distCoeffs)
else:
    print(f"No calibration data available for camera: {camera_socket}")

# Clean up
device.close()
