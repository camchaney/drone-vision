import depthai as dai

# Initialize device
device = dai.Device()

# Fetch the EEPROM calibration data
eeprom = device.readCalibration()

# Get the intrinsic calibration parameters for the RGB camera
intrinsics_rgb = eeprom.getCameraIntrinsics(dai.CameraBoardSocket.RGB)

# Print the calibration parameters
print("Camera Matrix:\n", intrinsics_rgb.cameraMatrix)
print("Distortion Coefficients:\n", intrinsics_rgb.distCoeffs)

# Clean up
device.close()