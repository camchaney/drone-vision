# List calibration values
import depthai as dai

# Create device and pipeline
device = dai.Device()
pipeline = dai.Pipeline()

# Create the color camera node
cam_rgb = pipeline.createColorCamera()

# Fetch the intrinsic calibration parameters
intrinsics = cam_rgb.getIntrinsics()

# Print the calibration parameters
print("Camera Matrix:\n", intrinsics.cameraMatrix)
print("Distortion Coefficients:\n", intrinsics.distCoeffs)

# Clean up
device.close()