import cv2
import numpy as np
import json
import glob

# Set the checkerboard size (number of inner corners per a chessboard row and column)
CHECKERBOARD = (8, 6)

# Set the square size in meters (update based on your checkerboard dimensions)
SQUARE_SIZE = 0.025

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Load images for calibration (update the path to your calibration images)
images = glob.glob("path_to_your_images/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow('Checkerboard Corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Display calibration results
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

# Save calibration data to a JSON file
calibration_data = {
    'camera_matrix': camera_matrix.tolist(),
    'distortion_coefficients': dist_coeffs.tolist()
}
with open('camera_calibration.json', 'w') as f:
    json.dump(calibration_data, f, indent=4)

print("Calibration complete. Data saved to 'camera_calibration.json'.")
