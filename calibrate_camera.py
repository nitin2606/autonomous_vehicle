import cv2
import numpy as np
import glob

# Set the dimensions of the checkerboard
CHECKERBOARD = (8, 6)  # Number of internal corners
square_size = 0.025    # Square size in meters

# Termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Prepare the object points (0,0,0), (1,0,0), ..., (7,5,0) scaled by square_size
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# Capture images or load from a directory
images = glob.glob("calibration_images/*.jpg")  # Replace with your image directory

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(refined_corners)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, refined_corners, ret)
        cv2.imshow('Checkerboard', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save calibration data
np.savez("camera_calibration_data.npz", 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs, 
         rvecs=rvecs, 
         tvecs=tvecs)

# Print results
print("Camera matrix:")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)
