import numpy as np
import cv2
import yaml
#import pathlib

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cap = cv2.VideoCapture("http://192.168.0.190/mjpeg/1")
# cap = cv2.VideoCapture(4)

print("press 'space' to capture next picture or 'q' to quit")

found = 0
while found < 10:
  # Capture frame-by-frame
  ret, frame = cap.read()
  cv2.imshow('video', frame)

  key = cv2.waitKey(1)

  if key & 0xFF == ord(' '): # wait for space to capture the next picture
    print("next picture")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
    # If found, add object points, image points (after refining them)

    if ret == True:
      objpoints.append(objp)   # Certainly, every loop objp is the same, in 3D.
      corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
      imgpoints.append(corners2)
      img = frame
      # Draw and display the corners
      img = cv2.drawChessboardCorners(img, (7,7), corners2, ret)
      found += 1
      cv2.imshow('img', img)
      cv2.waitKey(500)
    else:
      print("Cant detect chesboard. Please Try again")

  elif key & 0xFF == ord('q'): # wait for space to end
    break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

if found == 0:
  print("Please take more pictures to calibrate")

else:
  print("Number of pictures used for calibration: ", found)

  # calibration
  ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

  print(mtx)
  print("\ndist: \n")
  print(dist)

  # transform the matrix and distortion coefficients to writable lists
  data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

  # and save it to a file
  with open("calibration_matrix.yaml", "w") as f:
    yaml.dump(data, f)
