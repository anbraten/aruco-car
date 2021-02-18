import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import socket
import sys
import time

MARKER_LENGTH = 0.04 # 40 mm

CAR = True
#CAR = False

if CAR:
  CAR_IP = "192.168.0.190"
  CAMERA = "http://" + CAR_IP + "/mjpeg/1"
  CAMERA_CALIB_FILE = "calibration_matrix-car.yaml"
  # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  # sock.connect((CAR_IP, 1337))
  sock = None
else:
  sock = None
  CAMERA = 4
  CAMERA_CALIB_FILE = "calibration_matrix-webcam.yaml"

cap = cv2.VideoCapture(CAMERA)
if cap.isOpened() is False:
  print("Can't open camera")
  exit(-1)

width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print("size (w:h)", width, ":", height)

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
arucoParameters = aruco.DetectorParameters_create()
with open(CAMERA_CALIB_FILE, "r") as file:
  cameraCalibration = yaml.load(file, Loader=yaml.FullLoader)
  file.close()

cameraMatrix = np.array(cameraCalibration.get("camera_matrix"))
distCoeffs = np.array(cameraCalibration.get("dist_coeff"))

latest_steering = 0
marker_info = ""
socket_info = ""

def signed_with_leading_zero(num):
  res = str(abs(num))

  if abs(num) < 10:
    res = "0" + res

  if num < 0:
    res = "-" + res
  else:
    res = "+" + res

  return res


def drive(base_speed, steering):
  """
  Drive car with base_speed and steering

  Parameters:

  base_speed (int): Base speed in percent (0-100). `0` means stop and 100 means full speed.

  steering (int): Steering in percent (0-100). `-100` means full left turn and `+100` means full right turn. `0` means straight.
  """
  global socket_info


  l_motor = round(base_speed * steering / 100)
  r_motor = round(base_speed * (100 - steering) / 100)
  l_text = "l" + signed_with_leading_zero(l_motor)
  r_text = "r" + signed_with_leading_zero(r_motor)

  socket_info = l_text + " " + r_text
  # print(socket_info)

  if not sock is None:
    sock.sendall((l_text + "\n").encode())
    sock.sendall((r_text + "\n").encode())

def steer(tvec):
  global marker_info
  steering = round(tvec[0, 0] * 100)
  distance = round(tvec[0, 2] * 100)
  steering_with_distance = round((steering / distance) * 300)
  base_speed = min(50, steering_with_distance / 3)

  if steering_with_distance > 100:
    steering_with_distance = 100

  if steering_with_distance < -100:
    steering_with_distance = -100

  # break if closer than 20cm to the cube
  if distance <= 20:
    base_speed = 0
    steering_with_distance = 0

  marker_info = "s:" + str(steering) + "cm d:" + str(distance) + "cm => sd:" + str(steering_with_distance)
  # print(marker_info)
  drive(base_speed, steering)

while True:
  # Capture frame-by-frame
  ret, frame = cap.read()

  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
  aruco.drawDetectedMarkers(frame, corners)
  rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, cameraMatrix, distCoeffs)

  if not rvecs is None:
    for i in range(len(rvecs)):
      rvec = rvecs[i]
      tvec = tvecs[i]

      # draw center of marker
      pp, _jac = cv2.projectPoints(np.float32([0,0,0]), rvec, tvec, cameraMatrix, distCoeffs)
      cv2.circle(frame, tuple(pp[0].astype(int).ravel()), 3, (255, 0, 0), -1)
      center_x = int(tuple(pp[0].ravel())[0])
      cv2.line(frame, (center_x, 0), (center_x, int(height)), (0, 255, 0), 1)

      # aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, MARKER_LENGTH)

      # only steer every second
      now = round(time.time())
      if latest_steering == 0 or now - latest_steering >= 2:
        steer(tvec)
        latest_steering = now

  # stop car if no marker detected
  else:
    drive(0, 0)

  cv2.putText(frame, marker_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
  cv2.putText(frame, socket_info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)

  # draw vertical center line
  cv2.line(frame, (int(width / 2), 0), (int(width / 2), int(height)), (0, 0, 255), 1)

  # Display the resulting frame
  cv2.imshow('Display', frame)

  if cv2.waitKey(1) & 0xFF == ord('q'):
    if not sock is None:
      sock.sendall("!\n")
    break

  # time.sleep(3)

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
