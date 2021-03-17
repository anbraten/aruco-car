import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import socket
import sys
import time
import threading
import urllib.request

MARKER_LENGTH = 0.04 # 4 cm

CAR = True
#CAR = False

if CAR:
  CAR_IP = "192.168.0.190"
  CAMERA = "http://" + CAR_IP + "/mjpeg/1"
  CAMERA_CALIB_FILE = "calibration_matrix-car.yaml"
else:
  CAR_IP = None
  CAMERA = 4
  CAMERA_CALIB_FILE = "calibration_matrix-webcam.yaml"

marker_info = ""
socket_info = ""
speed_left = "l+00"
speed_right = "r+00"

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
  global socket_info, speed_left, speed_right

  steering_strength = 5
  steering = (steering_strength * steering / 100)

  l_motor = min(99, round(base_speed + steering) + 5)
  r_motor = min(99, round(base_speed - steering))
  speed_left = "l" + signed_with_leading_zero(l_motor)
  speed_right = "r" + signed_with_leading_zero(r_motor)

  socket_info = speed_left + " " + speed_right

def steer(tvec):
  global marker_info
  steering = round(tvec[0, 0] * 100)
  distance = round(tvec[0, 2] * 100)
  steering_with_distance = round((steering / distance) * 300)
  base_speed = 45 # min(80, distance + 20)

  if steering_with_distance > 100:
    steering_with_distance = 99

  if steering_with_distance < -100:
    steering_with_distance = -99

  # break if closer than 20cm to the cube
  if distance <= 20:
    base_speed = 0
    steering_with_distance = 0

  marker_info = "s:" + str(steering) + "cm d:" + str(distance) + "cm => sd:" + str(steering_with_distance) + " b:" + str(base_speed)
  drive(base_speed, steering_with_distance)

class myDriveThread(threading.Thread):
  def __init__(self, thread_id, name, car_ip):
    if car_ip is None:
      return

    threading.Thread.__init__(self)
    self.thread_id = thread_id
    self.name = name
    self.car_ip = car_ip
  def run(self):
    global speed_left, speed_right

    print("[" + self.name + "] connecting to socket ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((self.car_ip, 1337))
    sock.setblocking(False)
    print("[" + self.name + "] connected to socket")

    while True:
      # print("[" + self.name + "] " + speed_left + " " + speed_right)
      sock.sendall((speed_left + "\n" + speed_right + "\n").encode())
      time.sleep(0.1)

    sock.sendall("!\n".encode())

def image_loop(camera):
  global speed_left, speed_right, marker_info, socket_info, run_drive

  print("connecting to camera at: " + camera)

  cap = cv2.VideoCapture(camera)
  if cap.isOpened() is False:
    print("Can't open camera")
    exit(-1)

  print("Connected to camera")

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

  print("Starting camera loop")

  while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if ret == False:
      continue

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
        steer(tvec)

    # stop car if no marker detected
    else:
      drive(0, 0)

    cv2.putText(frame, marker_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, socket_info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)

    # draw vertical center line
    cv2.line(frame, (int(width / 2), 0), (int(width / 2), int(height)), (0, 0, 255), 1)

    # Display the resulting frame
    cv2.imshow('Display', frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
      break

  cap.release()

  speed_left = "l+00"
  speed_right = "r+00"

threadDrive = myDriveThread(1, "Drive", CAR_IP)
threadDrive.start()

image_loop(CAMERA)

# When everything done, release the capture
cv2.destroyAllWindows()
