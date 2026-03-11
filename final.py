import time
import numpy as np
import cv2
from ikpy.chain import Chain
import serial
import cv2.aruco as aruco
import math


chain = Chain.from_urdf_file("3_DOF.urdf")
# # --- Serial Setup ---
PORT = "COM3"  # windows ver
BAUD = 9600  # same speed as Ro
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset


# 2/12/26 - added control param
current_xyz = [0, 0, 0] # Define a global variable to track where the arm is currently, where tip of cam is
target_xyz = [0.1, 0, 0.15]
Kp = 0.2 # proportional control
center_threshold = 0.005 # 5mm tolerance
z_step = 0.01 # 1cm descent step


state = [current_xyz, target_xyz, [0,0,0], math.inf]
#detect (returns target pos) ->
#center xy ->
#descend z until uncentered ->
#center xy again ->
#descend and repeat until target position reached- >
#activate gripper


def detect_and_center():
   while True:
       port = '/dev/ttyACM0'
       global target_xyz


       # Initialize webcam
       cap = cv2.VideoCapture(0)  # 0 = default webcam


       if not cap.isOpened():
           print("Error: Could not open video stream.")
           exit()
      
       arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
       parameters = aruco.DetectorParameters()
       detector = aruco.ArucoDetector(arucoDict, parameters)
       markerLength = 0.05
       # lastSend = 0  # old var?
       # sendInt = 2  # old var?


       for _  in range(5):
           cap.grab()




       ret, frame = cap.read()
       if not ret:
           break




       h, w = frame.shape[:2]
       f = w




       cameraMatrix = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1]], dtype=np.float32)
      
       distCoeffs = np.zeros((5, 1))
       gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
       corners, ids, rejected = detector.detectMarkers(gray)
       #Xcorners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=parameters)
    
       # if the ArUco tag is detected properly, determine the position of the object based on the tag’s communicated information + initial camera view matrix, convert from pixels to meters
       if ids is not None:
           aruco.drawDetectedMarkers(frame, corners, ids)


           rvecs = []
           tvecs = []


            for corner in corners:




               objp = np.array([
                  [-markerLength/2,  markerLength/2, 0],
                  [ markerLength/2,  markerLength/2, 0],
                  [ markerLength/2, -markerLength/2, 0],
                  [-markerLength/2, -markerLength/2, 0]
              ], dtype=np.float32)




               retval, rvec, tvec = cv2.solvePnP(
                  objp,
                  corner,
                  cameraMatrix,
                  distCoeffs
              )




                rvecs.append(rvec)
                tvecs.append(tvec)




           for i in range(len(ids)):
               cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs,
                              rvecs[i], tvecs[i], 0.03)




           xC, yC, zC = tvecs[0].flatten()
          #Commented out below, lower OpenCV code
          #rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
          #for i in range(len(ids)):
              #try:
                  #cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
              #except AttributeError:
                  #aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
          # Get Aruco pose (2/12/26 - everything below newly added)
          #xC, yC, zC = tvecs[i][0]  # 2/12/26 - deleted current_tvec




          # compute XY pixel error
           cx = w / 2
           cy = h / 2




          #Xtag_x = np.mean(corners[i][0][:, 0])
          #Xtag_y = np.mean(corners[i][0][:, 1])
           tag_x = np.mean(corners[0][0][:, 0])
           tag_y = np.mean(corners[0][0][:, 1])




            error_x_pixels = tag_x - cx
            error_y_pixels = tag_y - cy




          # convert from pixel error to meter error
           error_x_m = (error_x_pixels * zC) / f
           error_y_m = (error_y_pixels * zC) / f




          # control logic
       #    if abs(error_x_m) < center_threshold and abs(error_y_m) < center_threshold:
       #        # If centered → descend Z only
       #        target_xyz[2] -= z_step
       #        print("Centered. Descending Z.")
       #    else:
       #        # Not centered → correct XY gradually using Kp


           # if abs(error_x_m) >= center_threshold or abs(error_y_m) >= center_threshold:
           #     target_xyz[0] += Kp * error_x_m
           #     target_xyz[1] += Kp * error_y_m
           #     print("Correcting XY")


           # Directly use marker pose as target
           target_xyz = [xC, yC, zC]


           # inverse kinematics
           jointAngles = chain.inverse_kinematics(target_xyz)
           servo_angles_deg = [
               math.degrees(jointAngles[1]),
               math.degrees(jointAngles[2]),
               math.degrees(jointAngles[3])
           ]
           send_angles(servo_angles_deg)


           # TODO: get current position and return that as well
           state = [current_xyz, target_xyz, servo_angles_deg, zC]
           return state
   return None


def descend_z(distance):
   #send signal to arduino to descend z by one unit
   # call ikpy to calculate new angles for z
   send_angles()
   return 0


def grip():
   #send signal to arduino to activate gripper
   return 0


def send_angles(angles):
   msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"
   ser.write(msg.encode())
   print(f"Sent: {msg.strip()}")


def main():
   while True:
       result = detect_and_center()
       vertical = result[3] if result else math.inf  # send angles if detection successful, else send zeros
       if vertical < 0.1:
           break
       descend_z(vertical)
   grip()


if __name__ == "__main__":
   main()