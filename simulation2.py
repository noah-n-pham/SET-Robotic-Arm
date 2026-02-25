import time
import numpy as np
import cv2
from ikpy.chain import Chain
import serial
import cv2.aruco as aruco
import math


# # --- Serial Setup ---
PORT = "COM3"  # windows ver
BAUD = 9600  # same speed as Ro
#Xser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset


# 2/12/26 - added control param
#Xtarget_xyz = [0, 0, 0] # Define a global variable to track where the arm is currently, where tip of cam is
target_xyz = [0.1, 0, 0.15]
Kp = 0.2 # proportional control
center_threshold = 0.005 # 5mm tolerance
z_step = 0.01 # 1cm descent step


# # --- Camera Setup ---
def detect():
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


   while True:
      
       # Read and discard 5 frames using cap.grab (prevents feedback lag)
       for _ in range(5):
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
           if abs(error_x_m) < center_threshold and abs(error_y_m) < center_threshold:
               # If centered → descend Z only
               target_xyz[2] -= z_step
               print("Centered. Descending Z.")
           else:
               # Not centered → correct XY gradually using Kp
               target_xyz[0] += Kp * error_x_m
               target_xyz[1] += Kp * error_y_m
               print("Correcting XY")


           # inverse kinematics
           jointAngles = chain.inverse_kinematics(target_xyz)


           servo_angles_deg = [
               math.degrees(jointAngles[1]),
               math.degrees(jointAngles[2]),
               math.degrees(jointAngles[3])
           ]


           # Send new joint serial angles
           #Xsend_angles(servo_angles_deg)
           print("Target XYZ:", target_xyz)
           print("Servo Angles:", servo_angles_deg)
           print("Marker Z distance:", zC)


           time.sleep(0.5) # prevent motion blur


           # Bottom code...
           #text_pos = (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10)
           #info_text = f"Dist: {zC:.2f}m | X:{xC:.2f} Y:{yC:.2f}"
           #cv2.putText(frame, info_text, text_pos,cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


           #current_time = time.time()
           #if ser is not None and (current_time - lastSend) > sendInt:
              # msg = f"{xC:.2f},{yC:.3f},{zC:.3f}\n"
               #ser.write(msg.encode('utf-8'))
               #print(f"Sent: {msg.strip()}")
               #lastSend = current_time
           # ...till here is apparently send_angles_from_pos()


       cv2.imshow("ArUco Tracking", frame)
       if cv2.waitKey(1) & 0xFF == ord('q'):
           break


   # Cleanup
   cap.release()
   cv2.destroyAllWindows()


chain = Chain.from_urdf_file("3_DOF.urdf")


# 12/2/26 - removed function bc apparently we already compute IK inside camera()
#def send_angles_from_pos(targetPosition):
   # targetPosition is a list of [x, y, z] coordinates
   #jointAngles = chain.inverse_kinematics(targetPosition)
   #servo_angles_deg = [math.degrees(a) for a in jointAngles[1:]]
   #msg = f"{servo_angles_deg[1]:.2f},{servo_angles_deg[2]:.2f},{servo_angles_deg[3]:.2f}\n"  # format: 45.00,30.00\n
   #ser.write(msg.encode())
   #print(f"Sent: {msg.strip()}")


def send_angles(angles):
   msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"  # format: 45.00,30.00\n
   ser.write(msg.encode())
   print(f"Sent: {msg.strip()}")




# # --- Run Program ---


camera()
send_angles([0, 0, 0])  # return arm to neutral (initial pos) at end

while True():
    camera()

#Xser.close()
#Xprint("Done!")  # done wahoo


# 2/12/26 - BELOW: simulation code


# active_links_mask = [False, False, True, True, True, False]
# arm_chain = Chain.from_urdf_file(
#    "3_DOF.urdf",
#    base_elements=["world"],
#    active_links_mask=active_links_mask
# )


# target_pos = [0.5,-0.5, 0.0]
# p.createMultiBody(
#     baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1]),
#     basePosition=target_pos
# )


# p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition = 4.23) #
# p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition = 0.6) # angle of base to first arm
# p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition = 0.6) # angle of first to second arm
# num = p.getNumJoints(robot_id)
# for i in range(num): 
#    info = p.getJointInfo(robot_id, i)
#    print(i, info[1].decode())


# for i in range(2000):
#    p.stepSimulation()
#    time.sleep(1/240)


#0 world_to_base
#1 base_yaw
#2 joint1
#3 joint2
#4 ee_fixed
