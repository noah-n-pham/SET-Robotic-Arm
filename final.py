import time
import numpy as np
import cv2
from ikpy.chain import Chain
import serial
import cv2.aruco as aruco
import math

chain = Chain.from_urdf_file("3_DOF.urdf")

# --- Serial Setup ---
PORT = "COM3"
BAUD = 9600
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# control params
current_xyz = [0, 0, 0]
target_xyz = [0.1, 0, 0.15]

Kp = 0.2
center_threshold = 0.005
z_step = 0.01

state = [current_xyz, target_xyz, [0,0,0], math.inf]


def detect_and_center():

    global target_xyz

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return None

    arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(arucoDict, parameters)

    markerLength = 0.05

    for _ in range(5):
        cap.grab()

    ret, frame = cap.read()

    if not ret:
        cap.release()
        return None

    h, w = frame.shape[:2]
    f = w

    cameraMatrix = np.array([[f,0,w/2],[0,f,h/2],[0,0,1]], dtype=np.float32)
    distCoeffs = np.zeros((5,1))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is None:
        cap.release()
        return None

    aruco.drawDetectedMarkers(frame, corners, ids)

    rvecs = []
    tvecs = []

    for corner in corners:

        objp = np.array([
            [-markerLength/2, markerLength/2, 0],
            [ markerLength/2, markerLength/2, 0],
            [ markerLength/2,-markerLength/2, 0],
            [-markerLength/2,-markerLength/2, 0]
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

    # center pixel
    cx = w/2
    cy = h/2

    tag_x = np.mean(corners[0][0][:,0])
    tag_y = np.mean(corners[0][0][:,1])

    error_x_pixels = tag_x - cx
    error_y_pixels = tag_y - cy

    error_x_m = (error_x_pixels * zC) / f
    error_y_m = (error_y_pixels * zC) / f

    # Directly use marker pose as target
    target_xyz = [xC, yC, zC]

    jointAngles = chain.inverse_kinematics(target_xyz)

    servo_angles_deg = [
        math.degrees(jointAngles[1]),
        math.degrees(jointAngles[2]),
        math.degrees(jointAngles[3])
    ]

    send_angles(servo_angles_deg)

    state = [current_xyz, target_xyz, servo_angles_deg, zC]

    cap.release()

    return state


def descend_z(distance):

    global target_xyz

    target_xyz[2] -= z_step

    jointAngles = chain.inverse_kinematics(target_xyz)

    servo_angles_deg = [
        math.degrees(jointAngles[1]),
        math.degrees(jointAngles[2]),
        math.degrees(jointAngles[3])
    ]

    send_angles(servo_angles_deg)


def grip():
    print("Gripper activated")
    ser.write(b"GRIP\n")


def send_angles(angles):

    msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"
    ser.write(msg.encode())
    print(f"Sent: {msg.strip()}")


def main():

    while True:

        result = detect_and_center()

        if result is None:
            continue

        vertical = result[3]

        if vertical < 0.1:
            break

        descend_z(vertical)

    grip()


if __name__ == "__main__":
    main()