import time
import numpy as np
import cv2
from ikpy.chain import Chain
import serial
import cv2.aruco as aruco
import math

# -----------------------------
# Robot / IK Setup
# -----------------------------
chain = Chain.from_urdf_file("3_DOF.urdf")

# -----------------------------
# Serial Setup
# -----------------------------
PORT = "COM3"
BAUD = 9600
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# -----------------------------
# Camera Setup
# -----------------------------
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("Camera failed to open.")
    exit()

# -----------------------------
# Control Parameters
# -----------------------------
current_xyz = [0, 0, 0]
target_xyz = [0.1, 0, 0.15]

Kp = 0.2
center_threshold = 0.005
z_step = 0.01

# pose filtering
last_xyz = [0, 0, 0]
alpha = 0.7

# state
state = [current_xyz, target_xyz, [0,0,0], math.inf]

# -----------------------------
# ArUco Setup
# -----------------------------
arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

markerLength = 0.05


# -----------------------------
# Send Servo Angles
# -----------------------------
def send_angles(angles):

    msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"
    ser.write(msg.encode())
    print(f"Sent: {msg.strip()}")


# -----------------------------
# Detect Marker + Move Robot
# -----------------------------
def detect_and_center():

    global target_xyz
    global last_xyz

    ret, frame = cap.read()

    if not ret:
        return None

    h, w = frame.shape[:2]
    f = w

    cameraMatrix = np.array([[f,0,w/2],[0,f,h/2],[0,0,1]], dtype=np.float32)
    distCoeffs = np.zeros((5,1))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters=parameters)

    if ids is None:

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()

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

    # -----------------------------
    # Low-pass filter (stability)
    # -----------------------------
    xF = alpha*last_xyz[0] + (1-alpha)*xC
    yF = alpha*last_xyz[1] + (1-alpha)*yC
    zF = alpha*last_xyz[2] + (1-alpha)*zC

    last_xyz = [xF, yF, zF]

    target_xyz = [xF, yF, zF]

    # -----------------------------
    # Inverse Kinematics
    # -----------------------------
    jointAngles = chain.inverse_kinematics(target_xyz)

    servo_angles_deg = [
        math.degrees(jointAngles[1]),
        math.degrees(jointAngles[2]),
        math.degrees(jointAngles[3])
    ]

    send_angles(servo_angles_deg)

    state = [current_xyz, target_xyz, servo_angles_deg, zF]

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

    return state


# -----------------------------
# Descend Robot
# -----------------------------
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


# -----------------------------
# Activate Gripper
# -----------------------------
def grip():

    print("Gripper activated")
    ser.write(b"GRIP\n")


# -----------------------------
# Main Control Loop
# -----------------------------
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

    cap.release()
    cv2.destroyAllWindows()


# -----------------------------
# Run Program
# -----------------------------
if __name__ == "__main__":
    main()