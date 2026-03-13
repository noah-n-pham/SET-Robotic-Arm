import time
import sys
import numpy as np
import cv2
from ikpy.chain import Chain
import serial
import serial.tools.list_ports
import cv2.aruco as aruco
import math

# -----------------------------
# Robot / IK Setup
# -----------------------------
chain = Chain.from_urdf_file(
    "3_DOF.urdf",
    active_links_mask=[False, True, True, True, False]
)
print(f"Chain has {len(chain.links)} links: {[link.name for link in chain.links]}")

# -----------------------------
# Serial Setup (auto-detect Arduino)
# -----------------------------
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        dev = port.device.lower()
        if any(kw in desc for kw in ["arduino", "ch340", "ch341", "cp210", "ftdi", "usb serial", "usb-serial"]):
            return port.device
        if any(kw in dev for kw in ["usbmodem", "usbserial", "ttyacm", "ttyusb"]):
            return port.device
    if len(ports) == 1:
        return ports[0].device
    return None

PORT = find_arduino_port()
if PORT is None:
    print("ERROR: No Arduino found. Available serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}: {p.description}")
    sys.exit(1)

print(f"Connecting to Arduino on {PORT}")
BAUD = 9600
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

# -----------------------------
# Camera Setup
# -----------------------------
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Camera failed to open.")
    ser.close()
    sys.exit(1)

# -----------------------------
# Control Parameters
# -----------------------------
target_xyz = [0.1, 0, 0.15]
z_step = 0.01

last_xyz = [0.0, 0.0, 0.0]
alpha = 0.7

# -----------------------------
# ArUco Setup
# -----------------------------
arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(arucoDict, parameters)

markerLength = 0.05


# -----------------------------
# Send Servo Angles (clamped to 0-270, sent as integers)
# -----------------------------
def send_angles(angles):
    clamped = [max(0, min(270, int(round(a)))) for a in angles]
    msg = f"{clamped[0]},{clamped[1]},{clamped[2]}\n"
    ser.write(msg.encode())
    ser.flush()
    print(f"Sent: {msg.strip()}")


# -----------------------------
# Compute IK and return servo angles in degrees
# Chain: [0]=origin, [1]=base_yaw, [2]=joint1, [3]=joint2, [4]=ee_fixed
# IK outputs 0° as neutral; physical servos have 135° as center
# -----------------------------
SERVO_CENTER = 135

def compute_servo_angles(target):
    joint_angles = chain.inverse_kinematics(target)
    return [
        math.degrees(joint_angles[1]) + SERVO_CENTER,
        math.degrees(joint_angles[2]) + SERVO_CENTER,
        math.degrees(joint_angles[3]) + SERVO_CENTER
    ]


# -----------------------------
# Detect Marker + Move Robot
# -----------------------------
def detect_and_center():
    global target_xyz, last_xyz

    ret, frame = cap.read()
    if not ret:
        return None

    h, w = frame.shape[:2]
    f = w

    cameraMatrix = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1]], dtype=np.float32)
    distCoeffs = np.zeros((5, 1))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is None:
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return "quit"
        return None

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
            objp, corner, cameraMatrix, distCoeffs
        )
        rvecs.append(rvec)
        tvecs.append(tvec)

    for i in range(len(ids)):
        cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs,
                          rvecs[i], tvecs[i], 0.03)

    xC, yC, zC = tvecs[0].flatten()

    # Low-pass filter for stability
    xF = alpha * last_xyz[0] + (1 - alpha) * xC
    yF = alpha * last_xyz[1] + (1 - alpha) * yC
    zF = alpha * last_xyz[2] + (1 - alpha) * zC

    last_xyz = [xF, yF, zF]
    target_xyz = [xF, yF, zF]

    servo_angles_deg = compute_servo_angles(target_xyz)
    send_angles(servo_angles_deg)

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return "quit"

    return [target_xyz, servo_angles_deg, zF]


# -----------------------------
# Descend Robot toward target
# -----------------------------
def descend_z():
    global target_xyz

    target_xyz[2] -= z_step
    servo_angles_deg = compute_servo_angles(target_xyz)
    send_angles(servo_angles_deg)


# -----------------------------
# Activate Gripper
# -----------------------------
def grip():
    print("Gripper activated")
    ser.write(b"GRIP\n")
    ser.flush()


# -----------------------------
# Main Control Loop
# -----------------------------
def main():
    print("Starting robotic arm control...")

    try:
        while True:
            result = detect_and_center()

            if result == "quit":
                break

            if result is None:
                continue

            zF = result[2]

            if zF < 0.1:
                break

            descend_z()

        grip()

    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        print("Cleanup complete.")


# -----------------------------
# Run Program
# -----------------------------
if __name__ == "__main__":
    main()
