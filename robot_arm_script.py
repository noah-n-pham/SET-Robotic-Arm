from simulation2 import camera
from simulation2 import send_angles
import cv2
import time
import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)


# --- Serial Setup ---
PORT = "COM5"  # windows ver
BAUD = 9600  # same speed as Ro
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset

def main():
    wait = False
    while True:
        #if wait: 
            #if serial sent/received 
                #wait = false
            #else
                #continue

        cap = cv2.VideoCapture(2)  # 0 = default webcam
        if not cap.isOpened():
            print("Error: Could not open video stream.")
            exit()

        success, frame = cap.read()
        if not success:
            continue
        cv2.imshow("Robot Arm Camera Feed", frame)
        cap.release()
        cv2.destroyAllWindows()
        
        #process frame to detect object (Aruco)
        send_angles(angles)
        wait = True
        

    
