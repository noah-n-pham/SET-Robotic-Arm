import pybullet as p
import pybullet_data
import time
import numpy as np
import imageio
import cv2
from ikpy.chain import Chain


# --- Serial Setup ---
PORT = "COM3"  # windows ver
BAUD = 9600  # same speed as Ro
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # allow Arduino reset

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setTimeStep(1 / 240)


robot_id = p.loadURDF("3_DOF.urdf", useFixedBase=True)

chain = Chain.from_urdf_file("3_DOF.urdf")

def send_angles_from_pos(targetPosition):
    # targetPosition is a list of [x, y, z] coordinates
    jointAngles = chain.inverse_kinematics(targetPosition)
    servo_angles_deg = [math.degrees(a) for a in jointAngles[1:]]
    msg = f"{servo_angles_deg[1]:.2f},{servo_angles_deg[2]:.2f},{servo_angles_deg[3]:.2f}\n"  # format: 45.00,30.00\n
    ser.write(msg.encode())
    print(f"Sent: {msg.strip()}")

def send_angles_from_pos(angles):
    msg = f"{angles[0]:.2f},{angles[1]:.2f},{angles[2]:.2f}\n"  # format: 45.00,30.00\n
    ser.write(msg.encode())
    print(f"Sent: {msg.strip()}")

send_angles([0, 0, 0])  # initial position

ser.close()
print("Done!")  # done wahoo



active_links_mask = [False, False, True, True, True, False]
arm_chain = Chain.from_urdf_file(
   "3_DOF.urdf",
   base_elements=["world"],
   active_links_mask=active_links_mask
)


# target_pos = [0.5,-0.5, 0.0]
# p.createMultiBody(
#     baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1]),
#     basePosition=target_pos
# )


p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, targetPosition = 4.23) #
p.setJointMotorControl2(robot_id, 2, p.POSITION_CONTROL, targetPosition = 0.6) # angle of base to first arm
p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, targetPosition = 0.6) # angle of first to second arm
num = p.getNumJoints(robot_id)
for i in range(num):  
   info = p.getJointInfo(robot_id, i)
   print(i, info[1].decode())




for i in range(2000):
   p.stepSimulation()
   time.sleep(1/240)


#0 world_to_base
#1 base_yaw
#2 joint1
#3 joint2
#4 ee_fixed
