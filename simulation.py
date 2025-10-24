import pybullet as p
import pybullet_data
import time       # needed for time.sleep
import math 
from ikpy.chain import Chain
from ikpy.link import URDFLink
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -9.8)
p.setTimeStep(1/240)

robot_id = p.loadURDF("3_DOF.urdf", [0,0,0], useFixedBase = True)

# Simulation loop

# 2. Get joint info and prints it
num_joints = p.getNumJoints(robot_id)
arm_chain = Chain.from_urdf_file("3_DOF.urdf")
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode()}")


# 4. Simulation loop
t = 0

p.resetDebugVisualizerCamera(
    cameraDistance=4,  # distance from target
    cameraYaw=50,        # rotation around vertical axis (degrees)
    cameraPitch=-30,     # up/down tilt (degrees)
    cameraTargetPosition=[0, 0, 0]  # what the camera points at

)
target_pos = [0.1, 0.1, 0.2]  # x, y, z in world coordinates
end_effector_index = num_joints -1 # last link
speed = 0.02  # radians per simulation step
current_positions = [0.0] * num_joints

while True:

    # Build the 4x4 target frame for IKPy
    target_frame = np.eye(4)
    target_frame[:3, 3] = target_pos  # set position, keep rotation identity

    # Compute joint angles using IKPy
    ik_angles = arm_chain.inverse_kinematics(target_frame)

    # Map IK angles to PyBullet joints (skip the fixed base)
    moving_joints = [joint.index for joint in arm_chain.active_links_mask if joint]  
# or manually: [0, 1, 2] if you know the first joint is base_yaw
    ik_angles_moving = ik_angles[1:4]  # skip fixed base

    # Smoothly move each joint
    for i, joint_index in enumerate(moving_joints):
        diff = ik_angles_moving[i] - current_positions[i]
        if abs(diff) > speed:
            current_positions[i] += speed if diff > 0 else -speed
        else:
            current_positions[i] = ik_angles_moving[i]

        p.resetJointState(robot_id, joint_index, current_positions[i])

    p.stepSimulation()
    time.sleep(1/240)
