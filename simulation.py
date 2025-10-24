import pybullet as p
import pybullet_data
import time       # needed for time.sleep
import math 

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -9.8)
p.setTimeStep(1/240)

robot_id = p.loadURDF("/Users/timothywang/Downloads/UF/SASE/SET - 2025-26/SET-Robotic-Arm/3_DOF.urdf", [0,0,0], useFixedBase = True)

# Simulation loop

# 2. Get joint info and prints it
num_joints = p.getNumJoints(robot_id)
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

sliders = [p.addUserDebugParameter(f"joint{i}", -3.14, 3.14, 0) for i in range(num_joints)]

while True:
    # Read slider values
    joint_positions = [p.readUserDebugParameter(sliders[i]) for i in range(num_joints)]
    
    # Move joints directly without swaying
    # for i in range(num_joints):
    #     p.setJointMotorControl2(
    #         robot_id,
    #         i,
    #         controlMode=p.POSITION_CONTROL,
    #         targetPosition=joint_positions[i],
    #         force=500,
    #         positionGain=1.0,
    #         velocityGain=1.0
    #     )
    for i in range(num_joints):
        p.resetJointState(robot_id, i, joint_positions[i])

    p.stepSimulation()
    time.sleep(1/240)

# while True:
    # Example: simple sinusoidal motion form all 3 joints
    # joint_positions = [
        # 0.5 * math.sin(t),
        # 0.5 * math.sin(t + math.pi/4)
    #     0.5,
    #     0.5
       
    # ]
    
    # for i in range(num_joints):
    #     p.setJointMotorControl2(
    #         bodyIndex=robot_id,
    #         jointIndex=i,
    #         controlMode=p.POSITION_CONTROL,
    #         targetPosition=joint_positions[i],
    #         force=100
    #     )
    
    # p.stepSimulation()
    # time.sleep(1/240)
    # t += 0.01