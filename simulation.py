import pybullet as p
import pybullet_data
import time
import numpy as np
from ikpy.chain import Chain

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setTimeStep(1 / 240)

robot_id = p.loadURDF("3_DOF.urdf", useFixedBase=True)
left_id = 5
right_id = 6

joint_name_to_idx = {
    p.getJointInfo(robot_id, i)[1].decode('utf-8'): i
    for i in range(p.getNumJoints(robot_id))
}

active_links_mask = [False, False, True, True, True, True, False]
arm_chain = Chain.from_urdf_file(
    "3_DOF.urdf",
    base_elements=["world"],
    active_links_mask=active_links_mask
)


target_pos = [1.0,1.0, 1.0]
#creates red dot where target_pos is positioned
p.createMultiBody(
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1]),
    basePosition=target_pos
)

ik_angles = arm_chain.inverse_kinematics(
    target_position=target_pos,
    initial_position=np.zeros(len(arm_chain.links))
)
revolute_angles = np.arctan2(
    np.sin(ik_angles[2:5]),
    np.cos(ik_angles[2:5])
)

print("\n=== IK RESULT ===")
for name, angle in zip(["base_yaw", "joint1", "joint2"], revolute_angles):
    print(f"{name:10s}: {angle: .4f} rad  ({np.degrees(angle): .2f}°)")

pb_joint_idx = [joint_name_to_idx[j] for j in ["base_yaw", "joint1", "joint2"]]
for pb_idx, angle in zip(pb_joint_idx, revolute_angles):
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=pb_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle

    )
    
# p.setJointMotorControl2(robot_id, left_id, p.POSITION_CONTROL, targetPosition=-0.4, force = 10)
# p.setJointMotorControl2(robot_id, right_id, p.POSITION_CONTROL, targetPosition=0.4, force = 10)

# p.setJointMotorControl2(robot_id, left_id, p.POSITION_CONTROL, targetPosition=0.4,force = 10)
# p.setJointMotorControl2(robot_id, right_id, p.POSITION_CONTROL, targetPosition=-0.4,force = 10)

ee_link_idx = joint_name_to_idx.get("ee_fixed")
tolerance = 0.05  # 1 cm

gripper_closed = False

for i in range(3600):
    p.stepSimulation()
    time.sleep(1/240)
    ee_pos = np.array(p.getLinkState(robot_id, ee_link_idx)[0])
    distance = np.linalg.norm(ee_pos - np.array(target_pos))

    # Close gripper when within tolerance
    if distance < tolerance:
        p.setJointMotorControl2(robot_id, left_id, p.POSITION_CONTROL, targetPosition=-0.4, force=50)
        p.setJointMotorControl2(robot_id, right_id, p.POSITION_CONTROL, targetPosition=-0.4, force=50)
        gripper_closed = True


full_ik = np.zeros(len(arm_chain.links))
full_ik[2:5] = revolute_angles
fk_pos = arm_chain.forward_kinematics(full_ik)[:3, 3]
print(f"\nFK (IKPy): {fk_pos}")
print(f"FK error : {np.linalg.norm(fk_pos - target_pos):.6f} m")
ee_link_idx = joint_name_to_idx.get("ee_fixed")
if ee_link_idx is not None:
    sim_pos = p.getLinkState(robot_id, ee_link_idx)[0]
    print(f"Sim EE pos: {sim_pos}")
    print(f"Sim error : {np.linalg.norm(np.array(sim_pos) - target_pos):.6f} m")

while True:
    p.stepSimulation()
    time.sleep(1 / 240)