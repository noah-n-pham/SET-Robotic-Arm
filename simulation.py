import pybullet as p
import pybullet_data
import time
import numpy as np
import imageio
import cv2
from ikpy.chain import Chain

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setTimeStep(1 / 240)

robot_id = p.loadURDF("3_DOF.urdf", useFixedBase=True)

# test cube
cube_start_pos = [0.6,-0.6, -0.2]   # x, y, z coordinates
cube_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
cube_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.05],
    rgbaColor=[0, 0, 1, 1]  # blue
)

cube_body = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=cube_collision,
    baseVisualShapeIndex=cube_visual,
    basePosition=cube_start_pos,
    baseOrientation=cube_start_orientation
)

# aruco cube
aruco_texture = p.loadTexture("aruco_tag.png")
aruco_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.05],
    rgbaColor=[1, 1, 1, 1]
)
aruco_cube = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=aruco_visual,
    basePosition=[0.5, 1.0, 0.0]
)
p.changeVisualShape(aruco_cube, -1, textureUniqueId=aruco_texture)


# Create a small yellow cube to represent the camera
camera_cube_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.01, 0.01, 0.01],  # 2 cm cube
    rgbaColor=[1, 1, 0, 1]           # yellow
)
camera_cube = p.createMultiBody(
    baseMass=0,
    baseVisualShapeIndex=camera_cube_visual,
    basePosition=[0, 0, 0]
)

joint_name_to_idx = {
    p.getJointInfo(robot_id, i)[1].decode('utf-8'): i
    for i in range(p.getNumJoints(robot_id))
}

active_links_mask = [False, False, True, True, True, False]
arm_chain = Chain.from_urdf_file(
    "3_DOF.urdf",
    base_elements=["world"],
    active_links_mask=active_links_mask
)

target_pos = [0.8,-0.5, 0.0]
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



for i in range(2000):
    p.stepSimulation()
    time.sleep(1/240)

    # newly added camera
    link_state = p.getLinkState(robot_id, joint_name_to_idx["ee_fixed"])
    ee_pos, ee_orn = link_state[0], link_state[1]
    rot_matrix = np.array(p.getMatrixFromQuaternion(ee_orn)).reshape(3, 3)
    camera_eye = ee_pos - 0.00 * rot_matrix[:, 0]- 0.1 * rot_matrix[:, 2]
    camera_target = ee_pos - 0.1 * rot_matrix[:, 0]
    camera_up = rot_matrix[:, 2]

    p.resetBasePositionAndOrientation(camera_cube, camera_eye, [0, 0, 0, 1])

    if i % 10 == 0:  # snapshot every 10 steps
        view_matrix = p.computeViewMatrix(camera_eye, camera_target, camera_up)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=0.01, farVal=5.0)

        width, height, rgb, _,_ = p.getCameraImage(
            width=320,
            height=240,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix
        )
        rgb_array = np.reshape(rgb, (height, width, 4))[:, :, :3]
        # imageio.imwrite(f"camerasnap{i:04d}.png", rgb_array)
        # print(f"📸 Saved camerasnap{i:04d}.png")
        pos, orn = p.getBasePositionAndOrientation(robot_id)
        ee_to_cam = [[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, -0.1],
                    [0, 0, 0, 1]]
        #get world to base matrix
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
        translation = np.array(pos).reshape(3,1)
        world_to_base = np.eye(4)
        world_to_base[:3,:3] = rot
        world_to_base[:3, 3] = translation.flatten()
        # get world to end effector matrix
        state = p.getLinkState(robot_id, 4, computeForwardKinematics=True)
        pos = state[4]          # world position (x, y, z)
        orn = state[5]          # world orientation quaternion (x, y, z, w)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        world_to_ee = np.eye(4)
        world_to_ee[:3, :3] = rot_matrix
        world_to_ee[:3, 3] = pos

        ee_to_base = np.linalg.inv(world_to_base) @ world_to_ee
        rot
        cam_pos, cam_quat = p.getBasePositionAndOrientation(camera_cube)
        rot = np.array(p.getMatrixFromQuaternion(cam_quat)).reshape(3,3)
        translation = np.array(cam_pos).reshape(3,1)
        cam_to_world = np.eye(4)
        cam_to_world[:3,:3] = rot
        cam_to_world[:3, 3] = translation.flatten()

        obj_pos, obj_quat = p.getBasePositionAndOrientation(aruco_cube)
        rot = np.array(p.getMatrixFromQuaternion(obj_quat)).reshape(3,3)
        translation = np.array(obj_pos).reshape(3,1)
        world_to_obj = np.eye(4)
        world_to_obj[:3,:3] = rot
        world_to_obj[:3, 3] = translation.flatten()

        cam_to_obj = np.linalg.inv(cam_to_world) @ world_to_obj
        matrix = np.matmul(ee_to_cam, ee_to_base)
        matrix = np.matmul(matrix, cam_to_obj)

        target_pos = matrix[:3, 3]
        ik_solution = arm_chain.inverse_kinematics(target_pos)
        print(ik_solution)

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

# math for camera to object
# pos, orn = p.getBasePositionAndOrientation(robot_id)
# ee_to_cam = [[1, 0, 0, 0],
#              [0, 1, 0, 0],
#              [0, 0, 1, -0.1],
#              [0, 0, 0, 1]]
# #get world to base matrix
# rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3,3)
# translation = np.array(pos).reshape(3,1)
# world_to_base = np.eye(4)
# world_to_base[:3,:3] = rot
# world_to_base[:3, 3] = translation
# # get world to end effector matrix
# state = p.getLinkState(robot_id, 4, computeForwardKinematics=True)
# pos = state[4]          # world position (x, y, z)
# orn = state[5]          # world orientation quaternion (x, y, z, w)
# rot_matrix = p.getMatrixFromQuaternion(orn)
# rot_matrix = np.array(rot_matrix).reshape(3, 3)
# world_to_ee = np.eye(4)
# world_to_ee[:3, :3] = rot_matrix
# world_to_ee[:3, 3] = pos

# ee_to_base = np.linalg.inv(world_to_base) @ world_to_ee
# rot
# cam_pos, cam_quat = p.getBasePositionAndOrientation(camera_cube)
# rot = np.array(p.getMatrixFromQuaternion(cam_quat)).reshape(3,3)
# translation = np.array(cam_pos).reshape(3,1)
# cam_to_world = np.eye(4)
# cam_to_world[:3,:3] = rot
# cam_to_world[:3, 3] = translation

# obj_pos, obj_quat = p.getBasePositionAndOrientation(aruco_cube)
# rot = np.array(p.getMatrixFromQuaternion(obj_quat)).reshape(3,3)
# translation = np.array(obj_pos).reshape(3,1)
# world_to_obj = np.eye(4)
# world_to_obj[:3,:3] = rot
# world_to_obj[:3, 3] = translation

# cam_to_obj = np.linalg.inv(cam_to_world) @ world_to_obj
# matrix = np.matmul(ee_to_cam, ee_to_base)
# matrix = np.matmul(matrix, cam_to_obj)

# ik_solution = arm_chain.inverse_kinematics(matrix)
# print("hi")
# print(ik_solution)
# while True:
#     p.stepSimulation()
#     time.sleep(1 / 240)