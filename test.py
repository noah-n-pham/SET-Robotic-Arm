import pybullet as p
import pybullet_data
import time
import numpy as np
from ikpy.chain import Chain
import imageio


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setTimeStep(1 / 240)


robot_id = p.loadURDF("3_DOF.urdf", useFixedBase=True)


snapshot_taken = False
pos_tol = 1e-3   # position tolerance
orn_tol = 1e-2   # orientation tolerance


# Create a small yellow cube to represent the camera
camera_cube_visual = p.createVisualShape(
   p.GEOM_BOX,
   halfExtents=[0.03, 0.03, 0.03],  # 2 cm cube
   rgbaColor=[1, 1, 0, 1]           # yellow
)
camera_cube = p.createMultiBody(
   baseMass=0,
   baseVisualShapeIndex=camera_cube_visual,
   basePosition=[0, 0, 0]
)


# aruco tag cube
# Base white cube
cube_visual = p.createVisualShape(
   p.GEOM_BOX,
   halfExtents=[0.05, 0.05, 0.05],
   rgbaColor=[1, 1, 1, 1]
)
cube = p.createMultiBody(
   baseMass=0,
   baseVisualShapeIndex=cube_visual,
   basePosition=[0.5, -0.7, 0]
)


# Load texture
aruco_texture = p.loadTexture("aruco_tag_mesh.png")


# Create a thin "sticker" plane just in front of +X face
half = 0.05  # same half size as cube
aruco_face = p.createVisualShape(
   p.GEOM_BOX,
   halfExtents=[half, half, 0.0005],  # very thin
   rgbaColor=[1, 1, 1, 1]
)


# Position sticker slightly in front of +Y face (avoid z-fighting)
pos_offset = [0, 0, half + 0.002]
aruco_face_id = p.createMultiBody(
   baseMass=0,
   baseVisualShapeIndex=aruco_face,
   basePosition=[0.5 + pos_offset[0], -0.7 + pos_offset[1], 0 + pos_offset[2]],
   baseOrientation=p.getQuaternionFromEuler([0, 0, 0]) # np.pi/2
)


# Apply the texture to that plane
p.changeVisualShape(aruco_face_id, -1, textureUniqueId=aruco_texture)


test_visual = p.createVisualShape(
   p.GEOM_BOX,
   halfExtents=[0.2, 0.2, 0.2],
   rgbaColor=[1, 1, 1, 1]
)


# 90° rotation around Z axis
rotation_quat = p.getQuaternionFromEuler([0, 0, np.pi]) # np.pi np.pi/2


test_cube = p.createMultiBody(
   baseMass=0,
   baseVisualShapeIndex=test_visual,
   basePosition=[0, 0, 1],
   baseOrientation=rotation_quat
)


p.changeVisualShape(test_cube, -1, textureUniqueId=p.loadTexture("aruco_tag_mesh.png"))


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


target_pos = [0.2, -0.35, 0.4]
p.createMultiBody(
   baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1]),
   basePosition=target_pos
)


ik_angles = arm_chain.inverse_kinematics(
   target_position=target_pos,
   initial_position=np.zeros(len(arm_chain.links))
)
revolute_angles = np.arctan2(np.sin(ik_angles[2:5]), np.cos(ik_angles[2:5]))


print("\n=== IK RESULT ===")
for name, angle in zip(["base_yaw", "joint1", "joint2"], revolute_angles):
   print(f"{name:10s}: {angle: .4f} rad  ({np.degrees(angle): .2f}°)")


pb_joint_idx = [joint_name_to_idx[j] for j in ["base_yaw", "joint1", "joint2"]]


# --- Reset joints before simulation starts ---
for pb_idx, angle in zip(pb_joint_idx, revolute_angles):
   p.resetJointState(robot_id, pb_idx, angle)


# Step once so visuals update immediately
p.stepSimulation()


# --- Then set motors to hold that position ---
for pb_idx, angle in zip(pb_joint_idx, revolute_angles):
   p.setJointMotorControl2(
       bodyUniqueId=robot_id,
       jointIndex=pb_idx,
       controlMode=p.POSITION_CONTROL,
       targetPosition=angle,
       force=500
   )


ee_link_idx = joint_name_to_idx.get("ee_fixed")
cam_offset = [-0.1, 0, -0.1]  # camera 5 cm in front of EE # +x made cam go down, +z made cam go forward
fov = 60
img_width = 320
img_height = 240
near = 0.01
far = 1.5


while True:
   p.stepSimulation()


   # Get EE pose
   if ee_link_idx is not None:
       ee_state = p.getLinkState(robot_id, ee_link_idx)
       ee_pos, ee_orn = ee_state[0], ee_state[1]
       cam_pos, cam_orn = p.multiplyTransforms(ee_pos, ee_orn, cam_offset, [0, 0, 0, 1])
       cam_mat = p.getMatrixFromQuaternion(cam_orn)
       # Camera forward vector
       #forward = [cam_mat[0], cam_mat[3], cam_mat[6]] # local +X
       forward = [cam_mat[2], cam_mat[5], cam_mat[8]]  # local +Z


       cam_target = [cam_pos[0]+forward[0], cam_pos[1]+forward[1], cam_pos[2]+forward[2]]
       p.resetBasePositionAndOrientation(camera_cube, cam_pos, [0, 0, 0, 1])


       view_matrix = p.computeViewMatrix(cam_pos, cam_target, [0, 0, 1])
       proj_matrix = p.computeProjectionMatrixFOV(fov, img_width/img_height, near, far)


       # Get camera image
       img_arr = p.getCameraImage(img_width, img_height, view_matrix, proj_matrix)
       rgb = img_arr[2]
      
       if not snapshot_taken:


           # Check if EE is close enough to the target
           dist = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))


           if dist < pos_tol:
               rgb_np = np.reshape(rgb, (img_height, img_width, 4))[:, :, :3]
               imageio.imwrite("camerasnap.png", rgb_np)
               print("📸 Saved camerasnap.png")
               snapshot_taken = True


   time.sleep(0.5)