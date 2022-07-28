from __future__ import division
from os import link
import sim
import pybullet as p
import random
import numpy as np
import math
from tqdm import tqdm

from util import *
from ur5_move import urx_move

def test_pose(num_trials = 1):
    passed = 0
    for i in range(num_trials):
        # Choose a reachable end-effector position and orientation
        random_position = env._workspace1_bounds[:, 0] + 0.15 + \
            np.random.random_sample((3)) * (env._workspace1_bounds[:, 1] - env._workspace1_bounds[:, 0] - 0.15)
        random_orientation = np.random.random_sample((3)) * np.pi / 4 - np.pi / 8
        random_orientation[1] += np.pi
        random_orientation = p.getQuaternionFromEuler(random_orientation)
        marker = sim.SphereMarker(position=random_position, radius=0.03, orientation=random_orientation)
        # Move tool
        env.move_tool(random_position, random_orientation)
        link_state = p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)
        link_marker = sim.SphereMarker(link_state[0], radius=0.03, orientation=link_state[1], rgba_color=[0, 1, 0, 0.8])
        # Test position
        delta_pos = np.max(np.abs(np.array(link_state[0]) - random_position))
        delta_orn = np.max(np.abs(np.array(link_state[1]) - random_orientation))
        if  delta_pos <= 1e-3 and delta_orn <= 1e-3:
            passed += 1
        env.step_simulation(100)
        # Return to robot's home configuration
        env.robot_go_home()
        del marker, link_marker
    print(f"[Robot Movement] {passed} / {num_trials} cases passed")

def test_trajactory():
    trajactory, _ = get_pose('Trajectory_Dummy')
    for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
        # print(idx)
        if idx%10 !=0:
            continue
        pose = trajac[0]
        quat = trajac[1]
        
        

        marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
        # print(pose)
        # env.move_tool(pose, quat)

        env.step_simulation(1)
    passed = 0
def show_trajactory(num_trials = 1):
    trajactory = get_bson_data()
    for trajac in trajactory:
        pose = [trajac['hand'][0]['loc']['x'], trajac['hand'][0]['loc']['y'], trajac['hand'][0]['loc']['z']]
        quat = [trajac['hand'][0]['quat']['x'], trajac['hand'][0]['quat']['y'], trajac['hand'][0]['quat']['z'], trajac['hand'][0]['quat']['w']]

        marker = sim.SphereMarker(position=pose, radius=0.03, orientation=quat)

    
    # for i in range(num_trials):
    #     # Choose a reachable end-effector position and orientation
    #     random_position = env._workspace1_bounds[:, 0] + 0.15 + \
    #         np.random.random_sample((3)) * (env._workspace1_bounds[:, 1] - env._workspace1_bounds[:, 0] - 0.15)
    #     random_orientation = np.random.random_sample((3)) * np.pi / 4 - np.pi / 8
    #     random_orientation[1] += np.pi
    #     random_orientation = p.getQuaternionFromEuler(random_orientation)
    #     marker = sim.SphereMarker(position=random_position, radius=0.03, orientation=random_orientation)
    #     # Move tool
    #     env.move_tool(random_position, random_orientation)
    #     link_state = p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)
    #     link_marker = sim.SphereMarker(link_state[0], radius=0.03, orientation=link_state[1], rgba_color=[0, 1, 0, 0.8])
    #     # Test position
    #     delta_pos = np.max(np.abs(np.array(link_state[0]) - random_position))
    #     delta_orn = np.max(np.abs(np.array(link_state[1]) - random_orientation))
    #     if  delta_pos <= 1e-3 and delta_orn <= 1e-3:
    #         passed += 1
    #     env.step_simulation(1000)
    #     # Return to robot's home configuration
    #     env.robot_go_home()
    #     del marker, link_marker
    # print(f"[Robot Movement] {passed} / {num_trials} cases passed")


def test_Grasping(num_trials=3):
    passed = 0
    # env.load_gripper()
    for _ in range(num_trials):
        # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
        object_id = env._objects_body_ids[0]        
        position, grasp_angle = get_grasp_position_angle(object_id)
        grasp_success = env.execute_grasp(position, grasp_angle)

        # test stop motion not falling object
        env.step_simulation(100, sleep_time=0.001, object_ori = object_id)

        # Test for grasping success (this test is a necessary condition, not sufficient):
        object_z = p.getBasePositionAndOrientation(object_id)[0][2]
        if object_z >= 0.2:
            passed += 1
        # env.reset_objects()
        env.set_objects()
    print(f"[Grasping] {passed} / {num_trials} cases passed")

def test_Grasping2(num_trials=3):
    """
    위치 조정 및 그리퍼 잡는 순간 고정 놓는 순간 고정 해제
    """
    passed = 0
    # env.load_gripper()
    for _ in range(num_trials):
        # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
        object_id = env._objects_body_ids[0]        
        position, grasp_angle = get_grasp_position_angle(object_id)

        #shift pose
        position = list(position)
        position[1] = position[1]+0.01
        
        grasp_success = env.execute_grasp(position, grasp_angle)

        # Test for grasping success (this test is a necessary condition, not sufficient):
        object_z = p.getBasePositionAndOrientation(object_id)[0][2]
        if object_z >= 0.2:
            passed += 1
        env.set_objects()
    print(f"[Grasping] {passed} / {num_trials} cases passed")


def get_grasp_position_angle(object_id):
    position, grasp_angle = np.zeros((3, 1)), 0
    # ========= PART 2============
    # Get position and orientation (yaw in radians) of the gripper for grasping
    # ==================================
    position, orientation = p.getBasePositionAndOrientation(object_id)
    grasp_angle = p.getEulerFromQuaternion(orientation)[2]
    return position, grasp_angle

if __name__ == '__main__':
    random.seed(1)
    obj_pose_list, obj_mesh = get_pose('HangingDummyLong')
    object_shapes = [
        # "assets/objects/cube.urdf",
        # "assets\\objects\\rod.urdf"
        "assets\dummy\dummy_test.urdf"

    ]
    # object_shapes = None
    env = sim.PyBulletSim(object_shapes = object_shapes)
    # load gripper
    env.load_gripper()

    u = urx_move(env)

    #Test UR Move ment
    u.show_trajactory()

    # Test trajactory
    # show_trajactory()
    # test_trajactory()

    # Load Gripper && Grasping
    # test_Grasping(100)

    # test grasping concat dummy
    # test_Grasping2(3)





    while True:
        pass
