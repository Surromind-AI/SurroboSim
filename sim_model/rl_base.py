from __future__ import division
import sys
sys.path.append('./')
from os import link
import sim as sim
import pybullet as p
import random
import numpy as np
import math
import time

from util import *
from ur5_move import urx_move

# Not load gripper
from sim_model import *

class RL_Base():
    def __init__(self):
        print('RL base Func')
        self._sim  = sim.PyBulletSim()
        self.dummy = None
        self.mount = None
        self.dummy_id  = None
        print('rl reset start')
        
        print('rl reset end')
        self.info_dict = None

        # 리워드 쓰레쉬홀드
        self._reward_epsilon = 0.01

        self.rl_reset()
    
    
    def get_state(self):
        # 현재 상태
        env = self._sim
        # ur5
        current_joint_state = [
                p.getJointState(env.robot_body_id, i)[0]
                for i in env._robot_joint_indices
            ]
        tool_tip_pose = p.getLinkState(env.robot_body_id,env.robot_tool_tip)[0:2]
        
        # dummy
        dummy_hole, dummy_center = self.dummy.pose_info()

        # mount
        mount_center, mount_tool_tip = self.mount.pose_info()

        info_dict = dict()
        # ur5
        info_dict['ur5_current_joint_state'] = current_joint_state
        info_dict['ur5_tool_tip_pose']       = tool_tip_pose
        # dummy
        info_dict['dummy_hole']              = dummy_hole
        info_dict['dummy_center']            = dummy_center
        # mount
        info_dict['mount_tool_tip']          = mount_tool_tip
        info_dict['mount_center']            = mount_center
        

        self.info_dict = info_dict
        return info_dict



        

        



    def get_reward(self):
        # 현재 상태에서의 reward

        reward_val = self.info_dict
        
        
        reward_pose= np.abs(np.array(reward_val['dummy_hole'][0]) - np.array(reward_val['mount_tool_tip'][0]))

        if all([np.abs(reward_val['dummy_hole'][0][i] - reward_val['mount_tool_tip'][0][i]) < self._reward_epsilon for i in range(len(reward_val['dummy_hole'][0])) ]):
            goal = True
        else:
            goal = False
        
        
        return goal, reward_pose

    
    def rl_step(self, ur5_joint):
        # action
        env = self._sim
        if len(ur5_joint) != 6:
            print(f'you get wornd 6DOF joint ur5_joint : {len(ur5_joint)}  ||  {ur5_joint}')
        env.move_joints(ur5_joint)

    def rl_reset(self, dummy_pose = None, dummy_quat = None, mount_pose = None):
        # 재설정
        env = self._sim
        self.dummy = dummy()
        self.mount = mount()

        # set default pose
        if dummy_pose == None:
            dummy_pose = [-1.1964399123833769, -0.791800028264185, 0.13686676473365827]
        if dummy_quat == None:
            dummy_quat = [-0.31538301706314087, -0.10541760921478271, 0.373131662607193, 0.8661370277404785]
        if mount_pose == None:
            mount_pose = [-0.09904367214900198, -0.5056011833545171, 1.1184632060025792]

        
        
        # env.move_joints(self.robot_home_joint_config, speed=1.0)
        
        self.dummy.obj_reset_pose(pose=dummy_pose, quat = dummy_quat)
        self.mount.obj_reset_tip(pose=mount_pose)
        self.dummy_id = self.dummy.return_id()
        # p.resetBaseVelocity(self.dummy_id)
        

        
        # set ur5 start pose
        env.robot_home_joint_config = [np.pi, -np.pi/2, 0, 0, 0, 0]
        env.move_joints(env.robot_home_joint_config)
        env.robot_home_joint_config = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        env.move_joints(env.robot_home_joint_config)


        # gripper open
        env.open_gripper()
        

        env.robot_home_joint_config = [0.5, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        env.move_joints(env.robot_home_joint_config)





        
        # start ur5 set
        
        ur_pose = [-1.162761600542553, -0.8290458752534293, 0.4]
        ur_quat = [0.8385665766060305, 0.34091009624327123, -0.18361610281863264, -0.3832382152974088]
        
        # target_joint_state = p.calculateInverseKinematics(env.robot_body_id,
        #                                                   env.robot_tool_tip,
        #                                                 #   self.tool0_fixed_joint_tool_tip_index,
        #                                                   ur_pose, ur_quat,
        #                                                   maxNumIterations=100, residualThreshold=1e-4)
        env.move_tool_shift(ur_pose, ur_quat)
        # env.move_joints(target_joint_state)
        concat_id = p.createConstraint(env.robot_body_id, env.robot_tool_tip, self.dummy_id, 0, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0,0,0], childFrameOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
        env.close_gripper()
        env.hold_gripper()


        ur_pose = [-1.162761600542553, -0.8290458752534293, 0.4]
        # target_joint_state = p.calculateInverseKinematics(env.robot_body_id,
        #                                                   env.robot_tool_tip,
        #                                                 #   self.tool0_fixed_joint_tool_tip_index,
        #                                                   ur_pose, ur_quat,
        #                                                   maxNumIterations=100, residualThreshold=1e-4)
        env.move_tool_shift(ur_pose, ur_quat)
        # env.move_joints(target_joint_state)

        # self.dummy.change_fix()
        

        


        
        
        ur_pose = [-1.162761600542553, -0.8290458752534293, 0.8]
        ur_quat = [0.8385665766060305, 0.34091009624327123, -0.18361610281863264, -0.3832382152974088]
        env.move_tool_shift(ur_pose, ur_quat)

        print('reset complete')
        

        



        
        

if __name__ == '__main__':
    

    RL = RL_Base()

    for idx in range(10):
        print(f'rl reset : {idx}')
        RL.rl_reset(dummy_pose = [0,0,0])

    print('end')
    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)
    


