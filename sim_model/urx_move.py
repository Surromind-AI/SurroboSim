from matplotlib.style import use
import numpy as np
import pybullet as p
import sim
from util import *
import time
from sim_model import *
import sys
sys.path.append('sim_model')
class ur5_move():
    """
    ur move sequence
    """
    def __init__(self, env):
        self.env = env
        self.time_data = None
        self.time_data_count = 0
        self.ur_time_data= None
    
    def shift_dummy_pose(self, time_data, x=0,y=0,z=0):
        for data in time_data:
            data[0][0] += x
            data[0][1] += y
            data[0][2] += z

        return time_data
    
    def _trajactory_info_reset(self):
        self.ur_time_data = []
        self.time_data_count = 0

    def _trajactory_info(self, dummy):
        tmp_data = dict()
            
        env = self.env
        current_joint_state = [
                p.getJointState(env.robot_body_id, i)[0]
                for i in env._robot_joint_indices
            ]
        tool_tip_pose = p.getLinkState(env.robot_body_id,env.robot_tool_tip)[0:2]

        
        ur_ik_joint = current_joint_state
        if env._ik_joint != None:
            ur_ik_joint = env._ik_joint


        tmp_data['idx']                 = self.time_data_count
        self.time_data_count += 1
        tmp_data['current_joint_state'] = list(current_joint_state)
        tmp_data['tool_tip_pose']       = list(tool_tip_pose)
        tmp_data['ur_ik_joint']         = list(ur_ik_joint)

        self.ur_time_data.append(tmp_data)

    
    def ur5_trajactory_pose(self, rand_xyz, scenario='SC2', ep = '1', obj_track = 'Trajectory_Dummy', reverse = False):
        x = rand_xyz[0]
        y = rand_xyz[1]
        z = rand_xyz[2]
        if scenario == 'SC1':
            sc_mount = 0
        elif scenario == 'SC2':
            sc_mount =-1
        # set start pose
        env = self.env
        # self.start_pose()
        # dummy_mesh = 'assets\dummy\dummy_tip.urdf'

        trajactory, _ = get_pose2(obj_track, SC=scenario, HD = 'HD', EP = ep)
        trajactory =  self.shift_dummy_pose(trajactory, x,y,z)
        # dummy = sim.dummy(pose,quat)
        print("start")
       

        # set mount
        mount = sim.mount()                     # mount load
        mount_pose = trajactory[sc_mount][0]    # get mount x y pose
        mount_pose_z = trajactory[sc_mount][0]    # get mount z pose

        mount_pose[2] = mount_pose_z[2]

        
        #### shift mount pose#######
        # y shift
        mount_pose[1] += 0.005
        # z shift
        mount_pose[2] -=0.005
        mount.obj_reset_tip(mount_pose)
        
        
        ur_infot_list = []

        # UR5 base bose
        robot_home_joint_config = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        env.move_joints(robot_home_joint_config, speed=1.0)
        trajactory = trajactory[300:]
        if reverse == True:
            trajactory.reverse()
            ur_strart_joint = [3.0687283501177918, -1.6517912191625128, 1.476245953335199, -1.5543621669103094, -0.8107936344286484, 4.912088813321593]
            env.move_joints(ur_strart_joint, speed=1.0)

        for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
        # for idx, trajac in enumerate(trajactory):
            
            tmp_time_data = dict()
            print(idx)
            pose = trajac[0]
            quat = trajac[1]

            current_joint_state = [
                p.getJointState(env.robot_body_id, i)[0]
                for i in env._robot_joint_indices
            ]
            tool_tip_pose = p.getLinkState(env.robot_body_id,env.robot_tool_tip)[0:2]
            
            if idx == 0:
                #set dummy
                dummy = sim.dummy(pose, useFix=False)
                # 매 useFix 가 False인 경우 step simulation 마다 계속 위치 업데이트 안그러면 떨어짐
                dummy.ur5_flag = True
                env._dummy_obj = dummy
                
                ur_pose = dummy.ur5_pose_quat[0]
                ur_quat = dummy.ur5_pose_quat[1]
                env.step_simulation(100)
                env.move_tool_shift(ur_pose, ur_quat)
                env.step_simulation(100)

                
                # dummy = sim.dummy(pose, useFix=False)
                # env.close_gripper()
                env.step_simulation(100)
                env.hold_gripper()
                env.step_simulation(100)
                self._trajactory_info_reset()
            # else:
            #     # input()
            #     d = SphereMarker(position = ur_pose)
            dummy.obj_reset_pose(pose, quat)
            # if idx>=750:
            #     input()
            pose_info, goal, reward_pose = env.get_pose_info(dummy = dummy, mount = mount)
            if goal == True:
                # 조건 다시 설정 이후에는 ur5와 더미 연결 제거
                dummy.ur5_flag = False
                # input()
                

                break

            # elif idx<300:
            #     continue

            # if idx >= 600:
            #     input()
                
            

            # if idx == 0:
            #     dummy = sim.dummy(pose,quat)
            #     env._dummy_id = dummy.return_id()

            # if idx%10 !=0:
            #     continue
            
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
            # dummy.obj_repose(pose, quat)
            
            
            
            self._trajactory_info(dummy)
            ur_pose = dummy.ur5_pose_quat[0]
            ur_quat = dummy.ur5_pose_quat[1]
            env.move_tool_shift(ur_pose, ur_quat)
            # # print(ur_ik_joint)

            # tmp_time_data['idx']                 = idx
            # tmp_time_data['current_joint_state'] = current_joint_state
            # tmp_time_data['tool_tip_pose']       = tool_tip_pose
            # tmp_time_data['ur_ik_joint']         = ur_ik_joint
            # ur_infot_list.append([idx, current_joint_state, tool_tip_pose])
            # ur_infot_list.append(tmp_time_data)
            

            # p.resetBasePositionAndOrientation(env._dummy_id, pose, quat)
            # time.sleep(0.01)
            
            
            
            
            

            # env.step_simulation(10)
        passed = 0
        # ur5 detach
        tool_tip_pose = p.getLinkState(env.robot_body_id, env.robot_tool_tip)[0:2]
        pose= list(tool_tip_pose[0])
        
        for i in range(30):
            
            relative_move = [0, -0.003, 0.003]
            
            tool_tip_pose = p.getLinkState(env.robot_body_id, env.robot_tool_tip)[0:2]
            pose= list(tool_tip_pose[0])
            print(pose)
            
            env.move_tool_shift_relative(relative_move)
            
            tool_tip_pose = p.getLinkState(env.robot_body_id, env.robot_tool_tip)[0:2]
            pose= list(tool_tip_pose[0])
            print(pose)
            self._trajactory_info(dummy)
            time.sleep(0.1)
        
        for _ in range(1000):
            p.stepSimulation()
            time.sleep(0.01)
            


        return  self.ur_time_data
    


