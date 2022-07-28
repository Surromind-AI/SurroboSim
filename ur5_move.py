from matplotlib.style import use
import numpy as np
import pybullet as p
import sim
from util import *
import time
from sim_model import *


class urx_move():
    """
    ur move sequence
    """
    def __init__(self, env):
        self.env = env
    
    def test_pose(self, num_trials = 3):
        env = self.env
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
            # env.move_tool(random_position, random_orientation)
            env.move_tool_shift(random_position, random_orientation)

            # link_state = p.getLinkState(env.robot_body_id, env.robot_end_effector_link_index)
            link_state = p.getLinkState(env.robot_body_id, env.robot_tool_tip)
            link_marker = sim.SphereMarker(link_state[0], radius=0.01, orientation=link_state[1], rgba_color=[0, 1, 0, 0.3])
            # Test position
            delta_pos = np.max(np.abs(np.array(link_state[0]) - random_position))
            delta_orn = np.max(np.abs(np.array(link_state[1]) - random_orientation))
            if  delta_pos <= 1e-3 and delta_orn <= 1e-3:
                passed += 1
            env.step_simulation(1000)
            # Return to robot's home configuration
            env.robot_go_home()
            del marker, link_marker
        print(f"[Robot Movement] {passed} / {num_trials} cases passed")
    
    def test_Grasping(self,num_trials=3, select_object = 0):
        """
        받은 object shape 를 잡아서 들어 올리는 테스트
        """
        env = self.env
        passed = 0
        
        for _ in range(num_trials):
            # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
            object_id = env._objects_body_ids[select_object]        
            position, grasp_angle = self.get_grasp_position_angle(object_id)

            #shift pose
            # position = list(position)
            # position[1] = position[1]+0.01

            grasp_success = env.execute_grasp(position, grasp_angle)

            # Test for grasping success (this test is a necessary condition, not sufficient):
            object_z = p.getBasePositionAndOrientation(object_id)[0][2]
            if object_z >= 0.2:
                passed += 1
            env.reset_objects()
        print(f"[Grasping] {passed} / {num_trials} cases passed")

    def test_Grasping_concat(self,num_trials=3, select_object = 0):
        """
        받은 object shape 를 잡아서 들어 올리는 테스트
        """
        env = self.env
        passed = 0
        env.reset_objects()
        
        for _ in range(num_trials):
            # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
            object_id = env._objects_body_ids[select_object]        
            position, grasp_angle = self.get_grasp_position_angle(object_id)

            #shift pose
            # position = list(position)
            # position[1] = position[1]+0.01

            grasp_success = env.execute_grasp_concat(position, grasp_angle, object_id)
            # grasp_success = env.execute_grasp_concat2(position, grasp_angle, object_id)

            # Test for grasping success (this test is a necessary condition, not sufficient):
            object_z = p.getBasePositionAndOrientation(object_id)[0][2]
            if object_z >= 0.2:
                passed += 1
            env.reset_objects()
        print(f"[Grasping] {passed} / {num_trials} cases passed")
    
    def ur5_move_dummy(self, num_trials=3):
        env = self.env
        dummy_id = env._tracking_dumy_id
        position, quat = p.getLinkState(dummy_id,2)[0:2]
        
        p.removeBody(dummy_id)
        # self.get_grasp_position_angle(dummy_id)]
        for _ in range(num_trials):
            # position, quat = p.getBasePositionAndOrientation(dummy_id)
            
            # ori = p.getEulerFromQuaternion(quat)[2]
            a = sim.SphereMarker(position=position, radius=0.01, rgba_color = (0, 0, 1, 0.8), orientation=quat)
            env.move_tool_shift(position, quat)
            
            link_state = p.getLinkState(env.robot_body_id, env.robot_tool_tip)
            link_marker = sim.SphereMarker(link_state[0], radius=0.01, orientation=link_state[1], rgba_color=[0, 1, 0, 0.3])
            # env.move_tool(position, quat)
            env.step_simulation(100, sleep_time=0.0001)
            env.robot_go_home()
            env.step_simulation(100, sleep_time=0.0001)



    

    def test_concat(self,num_trials=3, select_object = 0):
        """
        받은 object shape 를 잡아서 들어 올리는 테스트
        """
        env = self.env
        passed = 0
        env.reset_objects()
        
        for _ in range(num_trials):
            # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
            object_id = env._objects_body_ids[select_object]        
            position, grasp_angle = self.get_grasp_position_angle(object_id)

            #shift pose
            # position = list(position)
            # position[1] = position[1]+0.01

            grasp_success = env.execute_grasp_concat(position, grasp_angle, object_id)
            # grasp_success = env.execute_grasp_concat2(position, grasp_angle, object_id)

            # Test for grasping success (this test is a necessary condition, not sufficient):
            object_z = p.getBasePositionAndOrientation(object_id)[0][2]
            if object_z >= 0.2:
                passed += 1
            env.reset_objects()
        print(f"[Grasping] {passed} / {num_trials} cases passed")

    def start_concat(self,num_trials=3, select_object = 0):
        """
        받은 object shape 를 잡아서 들어 올리는 테스트
        """

        pose = [ 0.5, -0.93427129,  0.2]
        ori =  [math.pi/2, 0, math.pi/2]

        pose_ori = [pose, ori]
        env = self.env
        passed = 0
        # env.reset_objects()
        env.set_dummy_pose(pose_ori)
        
        for _ in range(num_trials):
            # _objects_body_ids[?] ? 표를 변경해서 들 물체를 정함
            object_id = env._objects_body_ids[select_object]
                   
            position, grasp_angle = self.get_grasp_position_angle(object_id)

            #shift pose
            # position = list(position)
            # position[1] = position[1]+0.01

            env.ur5_concat_dummy(position, grasp_angle, object_id)
            # grasp_success = env.execute_grasp_concat2(position, grasp_angle, object_id)

            # Test for grasping success (this test is a necessary condition, not sufficient):
            object_z = p.getBasePositionAndOrientation(object_id)[0][2]
            if object_z >= 0.2:
                passed += 1
            env.set_dummy_pose(pose_ori)
        print(f"[Grasping] {passed} / {num_trials} cases passed")
    

   
    
    def get_grasp_position_angle(self, object_id):
        position, grasp_angle = np.zeros((3, 1)), 0
        # ========= PART 2============
        # Get position and orientation (yaw in radians) of the gripper for grasping
        # ==================================
        position, orientation = p.getBasePositionAndOrientation(object_id)
        grasp_angle = p.getEulerFromQuaternion(orientation)[2]
        return position, grasp_angle

    def show_trajactory(self):
        """
        show trajactory with marker
        """
        env = self.env
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
    
    def test_trajactory_pose(self, num_trials = 1):
        # set start pose
        env = self.env
        # self.start_pose()

        trajactory, _ = get_pose('Trajectory_Dummy')
        for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
            # print(idx)
            if idx%10 !=0:
                continue
            pose = trajac[0]
            quat = trajac[1]
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)


            random_position = env._workspace1_bounds[:, 0] + 0.15 + \
                np.random.random_sample((3)) * (env._workspace1_bounds[:, 1] - env._workspace1_bounds[:, 0] - 0.15)
            random_orientation = np.random.random_sample((3)) * np.pi / 4 - np.pi / 8
            random_orientation[1] += np.pi
            random_orientation = p.getQuaternionFromEuler(random_orientation)
            env.move_tool_shift(pose, quat)
            # print(pose)
            # env.move_tool(pose, quat)

            env.step_simulation(10)
        passed = 0
    

    # def ur5_trajactory_pose(self):
    #     # set start pose
    #     env = self.env
    #     # self.start_pose()
    #     # dummy_mesh = 'assets\dummy\dummy_tip.urdf' 
    #     trajactory, _ = get_pose('Trajectory_Dummy')
    #     for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
    #         # print(idx)
    #         # if idx%10 !=0:
    #         #     continue
    #         pose = trajac[0]
    #         quat = trajac[1]
    #         # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
    #         # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
    #         dummy = dummy(pose,quat)

    #         random_position = env._workspace1_bounds[:, 0] + 0.15 + \
    #             np.random.random_sample((3)) * (env._workspace1_bounds[:, 1] - env._workspace1_bounds[:, 0] - 0.15)
    #         random_orientation = np.random.random_sample((3)) * np.pi / 4 - np.pi / 8
    #         random_orientation[1] += np.pi
    #         random_orientation = p.getQuaternionFromEuler(random_orientation)
    #         env.move_tool_shift(pose, quat)
    #         # print(pose)
    #         # env.move_tool(pose, quat)

    #         env.step_simulation(10)
    #     passed = 0
    
    def ur5_tip2_object(self, object_id):
        pass

    def start_pose(self):
        env = self.env
        start_pose = json_obj_path['ur5_start_pose']['pose']
        start_ori  = p.getQuaternionFromEuler(json_obj_path['ur5_start_pose']['ori'])
        env.move_tool(start_pose, start_ori)
        env.step_simulation(100)

    def ur5_trajactory_pose(self):

        scenario = json_obj_path['scenario']['SC']
        if scenario == 'SC1':
            sc_mount = 0
        elif scenario == 'SC2':
            sc_mount =-1
        # set start pose
        env = self.env
        # self.start_pose()
        # dummy_mesh = 'assets\dummy\dummy_tip.urdf'

        trajactory, _ = get_pose2('Trajectory_Dummy')
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
        
        

        for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
        # for idx, trajac in enumerate(trajactory):
            

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
                dummy = sim.dummy(pose)
            if idx <200:
                continue
            ur_infot_list.append([idx, current_joint_state, tool_tip_pose])

            # if idx == 0:
            #     dummy = sim.dummy(pose,quat)
            #     env._dummy_id = dummy.return_id()

            # if idx%10 !=0:
            #     continue
            
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
            # dummy.obj_repose(pose, quat)
            
            dummy.obj_reset_pose(pose, quat)
            
            
            ur_pose = dummy.ur5_pose_quat[0]
            ur_quat = dummy.ur5_pose_quat[1]
            env.move_tool_shift(ur_pose, ur_quat)
           

            # p.resetBasePositionAndOrientation(env._dummy_id, pose, quat)
            # time.sleep(0.01)
            
            
            
            
            

            # env.step_simulation(10)
        passed = 0
        return  ur_infot_list


    def shift_dummy_pose(self, time_data, x=0,y=0,z=0):
        for data in time_data:
            data[0][0] += x
            data[0][1] += y
            data[0][2] += z

        return time_data



    def ur5_trajactory_pose2(self, rand_xyz, scenario='SC2', ep = '2', obj_track = 'Trajectory_Dummy'):
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
        # mount_pose[1] += 0.005
        mount_pose[1] += 0.008
        # z shift
        mount_pose[2] -=0.005
        mount.obj_reset_tip(mount_pose)
        
        
        ur_infot_list = []

        # UR5 base bose
        robot_home_joint_config = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        env.move_joints(robot_home_joint_config, speed=1.0)
        
        

        for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
        # for idx, trajac in enumerate(trajactory):
            

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
                _pose = [-1.162761600542553, -0.8290458752534293, 0.07700031554018469]
                _quat = [0.8385665766060305, 0.34091009624327123, -0.18361610281863264, -0.3832382152974088]
                env.move_tool_shift(_pose, _quat)
                dummy = sim.dummy(pose, useFix=True)
            if idx <300:
                continue
            elif idx == 301:
                env.close_gripper()
                env.step_simulation(100)
                env.hold_gripper()
            ur_infot_list.append([idx, current_joint_state, tool_tip_pose])
            # if idx >= 700:
            #     input()

            # if idx == 0:
            #     dummy = sim.dummy(pose,quat)
            #     env._dummy_id = dummy.return_id()

            # if idx%10 !=0:
            #     continue
            
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
            # dummy.obj_repose(pose, quat)
            
            dummy.obj_reset_pose(pose, quat)
            
            
            ur_pose = dummy.ur5_pose_quat[0]
            ur_quat = dummy.ur5_pose_quat[1]
            env.move_tool_shift(ur_pose, ur_quat)
           

            # p.resetBasePositionAndOrientation(env._dummy_id, pose, quat)
            # time.sleep(0.01)
            
            
            
            
            

            # env.step_simulation(10)
        passed = 0
        return  ur_infot_list
    
    def ur5_trajactory_pose2_2(self, rand_xyz, scenario='SC2', ep = '1', obj_track = 'Trajectory_Dummy'):
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
        
        

        for idx, trajac in tqdm(enumerate(trajactory), total= len(trajactory)):
        # for idx, trajac in enumerate(trajactory):
            

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
                _pose = [-1.162761600542553, -0.8290458752534293, 0.07700031554018469]
                _quat = [0.8385665766060305, 0.34091009624327123, -0.18361610281863264, -0.3832382152974088]
                env.move_tool_shift(_pose, _quat)
                dummy = sim.dummy(pose, useFix=True)
            if idx <300:
                continue
            elif idx == 301:
                env.close_gripper()
                env.step_simulation(100)
                env.hold_gripper()
            if idx >=500:
                input()
            ur_infot_list.append([idx, current_joint_state, tool_tip_pose])

            # if idx == 0:
            #     dummy = sim.dummy(pose,quat)
            #     env._dummy_id = dummy.return_id()

            # if idx%10 !=0:
            #     continue
            
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
            # dummy.obj_repose(pose, quat)
            
            dummy.obj_reset_pose(pose, quat)
            
            
            ur_pose = dummy.ur5_pose_quat[0]
            ur_quat = dummy.ur5_pose_quat[1]
            env.move_tool_shift(ur_pose, ur_quat)
           

            # p.resetBasePositionAndOrientation(env._dummy_id, pose, quat)
            # time.sleep(0.01)
            
            
            
            
            

            # env.step_simulation(10)
        passed = 0
        return  ur_infot_list

    
    def ur5_trajactory_pose3(self, rand_xyz, scenario='SC2', ep = '1', obj_track = 'Trajectory_Dummy', reverse = False):
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
                dummy = sim.dummy(pose, useFix=True)
                ur_pose = dummy.ur5_pose_quat[0]
                ur_quat = dummy.ur5_pose_quat[1]
                env.step_simulation(100)
                del(dummy)
                env.move_tool_shift(ur_pose, ur_quat)
                env.step_simulation(100)

                
                dummy = sim.dummy(pose, useFix=True)
                # env.close_gripper()
                env.step_simulation(100)
                env.hold_gripper()
                env.step_simulation(100)
            else:
                input()
                d = SphereMarker(position = ur_pose)
            dummy.obj_reset_pose(pose, quat)

            # elif idx<300:
            #     continue

            # elif idx >= 900:
            #     input()
                
            

            # if idx == 0:
            #     dummy = sim.dummy(pose,quat)
            #     env._dummy_id = dummy.return_id()

            # if idx%10 !=0:
            #     continue
            
            # quat = p.getQuaternionFromEuler([math.pi, 0,0])
            
            # marker = sim.SphereMarker(position=pose, radius=0.02, orientation=quat)
            # dummy.obj_repose(pose, quat)
            
            
            
            
            ur_pose = dummy.ur5_pose_quat[0]
            ur_quat = dummy.ur5_pose_quat[1]
            ur_ik_joint = env.move_tool_shift(ur_pose, ur_quat)
            print(ur_ik_joint)

            tmp_time_data['idx']                 = idx
            tmp_time_data['current_joint_state'] = current_joint_state
            tmp_time_data['tool_tip_pose']       = tool_tip_pose
            tmp_time_data['ur_ik_joint']         = ur_ik_joint
            # ur_infot_list.append([idx, current_joint_state, tool_tip_pose])
            ur_infot_list.append(tmp_time_data)
           

            # p.resetBasePositionAndOrientation(env._dummy_id, pose, quat)
            # time.sleep(0.01)
            
            
            
            
            

            # env.step_simulation(10)
        passed = 0
        return  ur_infot_list