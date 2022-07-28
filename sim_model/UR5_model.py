import pybullet as p
import pybullet_data
import numpy as np
import time
import json


with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)


class UR5_Model:
    def __init__(self, _ur5=None, _ur5Mount=None, set_gripper = False, ur5_concat = False):
        # UR5 불러오기
        ur5_path = json_obj_path['ur5']['urdf']
        if ur5_concat == True:
            ur5_path = "assets/ur5/ur5_concat_dummy.urdf"
        ur5_pose = json_obj_path['ur5']['pose']
        ur5_ori  = json_obj_path['ur5']['ori']

        # # UR5 setting
        self._joint_epsilon = 1e-3
        self.robot_home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        self._ik_joint = None

        

        



        self.robot_body_id = p.loadURDF(ur5_path, ur5_pose, p.getQuaternionFromEuler(ur5_ori))
        
        # self.robot_body_id = p.loadURDF(
        #     json_obj_path['ur5']['urdf'], json_obj_path['ur5']['pose'], p.getQuaternionFromEuler(json_obj_path['ur5']['ori']))

        # # set end Effectory
        self.robot_end_effector_link_index = 9
        self.robot_tool_tip = 10
        self._robot_tool_offset = [0, 0, -0.05]
        # Distance between tool tip and end-effector joint
        self._tool_tip_to_ee_joint = np.array([0, 0, 0.15])

        # # UR5 Mount 불러오기
        _ur5Mount_path    = json_obj_path['ur5_mount']['urdf']
        _ur5Mount_pose    = ur5_pose
        _ur5Mount_pose[2]-= 0.2
        _ur5Mount_ori     = json_obj_path['ur5_mount']['ori']
        self._mount_body_id = p.loadURDF(_ur5Mount_path, _ur5Mount_pose, p.getQuaternionFromEuler(_ur5Mount_ori))

        # Get revolute joint indices of robot (skip fixed joints)
        robot_joint_info = [p.getJointInfo(self.robot_body_id, i) for i in range(p.getNumJoints(self.robot_body_id))]
        self._robot_joint_indices = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]

        # set gripper
        self._gripper_body_id = None
        if set_gripper == True:
            self.load_gripper()


        # set Base Pose
        self.robot_home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        # self.robot_home_joint_config = [np.pi, -np.pi/2, 0, 0, 0, 0]
        self.move_joints(self.robot_home_joint_config, speed=1.0)

    def load_gripper(self):
        if self._gripper_body_id is not None:
            print("Gripper already loaded")
            return

        # Attach robotiq gripper to UR5 robot
        # - We use createConstraint to add a fixed constraint between the ur5 robot and gripper.
        self._gripper_body_id = p.loadURDF("assets/gripper/robotiq_2f_85.urdf")
        p.resetBasePositionAndOrientation(self._gripper_body_id, [
                                        0.5, 0.1, 0.2], p.getQuaternionFromEuler([np.pi, 0, 0]))

        p.createConstraint(self.robot_body_id, self.robot_end_effector_link_index, self._gripper_body_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
                        0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=self._robot_tool_offset, childFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2]))

        # Set friction coefficients for gripper fingers
        for i in range(p.getNumJoints(self._gripper_body_id)):
            p.changeDynamics(self._gripper_body_id, i, lateralFriction=1.0, spinningFriction=1.0,
                            rollingFriction=0.0001, frictionAnchor=True)
        self.step_simulation(100)
        self.open_gripper()

        self.step_simulation(100)
    
    def close_gripper(self, f = 10000):
        if self._gripper_body_id is not None:
            p.setJointMotorControl2(
                self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=f)
            self.step_simulation(4e2)
        
    def open_gripper(self):
        if self._gripper_body_id is not None:
            p.setJointMotorControl2(
                self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=-5, force=10000)
            self.step_simulation(4e2)
        
    
    def hold_gripper(self):
        if self._gripper_body_id is not None:
            motor_kp = 0.5
            motor_kd = 0.225
            motor_torque = 1.4
            motor_max_velocity = 5.9
            
            p.setJointMotorControl2(
                bodyIndex = self._gripper_body_id, jointIndex = 1, controlMode = p.POSITION_CONTROL, targetPosition=p.getJointState(self._gripper_body_id, 1)[0],
                positionGain=motor_kp, velocityGain=motor_kd, force=motor_torque, maxVelocity=motor_max_velocity

            )


    
    def return_id(self):
        dict_id = dict()
        dict_id['robot_body_id'] = self.robot_body_id
        dict_id['robot_end_effector_link_index'] = self.robot_end_effector_link_index
        dict_id['robot_tool_tip'] = self.robot_tool_tip
        dict_id['_robot_tool_offset'] = self._robot_tool_offset
        dict_id['_tool_tip_to_ee_joint'] = self._tool_tip_to_ee_joint

        return dict_id

    def obj_reset_pose(self, pose, quat):


        if len(quat) == 3:
            # ori -> quat
            quat = p.getQuaternionFromEuler(quat)
        # UR5 pose set
        p.resetBasePositionAndOrientation(self.robot_body_id, pose, quat)
        pose[2] -= 0.2

        # UR5 Mount pose set
        p.resetBasePositionAndOrientation(self._mount_body_id, pose, quat)

    
    #### move UR5
    # move joint
    def move_joints(self, target_joint_state, speed=0.3):
        """
            Move robot arm to specified joint configuration by appropriate motor control
        """
        assert len(self._robot_joint_indices) == len(target_joint_state)
        p.setJointMotorControlArray(
            self.robot_body_id, self._robot_joint_indices,
            p.POSITION_CONTROL, target_joint_state,
            positionGains=speed * np.ones(len(self._robot_joint_indices))
        )

        timeout_t0 = time.time()
        while True:
            # Keep moving until joints reach the target configuration
            current_joint_state = [
                p.getJointState(self.robot_body_id, i)[0]
                for i in self._robot_joint_indices
            ]
            if all([
                np.abs(
                    current_joint_state[i] - target_joint_state[i]) < self._joint_epsilon
                for i in range(len(self._robot_joint_indices))
            ]):
                break
            if time.time()-timeout_t0 > 1:
                print(
                    "Timeout: robot is taking longer than 10s to reach the target joint state. Skipping...")
                # p.setJointMotorControlArray(
                #     self.robot_body_id, self._robot_joint_indices,
                #     p.POSITION_CONTROL, self.robot_home_joint_config,
                #     positionGains=np.ones(len(self._robot_joint_indices))
                # )
                break
            self.step_simulation(1)
    def move_tool(self, position, orientation, speed=0.03):
        """
            Move robot tool (end-effector) to a specified pose
            @param position: Target position of the end-effector link
            @param orientation: Target orientation of the end-effector link
        """
        target_joint_state = np.zeros((6,))  # this should contain appropriate joint angle values
        # ========= Part 1 ========
        # Using inverse kinematics (p.calculateInverseKinematics), find out the target joint configuration of the robot
        # in order to reach the desired end_effector position and orientation
        # HINT: p.calculateInverseKinematics takes in the end effector **link index** and not the **joint index**. You can use 
        #   self.robot_end_effector_link_index for this 
        # HINT: You might want to tune optional parameters of p.calculateInverseKinematics for better performance
        # ===============================
        target_joint_state = p.calculateInverseKinematics(self.robot_body_id,
                                                          self.robot_end_effector_link_index,
                                                          position, orientation,
                                                          maxNumIterations=100, residualThreshold=1e-4)
        self.move_joints(target_joint_state)
    
    def move_tool_shift(self, position, orientation, speed=0.03):
        """
            Move robot tool (end-effector) to a specified pose
            @param position: Target position of the end-effector link
            @param orientation: Target orientation of the end-effector link
        """
        target_joint_state = np.zeros((6,))  # this should contain appropriate joint angle values
        # ========= Part 1 ========
        # Using inverse kinematics (p.calculateInverseKinematics), find out the target joint configuration of the robot
        # in order to reach the desired end_effector position and orientation
        # HINT: p.calculateInverseKinematics takes in the end effector **link index** and not the **joint index**. You can use 
        #   self.robot_end_effector_link_index for this 
        # HINT: You might want to tune optional parameters of p.calculateInverseKinematics for better performance
        # ===============================
        target_joint_state = p.calculateInverseKinematics(self.robot_body_id,
                                                          self.robot_tool_tip,
                                                        #   self.tool0_fixed_joint_tool_tip_index,
                                                          position, orientation,
                                                          maxNumIterations=100, residualThreshold=1e-4)
        self._ik_joint = target_joint_state
        self.move_joints(target_joint_state)
        return target_joint_state

    def move_tool_shift_relative(self, relative_pose):
        # get relative pose x y z
        
        # 현재의 위치 확인
        tool_tip_pose = p.getLinkState(self.robot_body_id, self.robot_tool_tip)[0:2]
        pose= list(tool_tip_pose[0])
       
        # set goal pose
        a=1
        for i in range(3):
            pose[i] += relative_pose[i]

        self.move_tool_shift(pose, tool_tip_pose[1])

        tool_tip_pose = p.getLinkState(self.robot_body_id, self.robot_tool_tip)[0:2]
        pose= list(tool_tip_pose[0])
        



        



    

    ###### move ur5 End ########

    
    
    def close_gripper(self, f = 1):
        if self._gripper_body_id is not None:
            p.setJointMotorControl2(
                self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=f)
            self.step_simulation(4e2)
        
    def open_gripper(self):
        if self._gripper_body_id is not None:
            p.setJointMotorControl2(
                self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=-5, force=10000)
            self.step_simulation(4e2)
        
    
    def hold_gripper(self):
        if self._gripper_body_id is not None:
            motor_kp = 0.5
            motor_kd = 0.225
            motor_torque = 1.4
            motor_max_velocity = 5.9
            
            p.setJointMotorControl2(
                bodyIndex = self._gripper_body_id, jointIndex = 1, controlMode = p.POSITION_CONTROL, targetPosition=p.getJointState(self._gripper_body_id, 1)[0],
                positionGain=motor_kp, velocityGain=motor_kd, force=motor_torque, maxVelocity=motor_max_velocity

            )

    
    def check_grasp_success(self):
        # return p.getJointState(self._gripper_body_id, 1)[0] < 0.834 - 0.001
        if self._gripper_body_id is not None:
            return p.getJointState(self._gripper_body_id, 1)[0] < 0.7
    
    # for UR5 model test
    def step_simulation(self, num_steps, sleep_time = None, object_ori = None):
        for i in range(int(num_steps)):
            if sleep_time != None:
                time.sleep(sleep_time)
            p.stepSimulation()
            if self._gripper_body_id is not None:
                # Constraints
                gripper_joint_positions = np.array([p.getJointState(self._gripper_body_id, i)[
                                                0] for i in range(p.getNumJoints(self._gripper_body_id))])
                p.setJointMotorControlArray(
                    self._gripper_body_id, [6, 3, 8, 5, 10], p.POSITION_CONTROL,
                    [
                        gripper_joint_positions[1], -gripper_joint_positions[1], 
                        -gripper_joint_positions[1], gripper_joint_positions[1],
                        gripper_joint_positions[1]
                    ],
                    positionGains=np.ones(5)
                )

    
    def __del__(self):
        print('del UR5')
        p.removeBody(self.robot_body_id)
        print('del UR5 Mount')
        p.removeBody(self._mount_body_id)
        if self._gripper_body_id != None:
            print('del gripper')
            p.removeBody(self._gripper_body_id)


# class UR5_Model_concat_dummy:
#     def __init__(self, _ur5=None, _ur5Mount=None, set_gripper = False):
#         # UR5 불러오기
#         ur5_path = "assets/ur5/ur5_concat_dummy.urdf"
#         ur5_pose = json_obj_path['ur5']['pose']
#         ur5_ori  = json_obj_path['ur5']['ori']

#         # # UR5 setting
#         self._joint_epsilon = 1e-3
#         self.robot_home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]

        



#         self.robot_body_id = p.loadURDF(ur5_path, ur5_pose, p.getQuaternionFromEuler(ur5_ori))
        
#         # self.robot_body_id = p.loadURDF(
#         #     json_obj_path['ur5']['urdf'], json_obj_path['ur5']['pose'], p.getQuaternionFromEuler(json_obj_path['ur5']['ori']))

#         # # set end Effectory
#         self.robot_end_effector_link_index = 9
#         self.robot_tool_tip = 10
#         self._robot_tool_offset = [0, 0, -0.05]
#         # Distance between tool tip and end-effector joint
#         self._tool_tip_to_ee_joint = np.array([0, 0, 0.15])

#         # # UR5 Mount 불러오기
#         _ur5Mount_path    = json_obj_path['ur5_mount']['urdf']
#         _ur5Mount_pose    = ur5_pose
#         _ur5Mount_pose[2]-= 0.2
#         _ur5Mount_ori     = json_obj_path['ur5_mount']['ori']
#         self._mount_body_id = p.loadURDF(_ur5Mount_path, _ur5Mount_pose, p.getQuaternionFromEuler(_ur5Mount_ori))

#         # Get revolute joint indices of robot (skip fixed joints)
#         robot_joint_info = [p.getJointInfo(self.robot_body_id, i) for i in range(p.getNumJoints(self.robot_body_id))]
#         self._robot_joint_indices = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]

#         # set gripper
#         self._gripper_body_id = None
#         if set_gripper == True:
#             self.load_gripper()


#         # set Base Pose
#         self.robot_home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
#         # self.robot_home_joint_config = [np.pi, -np.pi/2, 0, 0, 0, 0]
#         self.move_joints(self.robot_home_joint_config, speed=1.0)

#     def load_gripper(self):
#         if self._gripper_body_id is not None:
#             print("Gripper already loaded")
#             return

#         # Attach robotiq gripper to UR5 robot
#         # - We use createConstraint to add a fixed constraint between the ur5 robot and gripper.
#         self._gripper_body_id = p.loadURDF("assets/gripper/robotiq_2f_85.urdf")
#         p.resetBasePositionAndOrientation(self._gripper_body_id, [
#                                         0.5, 0.1, 0.2], p.getQuaternionFromEuler([np.pi, 0, 0]))

#         p.createConstraint(self.robot_body_id, self.robot_end_effector_link_index, self._gripper_body_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
#                         0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=self._robot_tool_offset, childFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2]))

#         # Set friction coefficients for gripper fingers
#         for i in range(p.getNumJoints(self._gripper_body_id)):
#             p.changeDynamics(self._gripper_body_id, i, lateralFriction=1.0, spinningFriction=1.0,
#                             rollingFriction=0.0001, frictionAnchor=True)
#         self.step_simulation(100)
#         self.open_gripper()

#         self.step_simulation(100)
    
#     def close_gripper(self, f = 10000):
#         if self._gripper_body_id is not None:
#             p.setJointMotorControl2(
#                 self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=f)
#             self.step_simulation(4e2)
        
#     def open_gripper(self):
#         if self._gripper_body_id is not None:
#             p.setJointMotorControl2(
#                 self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=-5, force=10000)
#             self.step_simulation(4e2)
        
    
#     def hold_gripper(self):
#         if self._gripper_body_id is not None:
#             motor_kp = 0.5
#             motor_kd = 0.225
#             motor_torque = 1.4
#             motor_max_velocity = 5.9
            
#             p.setJointMotorControl2(
#                 bodyIndex = self._gripper_body_id, jointIndex = 1, controlMode = p.POSITION_CONTROL, targetPosition=p.getJointState(self._gripper_body_id, 1)[0],
#                 positionGain=motor_kp, velocityGain=motor_kd, force=motor_torque, maxVelocity=motor_max_velocity

#             )


    
#     def return_id(self):
#         dict_id = dict()
#         dict_id['robot_body_id'] = self.robot_body_id
#         dict_id['robot_end_effector_link_index'] = self.robot_end_effector_link_index
#         dict_id['robot_tool_tip'] = self.robot_tool_tip
#         dict_id['_robot_tool_offset'] = self._robot_tool_offset
#         dict_id['_tool_tip_to_ee_joint'] = self._tool_tip_to_ee_joint

#         return dict_id

#     def obj_reset_pose(self, pose, quat):


#         if len(quat) == 3:
#             # ori -> quat
#             quat = p.getQuaternionFromEuler(quat)
#         # UR5 pose set
#         p.resetBasePositionAndOrientation(self.robot_body_id, pose, quat)
#         pose[2] -= 0.2

#         # UR5 Mount pose set
#         p.resetBasePositionAndOrientation(self._mount_body_id, pose, quat)

    

#     # move joint 
#     def move_joints(self, target_joint_state, speed=0.3):
#         """
#             Move robot arm to specified joint configuration by appropriate motor control
#         """
#         assert len(self._robot_joint_indices) == len(target_joint_state)
#         p.setJointMotorControlArray(
#             self.robot_body_id, self._robot_joint_indices,
#             p.POSITION_CONTROL, target_joint_state,
#             positionGains=speed * np.ones(len(self._robot_joint_indices))
#         )

#         timeout_t0 = time.time()
#         while True:
#             # Keep moving until joints reach the target configuration
#             current_joint_state = [
#                 p.getJointState(self.robot_body_id, i)[0]
#                 for i in self._robot_joint_indices
#             ]
#             if all([
#                 np.abs(
#                     current_joint_state[i] - target_joint_state[i]) < self._joint_epsilon
#                 for i in range(len(self._robot_joint_indices))
#             ]):
#                 break
#             if time.time()-timeout_t0 > 1:
#                 print(
#                     "Timeout: robot is taking longer than 10s to reach the target joint state. Skipping...")
#                 # p.setJointMotorControlArray(
#                 #     self.robot_body_id, self._robot_joint_indices,
#                 #     p.POSITION_CONTROL, self.robot_home_joint_config,
#                 #     positionGains=np.ones(len(self._robot_joint_indices))
#                 # )
#                 break
#             self.step_simulation(1)

    
    
#     def close_gripper(self, f = 1):
#         if self._gripper_body_id is not None:
#             p.setJointMotorControl2(
#                 self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=f)
#             self.step_simulation(4e2)
        
#     def open_gripper(self):
#         if self._gripper_body_id is not None:
#             p.setJointMotorControl2(
#                 self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=-5, force=10000)
#             self.step_simulation(4e2)
        
    
#     def hold_gripper(self):
#         if self._gripper_body_id is not None:
#             motor_kp = 0.5
#             motor_kd = 0.225
#             motor_torque = 1.4
#             motor_max_velocity = 5.9
            
#             p.setJointMotorControl2(
#                 bodyIndex = self._gripper_body_id, jointIndex = 1, controlMode = p.POSITION_CONTROL, targetPosition=p.getJointState(self._gripper_body_id, 1)[0],
#                 positionGain=motor_kp, velocityGain=motor_kd, force=motor_torque, maxVelocity=motor_max_velocity

#             )

    
#     def check_grasp_success(self):
#         # return p.getJointState(self._gripper_body_id, 1)[0] < 0.834 - 0.001
#         if self._gripper_body_id is not None:
#             return p.getJointState(self._gripper_body_id, 1)[0] < 0.7
    
#     # for UR5 model test
#     def step_simulation(self, num_steps, sleep_time = None, object_ori = None):
#         for i in range(int(num_steps)):
#             if sleep_time != None:
#                 time.sleep(sleep_time)
#             p.stepSimulation()
#             if self._gripper_body_id is not None:
#                 # Constraints
#                 gripper_joint_positions = np.array([p.getJointState(self._gripper_body_id, i)[
#                                                 0] for i in range(p.getNumJoints(self._gripper_body_id))])
#                 p.setJointMotorControlArray(
#                     self._gripper_body_id, [6, 3, 8, 5, 10], p.POSITION_CONTROL,
#                     [
#                         gripper_joint_positions[1], -gripper_joint_positions[1], 
#                         -gripper_joint_positions[1], gripper_joint_positions[1],
#                         gripper_joint_positions[1]
#                     ],
#                     positionGains=np.ones(5)
#                 )

    
#     def __del__(self):
#         print('del UR5')
#         p.removeBody(self.robot_body_id)
#         print('del UR5 Mount')
#         p.removeBody(self._mount_body_id)
#         if self._gripper_body_id != None:
#             print('del gripper')
#             p.removeBody(self._gripper_body_id)


        

        
if __name__ == '__main__':
    # 기본 세팅
    p.connect(p.GUI)

    # set option
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)            
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)      
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)     
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)

    # 기본 배경 로드 및 중력 적용
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)


    # test ur5 load end delete
    # for i in range(2):
    #     print('ur5 생성')
    #     ur5 = UR5_Model(set_gripper=False)
    #     time.sleep(2)
    #     print('ur5 제거')
    #     del(ur5)
    #     time.sleep(2)
    
    
    ur5 = UR5_Model_concat_dummy()

    # UR5 move joint
    print("UR5 Move Joint Test")

    

    # for idx, i in enumerate(range(1)):
    #     # ur5.move_joints([-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])
    #     print(idx)
    #     ur5.move_joints([np.pi, -np.pi/2, 0, 0, 0, 0], speed=1.0)
    #     time.sleep(0.5)
    #     ur5.move_joints([0, 0,0, 0, 0, 0], speed=1.0)
    #     time.sleep(2)
    
    # # test ur5 pose reset
    # for i in range(2):

    #     print('pose : 1,1,1')
    #     ur5.obj_reset_pose([1,1,1], [0,0,0])
    #     time.sleep(1)
    
    #     ur5.obj_reset_pose([1,1,1.5], [0,0,0])
    #     print('pose : 1,1,1.5')
    #     for _ in range(100):
    #         p.stepSimulation()
        
    #     time.sleep(1)
    
    # for idx, i in enumerate(range(1)):
    #     # ur5.move_joints([-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0])
    #     print(idx)
    #     ur5.move_joints([np.pi, -np.pi/2, 0, 0, 0, 0], speed=1.0)
    #     time.sleep(0.5)
    #     ur5.move_joints([0, 0,0, 0, 0, 0], speed=1.0)
    #     time.sleep(2)


    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)



