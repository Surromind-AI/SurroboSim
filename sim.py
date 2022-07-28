import pybullet as p
import pybullet_data
import numpy as np
import time
import json

from util import rotate_quant
from util import *

# from sim_model_class import SphereMarker, simul_dummy



# from sim_model.dummy_model import dummy
from sim_model import *

with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)

    
class PyBulletSim(UR5_Model):
    """
    PyBulletSim: Implements two tote UR5 simulation environment with obstacles for grasping 
        and manipulation
    """
    def __init__(self, use_random_objects=False, object_shapes=None, gui=True, verbose = True, object_random = True):
        self._get_info = False
        if gui:
            p.connect(p.GUI)

            # set option
            p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)            
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0)      
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0)     
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
            
        else:
            print("cannot connetc GUI")
        
        # setting vale
        # self.robot_body_id                 = None
        # self._mount_body_id                = None
        # self._gripper_body_id              = None
        # self._dummy_id                     = None
        # self.robot_end_effector_link_index = None
        # self.robot_tool_tip                = None
        # self._robot_tool_offset            = None
        # self._tool_tip_to_ee_joint         = None
        # self._robot_joint_indices          = None
        self._get_time_data                = []
        self._dummy_obj = None

        # set dummy
        self._tracking_dumy_id             = None

        # joint position threshold in radians (i.e. move until joint difference < epsilon)
        # self._joint_epsilon = 1e-3
        # self._joint_epsilon = 0.1
        # Robot home joint configuration (over tote 1)
        # sc1
        # self.robot_home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        # sc2
        # self.robot_home_joint_config = [0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]

        # self.robot_home_joint_config = [np.pi, -np.pi/2, 0, 0, 0, 0]
        # Robot goal joint configuration (over tote 2)
        # self.robot_goal_joint_config = [
        #     0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        # - possible object colors
        self._object_colors = get_tableau_palette()

        self._reward_epsilon = 0.01


        # 3D workspace for tote 1
        # self._workspace1_bounds = np.array([
        #     [0.4, 0.5],  # 3x2 rows: x,y,z cols: min,max 
        #     [-1, -1.22],
        #     [0.00, 0.5]
        # ])
        # # 3D workspace for tote 2
        # self._workspace2_bounds = np.copy(self._workspace1_bounds)
        # self._workspace2_bounds[0, :] = - self._workspace2_bounds[0, ::-1]

        
        
        

        # 기본 배경 로드 및 중력 적용
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._plane_id = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.8)
        # p.setGravity(0, 0, 0)

        # UR5 불러오기
        # self._init_ur5()
        # mount bar 불러오기
        # self._init_moutbar()
        super().__init__()
        self.move_joints(self.robot_home_joint_config, speed=1.0)

        # Load totes and fix them to their position
        # self._tote1_position = (
        #     self._workspace1_bounds[:, 0] + self._workspace1_bounds[:, 1]) / 2
        # self._tote1_position[2] = 0.01
        # self._tote1_body_id = p.loadURDF(
        #     "assets/tote/toteA_large.urdf", self._tote1_position, p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        # set basket if it need
        # self._tote1_body_id = p.loadURDF(json_obj_path['basket']['urdf'], json_obj_path['basket']['pose'], p.getQuaternionFromEuler(json_obj_path['basket']['ori']), useFixedBase=True)

        # self._tote2_position = (
        #     self._workspace2_bounds[:, 0] + self._workspace2_bounds[:, 1]) / 2
        # self._tote2_position[2] = 0.01
        # self._tote2_body_id = p.loadURDF(
        #     "assets/tote/toteA_large.urdf", self._tote2_position, p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)

        
        # Load objects
        

        # 물체 입력 예제
        # if object_shapes is not None:
        #     self._object_shapes = object_shapes
        # else:
        #     # 물체가 없을 경우 테스트 용
        #     self._object_shapes = [
        #         "assets/objects/cube.urdf",
        #         "assets/objects/rod.urdf",
        #         "assets/objects/custom.urdf"
        #     ]


        # self._num_objects = len(self._object_shapes)
        # self._object_shape_ids = [
        #     i % len(self._object_shapes) for i in range(self._num_objects)]
        # self._objects_body_ids = []
        
        
        # for i in range(self._num_objects):
        #     object_body_id = p.loadURDF(self._object_shapes[i], [ 0.5, 0.1, 0.1], p.getQuaternionFromEuler([2, 3.14, 0]))
        #     self._objects_body_ids.append(object_body_id)
        #     p.changeVisualShape(object_body_id, -1, rgbaColor=[*self._object_colors[i], 2])
        # self.reset_objects()
        # self.set_objects()

        
        
        

        # 더미 파일 로드
        # self._dummy_ids = []
        # self.set_dummy()
        
        if verbose == True:
            self.check_info()
    def _init_ur5(self):
        # UR5 불러오기
        self.robot_body_id = p.loadURDF(
            json_obj_path['ur5']['urdf'], json_obj_path['ur5']['pose'], p.getQuaternionFromEuler(json_obj_path['ur5']['ori']))
        # ur5 Mount
        self._mount_body_id = p.loadURDF(
            json_obj_path['ur5_mount']['urdf'], json_obj_path['ur5_mount']['pose'], p.getQuaternionFromEuler(json_obj_path['ur5_mount']['ori']))
        # Placeholder for gripper body id  그리퍼 관련
        self._gripper_body_id = None
        self.robot_end_effector_link_index = 9
        self.robot_tool_tip = 10
        self._robot_tool_offset = [0, 0, -0.05]
        # Distance between tool tip and end-effector joint
        self._tool_tip_to_ee_joint = np.array([0, 0, 0.15])

        # Get revolute joint indices of robot (skip fixed joints)
        robot_joint_info = [p.getJointInfo(self.robot_body_id, i) for i in range(
            p.getNumJoints(self.robot_body_id))]
        self._robot_joint_indices = [
            x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
    
    def _init_moutbar(self):
        # Mount Bar Load
        obj_pose_list, obj_mesh = get_pose('MountingBar')
        # obj_mesh = 'assets/tmp/shelf_mount.urdf'
        self.hook_object = []
        for idx, obj_pose in enumerate(obj_pose_list):
            if idx != 0:
                continue
            obj_pose[0][0] += 0.5
            self.hook_object.append(p.loadURDF(obj_mesh, basePosition=obj_pose[0], baseOrientation=obj_pose[1], useFixedBase=True))
    


    
    
    def get_pose_info(self, dummy = None, mount = None):
        pose_info = dict()
        pose_info['tool_tip_pose']    = p.getLinkState(self.robot_body_id, self.robot_tool_tip)[0:2]
        goal = None
        if dummy != None and mount != None:
            # dummy pose
            pose_info['dummy_hole']   = p.getBasePositionAndOrientation(dummy.dummy_id)
            pose_info['dummy_center'] = p.getLinkState(dummy.dummy_id,2)[0:2]

            # mount pose
            mount_center, mount_tool_tip = mount.pose_info()
            pose_info['mount_center'] = mount_center
            pose_info['mount_tool_tip'] = mount_tool_tip



            reward_pose= np.abs(np.array(pose_info['dummy_hole'][0]) - np.array(pose_info['mount_tool_tip'][0]))


            if all([np.abs(pose_info['dummy_hole'][0][i] - pose_info['mount_tool_tip'][0][i]) < self._reward_epsilon for i in range(len(pose_info['dummy_hole'][0])) ]):
                goal = True
            else:
                goal = False
        
        
        return pose_info, goal, reward_pose
        

        
        
    # def set_dummy(self):
    #     # dummy setting in sceinario
    #     obj_pose, _ = get_pose('HangingDummy')
    #     _pose = obj_pose[0]
    #     # _pose = pose_shift(_pose, x = 0.03, z=-0.075, y =0.01)
    #     _quant = obj_pose[1]
        

    #     # dummy_mesh = 'assets\\dummy\\dummy_test.urdf'
    #     # dummy_mesh = 'assets/tmp/tmp_dummy.urdf'
    #     dummy_mesh = 'assets\dummy\dummy_tip.urdf'        
    #     # test
    #     # dummy_mesh = "assets\\objects\\rod.urdf"
    #     # _quant = rotate_quant(_quant, y = math.pi/2)
        
    #     # p.loadURDF('assets\\dummy\\dummy_test.urdf', obj_pose[0], obj_pose[1])
    #     track_id = p.loadURDF(dummy_mesh, _pose, _quant, useFixedBase=True)
    #     self._dummy_ids.append(track_id)
    #     self._tracking_dumy_id = track_id

    # def move_joints(self, target_joint_state, speed=0.03):
    #     """
    #         Move robot arm to specified joint configuration by appropriate motor control
    #     """
    #     assert len(self._robot_joint_indices) == len(target_joint_state)
    #     p.setJointMotorControlArray(
    #         self.robot_body_id, self._robot_joint_indices,
    #         p.POSITION_CONTROL, target_joint_state,
    #         positionGains=speed * np.ones(len(self._robot_joint_indices))
    #     )

    #     timeout_t0 = time.time()
    #     while True:
    #         # Keep moving until joints reach the target configuration
    #         current_joint_state = [
    #             p.getJointState(self.robot_body_id, i)[0]
    #             for i in self._robot_joint_indices
    #         ]
    #         #  # IK 확인용
    #         # tmp_epsilon = []
    #         # for i in range(len(self._robot_joint_indices)):
    #         #     tmp_epsilon.append(np.abs(current_joint_state[i] - target_joint_state[i]))
    #         # print(tmp_epsilon)
    #         # # IK 확인용 end
    #         if all([np.abs(current_joint_state[i] - target_joint_state[i]) < self._joint_epsilon for i in range(len(self._robot_joint_indices))]):
    #             break
    #         if time.time()-timeout_t0 > 0.2:
    #             print(
    #                 "Timeout: robot is taking longer than 10s to reach the target joint state. Skipping...")
    #             p.setJointMotorControlArray(
    #                 self.robot_body_id, self._robot_joint_indices,
    #                 p.POSITION_CONTROL, self.robot_home_joint_config,
    #                 positionGains=np.ones(len(self._robot_joint_indices))
    #             )
    #             break
    #         self.step_simulation(1)

    def step_simulation(self, num_steps, sleep_time = None, object_ori = None):
        for i in range(int(num_steps)):
            p.stepSimulation()
            # dummy pose check
        
            if self._dummy_obj is not None and self._dummy_obj.ur5_flag == True:
                self._dummy_obj.obj_reset_pose()


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
            # time.sleep(0.0001)
            if sleep_time != None:
                time.sleep(sleep_time)

            
            current_joint_state = [
                p.getJointState(self.robot_body_id, i)[0]
                for i in self._robot_joint_indices
            ]
            tool_tip_pose = p.getLinkState(self.robot_body_id,self.robot_tool_tip)[0:2]
            # dummy_pose = p.getBasePositionAndOrientation(self._dummy_id)

            self._get_time_data.append([current_joint_state, tool_tip_pose])


            # print(current_joint_state)
            if self._get_info == True:
                # write what you want 
                # tool tip pose
                tool_tip_pose = p.getLinkState(self.robot_body_id,self.robot_tool_tip)[0:2]
                print(f'tool_tip_pose {tool_tip_pose}' )

                # hook pose
                for idx, hook_obj in enumerate(self.hook_object):
                    print(f'{idx}_hook {p.getBasePositionAndOrientation(hook_obj)}')
                    
                
                # obj
                for idx, obj_pose in enumerate(self._object_shape_ids):
                    print(f'{idx}_object {p.getBasePositionAndOrientation(obj_pose)}')
                
                # dummy
                for idx, dummy in enumerate(self._dummy_ids):
                    print(f'{idx}_dummy {p.getBasePositionAndOrientation(dummy)}')
    
    

    # def reset_objects(self):
    #     for object_body_id in self._objects_body_ids:
    #         random_position = np.random.random_sample((3))*(self._workspace1_bounds[:, 1]-(
    #             self._workspace1_bounds[:, 0]+0.1))+self._workspace1_bounds[:, 0]+0.1
    #         random_orientation = np.random.random_sample((3))*2*np.pi-np.pi
    #         random_orientation[2] = np.pi/2
    #         p.resetBasePositionAndOrientation(
    #             object_body_id, random_position, p.getQuaternionFromEuler(random_orientation))
    #     self.step_simulation(2e2)
    
    # def set_dummy_pose(self, pose_ori):
    #     for object_body_id in self._objects_body_ids:
    #         p.resetBasePositionAndOrientation(
    #             object_body_id, pose_ori[0], p.getQuaternionFromEuler(pose_ori[1]))
    #     self.step_simulation(2e2)


    
    def set_objects(self):
        for object_body_id in self._objects_body_ids:
            # random_position = np.random.random_sample((3))*(self._workspace1_bounds[:, 1]-(
            #     self._workspace1_bounds[:, 0]+0.1))+self._workspace1_bounds[:, 0]+0.1
            # random_orientation = np.random.random_sample((3))*2*np.pi-np.pi
            id_info = p.getBodyInfo(object_body_id)[1].decode('UTF-8')

            if json_obj_path[id_info]['quant'] != 'None':
                p.resetBasePositionAndOrientation(
                object_body_id, json_obj_path[id_info]['pose'], json_obj_path[id_info]['quant'])
            else:
                print("None qunat  set by ori")
                p.resetBasePositionAndOrientation(
                object_body_id, json_obj_path[id_info]['pose'], p.getQuaternionFromEuler(json_obj_path[id_info]['ori']))

            
        self.step_simulation(2e2, sleep_time= 0.01)
    

    # def move_tool(self, position, orientation, speed=0.03):
    #     """
    #         Move robot tool (end-effector) to a specified pose
    #         @param position: Target position of the end-effector link
    #         @param orientation: Target orientation of the end-effector link
    #     """
    #     target_joint_state = np.zeros((6,))  # this should contain appropriate joint angle values
    #     # ========= Part 1 ========
    #     # Using inverse kinematics (p.calculateInverseKinematics), find out the target joint configuration of the robot
    #     # in order to reach the desired end_effector position and orientation
    #     # HINT: p.calculateInverseKinematics takes in the end effector **link index** and not the **joint index**. You can use 
    #     #   self.robot_end_effector_link_index for this 
    #     # HINT: You might want to tune optional parameters of p.calculateInverseKinematics for better performance
    #     # ===============================
    #     target_joint_state = p.calculateInverseKinematics(self.robot_body_id,
    #                                                       self.robot_end_effector_link_index,
    #                                                       position, orientation,
    #                                                       maxNumIterations=100, residualThreshold=1e-4)
    #     self.move_joints(target_joint_state)
    
    # def move_tool_shift(self, position, orientation, speed=0.03):
    #     """
    #         Move robot tool (end-effector) to a specified pose
    #         @param position: Target position of the end-effector link
    #         @param orientation: Target orientation of the end-effector link
    #     """
    #     target_joint_state = np.zeros((6,))  # this should contain appropriate joint angle values
    #     # ========= Part 1 ========
    #     # Using inverse kinematics (p.calculateInverseKinematics), find out the target joint configuration of the robot
    #     # in order to reach the desired end_effector position and orientation
    #     # HINT: p.calculateInverseKinematics takes in the end effector **link index** and not the **joint index**. You can use 
    #     #   self.robot_end_effector_link_index for this 
    #     # HINT: You might want to tune optional parameters of p.calculateInverseKinematics for better performance
    #     # ===============================
    #     target_joint_state = p.calculateInverseKinematics(self.robot_body_id,
    #                                                       self.robot_tool_tip,
    #                                                     #   self.tool0_fixed_joint_tool_tip_index,
    #                                                       position, orientation,
    #                                                       maxNumIterations=100, residualThreshold=1e-4)
    #     self.move_joints(target_joint_state)
    #     return target_joint_state

    def robot_go_home(self, speed=0.1):
        self.move_joints(self.robot_home_joint_config, speed)

    # def load_gripper(self):
    #     if self._gripper_body_id is not None:
    #         print("Gripper already loaded")
    #         return

    #     # Attach robotiq gripper to UR5 robot
    #     # - We use createConstraint to add a fixed constraint between the ur5 robot and gripper.
    #     self._gripper_body_id = p.loadURDF("assets/gripper/robotiq_2f_85.urdf")
    #     p.resetBasePositionAndOrientation(self._gripper_body_id, [
    #                                       0.5, 0.1, 0.2], p.getQuaternionFromEuler([np.pi, 0, 0]))

    #     p.createConstraint(self.robot_body_id, self.robot_end_effector_link_index, self._gripper_body_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #                        0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=self._robot_tool_offset, childFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2]))

    #     # Set friction coefficients for gripper fingers
    #     for i in range(p.getNumJoints(self._gripper_body_id)):
    #         p.changeDynamics(self._gripper_body_id, i, lateralFriction=1.0, spinningFriction=1.0,
    #                          rollingFriction=0.0001, frictionAnchor=True)
    #     self.step_simulation(50)

    # def execute_grasp(self, grasp_position, grasp_angle):
    #     """
    #         Execute grasp sequence
    #         @param: grasp_position: 3d position of place where the gripper jaws will be closed
    #         @param: grasp_angle: angle of gripper before executing grasp from positive x axis in radians 
    #     """
    #     # Adjust grasp_position to account for end-effector length
    #     grasp_position = grasp_position + self._tool_tip_to_ee_joint
    #     gripper_orientation = p.getQuaternionFromEuler(
    #         [np.pi, 0, grasp_angle])
    #     pre_grasp_position_over_bin = grasp_position+np.array([0, 0, 0.3])
    #     pre_grasp_position_over_object = grasp_position+np.array([0, 0, 0.1])
    #     post_grasp_position = grasp_position+np.array([0, 0, 0.3])
    #     grasp_success = False
    #     # ========= PART 2============
    #     # Implement the following grasp sequence:
    #     # 1. open gripper
    #     # 2. Move gripper to pre_grasp_position_over_bin
    #     # 3. Move gripper to pre_grasp_position_over_object
    #     # 4. Move gripper to grasp_position
    #     # 5. Close gripper
    #     # 6. Move gripper to post_grasp_position
    #     # 7. Move robot to robot_home_joint_config
    #     # 8. Detect whether or not the object was grasped and return grasp_success
    #     # ============================
    #     self.open_gripper()
    #     self.move_tool(pre_grasp_position_over_bin, None)
    #     self.move_tool(pre_grasp_position_over_object, gripper_orientation)
    #     self.move_tool(grasp_position, gripper_orientation)
    #     self.close_gripper()
    #     self.move_tool(post_grasp_position, None)
    #     self.robot_go_home(speed=0.01)
    #     grasp_success = self.check_grasp_success()
    #     return grasp_success
    
    # def execute_grasp_concat(self, grasp_position, grasp_angle, object_id):
    #     """
    #         concat dummy to make sure
    #         Execute grasp sequence
    #         @param: grasp_position: 3d position of place where the gripper jaws will be closed
    #         @param: grasp_angle: angle of gripper before executing grasp from positive x axis in radians 
    #     """
    #     # Adjust grasp_position to account for end-effector length
    #     grasp_position = grasp_position + self._tool_tip_to_ee_joint
    #     gripper_orientation = p.getQuaternionFromEuler(
    #         [np.pi, 0, grasp_angle])
    #     pre_grasp_position_over_bin = grasp_position+np.array([0, 0, 0.3])
    #     pre_grasp_position_over_object = grasp_position+np.array([0, 0, 0.1])
    #     post_grasp_position = grasp_position+np.array([0, 0, 0.3])
    #     grasp_success = False
    #     # ========= PART 2============
    #     # Implement the following grasp sequence:
    #     # 1. open gripper
    #     # 2. Move gripper to pre_grasp_position_over_bin
    #     # 3. Move gripper to pre_grasp_position_over_object
    #     # 4. Move gripper to grasp_position
    #     # 5. Close gripper
    #     # 6. Move gripper to post_grasp_position
    #     # 7. Move robot to robot_home_joint_config
    #     # 8. Detect whether or not the object was grasped and return grasp_success
    #     # ============================
    #     self.open_gripper()
    #     self.move_tool(pre_grasp_position_over_bin, None)
    #     self.move_tool(pre_grasp_position_over_object, gripper_orientation)
    #     self.move_tool(grasp_position, gripper_orientation)
    #     self.close_gripper(f = 0)
    #     grasp_success = self.check_grasp_success()
    #     if grasp_success == True:
    #         # 상대좌표 계산
    #         # tool_tip_pose = p.getLinkState(self.robot_body_id,self.robot_tool_tip)[0:2]
    #         # obj_pose = p.getBasePositionAndOrientation(object_id)
    #         # transform_pose = self.transfrom_pose(tool_tip_pose, obj_pose)
    #         # concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #         #                 0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=transform_pose[0], childFrameOrientation=transform_pose[1])
    #         concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0,0,0], childFrameOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
    #         print("concat")
    #         self.hold_gripper()
    #         self.step_simulation(50, sleep_time=0.0001)
    #     self.move_tool(post_grasp_position, None)
    #     self.robot_go_home(speed=0.03)
        

    #     self.step_simulation(50, sleep_time=0.0001)
        
        
    #     if grasp_success == True:
    #         p.removeConstraint(concat_id)
    #     self.open_gripper()
    #     self.step_simulation(50, sleep_time=0.0001)

    #     return grasp_success
    

    def ur5_concat_dummy(self, grasp_position, grasp_angle, object_id):
        """
            hold dummy and release dummy
        """
        # self.open_gripper()
        concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0,0,0], childFrameOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, 0]))
        # self.close_gripper(f = 0)
        # print("hold")
        # self.hold_gripper()
        self.step_simulation(100, sleep_time=0.0001)
        
        p.removeConstraint(concat_id)
        # self.open_gripper()
        
        
        self.step_simulation(100, sleep_time=0.0001)




       

    
    # None gripper
    # def execute_grasp_concat2(self, grasp_position, grasp_angle, object_id):
    #     """
    #         concat dummy to make sure
    #         Execute grasp sequence
    #         @param: grasp_position: 3d position of place where the gripper jaws will be closed
    #         @param: grasp_angle: angle of gripper before executing grasp from positive x axis in radians 
    #     """
    #     # Adjust grasp_position to account for end-effector length
    #     grasp_position = grasp_position + self._tool_tip_to_ee_joint
    #     gripper_orientation = p.getQuaternionFromEuler(
    #         [np.pi, 0, grasp_angle])
    #     pre_grasp_position_over_bin = grasp_position+np.array([0, 0, 0.3])
    #     pre_grasp_position_over_object = grasp_position+np.array([0, 0, 0.1])
    #     post_grasp_position = grasp_position+np.array([0, 0, 0.3])
    #     grasp_success = False
    #     # ========= PART 2============
    #     # Implement the following grasp sequence:
    #     # 1. open gripper
    #     # 2. Move gripper to pre_grasp_position_over_bin
    #     # 3. Move gripper to pre_grasp_position_over_object
    #     # 4. Move gripper to grasp_position
    #     # 5. Close gripper
    #     # 6. Move gripper to post_grasp_position
    #     # 7. Move robot to robot_home_joint_config
    #     # 8. Detect whether or not the object was grasped and return grasp_success
    #     # ============================
    #     # self.open_gripper()
    #     self.move_tool(pre_grasp_position_over_bin, None)
    #     self.move_tool(pre_grasp_position_over_object, gripper_orientation)
    #     self.move_tool(grasp_position, gripper_orientation)
    #     # self.close_gripper(f = 0)

       
    #     # p.createConstraint(self.robot_body_id, self.robot_end_effector_link_index, self._gripper_body_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #     #                    0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=self._robot_tool_offset, childFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi/2]))
    #     # self.robot_tool_tip

        


        
    #     # grasp_success = self.check_grasp_success()
    #     # if grasp_success == True:
    #     # add concat  [0, 0, -0.05]

    #     tool_tip_pose = p.getLinkState(self.robot_body_id,self.robot_tool_tip)[0:2]
    #     obj_pose = p.getBasePositionAndOrientation(object_id)

    
        
    #     transform_pose = self.transfrom_pose(tool_tip_pose, obj_pose)
    #     # concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #     #                 0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0,0,0], childFrameOrientation=p.getBasePositionAndOrientation(object_id)[1])

    #     concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #                     0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=transform_pose[0], childFrameOrientation=transform_pose[1])
    #     # concat_id = p.createConstraint(self.robot_body_id, self.robot_tool_tip, object_id, 0, jointType=p.JOINT_FIXED, jointAxis=[
    #     #             0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0,0,0], childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]))
    #     print("concat")
    #     self.step_simulation(100, sleep_time=0.001)
    #     self.move_tool(post_grasp_position, None)
    #     self.robot_go_home(speed=0.03)
        

    #     self.step_simulation(300, sleep_time=0.001)

    #     # if grasp_success == True:
    #     p.removeConstraint(concat_id)
    #     self.step_simulation(300, sleep_time=0.001)

    #     return grasp_success


    def execute_grasp_move_trajactory(self, grasp_position, grasp_angle, trajactory):
        """
            Execute grasp sequence
            @param: grasp_position: 3d position of place where the gripper jaws will be closed
            @param: grasp_angle: angle of gripper before executing grasp from positive x axis in radians 
        """
        # Adjust grasp_position to account for end-effector length
        grasp_position = grasp_position + self._tool_tip_to_ee_joint
        gripper_orientation = p.getQuaternionFromEuler(
            [np.pi, 0, grasp_angle])
        pre_grasp_position_over_bin = grasp_position+np.array([0, 0, 0.3])
        pre_grasp_position_over_object = grasp_position+np.array([0, 0, 0.1])
        post_grasp_position = grasp_position+np.array([0, 0, 0.3])
        grasp_success = False
        # ========= PART 2============
        # Implement the following grasp sequence:
        # 1. open gripper
        # 2. Move gripper to pre_grasp_position_over_bin
        # 3. Move gripper to pre_grasp_position_over_object
        # 4. Move gripper to grasp_position
        # 5. Close gripper
        # 6. Move gripper to post_grasp_position
        # 7. Move robot to robot_home_joint_config
        # 8. Detect whether or not the object was grasped and return grasp_success
        # ============================
        self.open_gripper()
        self.move_tool(pre_grasp_position_over_bin, None)
        self.move_tool(pre_grasp_position_over_object, gripper_orientation)
        self.move_tool(grasp_position, gripper_orientation)
        self.close_gripper()
        self.move_tool(post_grasp_position, None)
        grasp_success = self.check_grasp_success()
        if grasp_success:
            for move_pose in trajactory:
                self.move_tool(move_pose[0], move_pose[1])

            # go trajactory
            return grasp_success
        else:
            self.robot_go_home(speed=0.03)
            return grasp_success


    def transfrom_pose(self, parent, child):
        parent_pose = parent[0]
        parent_ori  = parent[1]
        child_pose = child[0]
        child_ori  = child[1]
        return p.multiplyTransforms(parent_pose, parent_ori, child_pose, child_ori)
    
    
    # def close_gripper(self, f = 10000):
    #     p.setJointMotorControl2(
    #         self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=1, force=f)
    #     self.step_simulation(4e2)
        
    
    # def hold_gripper(self):
    #     motor_kp = 0.5
    #     motor_kd = 0.225
    #     motor_torque = 1.4
    #     motor_max_velocity = 5.9
         
    #     p.setJointMotorControl2(
    #         bodyIndex = self._gripper_body_id, jointIndex = 1, controlMode = p.POSITION_CONTROL, targetPosition=p.getJointState(self._gripper_body_id, 1)[0],
    #         positionGain=motor_kp, velocityGain=motor_kd, force=motor_torque, maxVelocity=motor_max_velocity

    #     )

    # def open_gripper(self):
    #     p.setJointMotorControl2(
    #         self._gripper_body_id, 1, p.VELOCITY_CONTROL, targetVelocity=-5, force=10000)
    #     self.step_simulation(4e2)
    def check_grasp_success(self):
        # return p.getJointState(self._gripper_body_id, 1)[0] < 0.834 - 0.001
        return p.getJointState(self._gripper_body_id, 1)[0] < 0.7
    

    def check_info(self):
        """
        object ID 및 기타 확인용
        """
        print("\n\n UR5 INFO")
        ur5 = self.robot_body_id
        n_joints = p.getNumJoints(ur5)
        print(f"n_joints: {n_joints}")
        jointNames = [p.getJointInfo(ur5, i)[1] for i in range(0,n_joints)]
        for i, n in enumerate(jointNames):
            print(i,n)


def get_tableau_palette():
    """
    returns a beautiful color palette
    :return palette (np.array object): np array of rgb colors in range [0, 1]
    """
    palette = np.array(
        [
            [89, 169, 79],  # green
            [156, 117, 95],  # brown
            [237, 201, 72],  # yellow
            [78, 121, 167],  # blue
            [255, 87, 89],  # red
            [242, 142, 43],  # orange
            [176, 122, 161],  # purple
            [255, 157, 167],  # pink
            [118, 183, 178],  # cyan
            [186, 176, 172]  # gray
        ],
        dtype=np.float
    )
    return palette / 255.


# class SphereMarker:
#     def __init__(self, position, radius=0.05, rgba_color=(1, 0, 0, 0.8), text=None, orientation=None, p_id=0):
#         self.p_id = p_id
#         position = np.array(position)
#         vs_id = p.createVisualShape(
#             p.GEOM_SPHERE, radius=radius, rgbaColor=rgba_color, physicsClientId=self.p_id)

#         self.marker_id = p.createMultiBody(
#             baseMass=0,
#             baseInertialFramePosition=[0, 0, 0],
#             baseCollisionShapeIndex=-1,
#             baseVisualShapeIndex=vs_id,
#             basePosition=position,
#             useMaximalCoordinates=False
#         )

#         self.debug_item_ids = list()
#         if text is not None:
#             self.debug_item_ids.append(
#                 p.addUserDebugText(text, position + radius)
#             )
        
#         if orientation is not None:
#             # x axis
#             axis_size = 2 * radius
#             rotation_mat = np.asarray(p.getMatrixFromQuaternion(orientation)).reshape(3,3)

#             # x axis
#             x_end = np.array([[axis_size, 0, 0]]).transpose()
#             x_end = np.matmul(rotation_mat, x_end)
#             x_end = position + x_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, x_end, lineColorRGB=(1, 0, 0))
#             )
#             # y axis
#             y_end = np.array([[0, axis_size, 0]]).transpose()
#             y_end = np.matmul(rotation_mat, y_end)
#             y_end = position + y_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, y_end, lineColorRGB=(0, 1, 0))
#             )
#             # z axis
#             z_end = np.array([[0, 0, axis_size]]).transpose()
#             z_end = np.matmul(rotation_mat, z_end)
#             z_end = position + z_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, z_end, lineColorRGB=(0, 0, 1))
#             )

#     def __del__(self):
#         p.removeBody(self.marker_id, physicsClientId=self.p_id)
#         for debug_item_id in self.debug_item_ids:
#             p.removeUserDebugItem(debug_item_id)






# class simul_dummy:
#     def __init__(self, position, radius=0.05, rgba_color=(1, 0, 0, 0.8), text=None, orientation=None, p_id=0):
#         self.p_id = p_id
#         position = np.array(position)
#         vs_id = p.createVisualShape(
#             p.GEOM_SPHERE, radius=radius, rgbaColor=rgba_color, physicsClientId=self.p_id)

#         self.marker_id = p.createMultiBody(
#             baseMass=0,
#             baseInertialFramePosition=[0, 0, 0],
#             baseCollisionShapeIndex=-1,
#             baseVisualShapeIndex=vs_id,
#             basePosition=position,
#             useMaximalCoordinates=False
#         )

#         self.debug_item_ids = list()
#         if text is not None:
#             self.debug_item_ids.append(
#                 p.addUserDebugText(text, position + radius)
#             )
        
#         if orientation is not None:
#             # x axis
#             axis_size = 2 * radius
#             rotation_mat = np.asarray(p.getMatrixFromQuaternion(orientation)).reshape(3,3)

#             # x axis
#             x_end = np.array([[axis_size, 0, 0]]).transpose()
#             x_end = np.matmul(rotation_mat, x_end)
#             x_end = position + x_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, x_end, lineColorRGB=(1, 0, 0))
#             )
#             # y axis
#             y_end = np.array([[0, axis_size, 0]]).transpose()
#             y_end = np.matmul(rotation_mat, y_end)
#             y_end = position + y_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, y_end, lineColorRGB=(0, 1, 0))
#             )
#             # z axis
#             z_end = np.array([[0, 0, axis_size]]).transpose()
#             z_end = np.matmul(rotation_mat, z_end)
#             z_end = position + z_end[:, 0]
#             self.debug_item_ids.append(
#                 p.addUserDebugLine(position, z_end, lineColorRGB=(0, 0, 1))
#             )

#     def __del__(self):
#         p.removeBody(self.marker_id, physicsClientId=self.p_id)
#         for debug_item_id in self.debug_item_ids:
#             p.removeUserDebugItem(debug_item_id)