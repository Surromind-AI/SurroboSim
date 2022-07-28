import pybullet as p
import pybullet_data
import numpy as np
import time
import json
import sys
sys.path.append('sim_model')
from SphereMarker_model import SphereMarker
# from sim_model.SphereMarker_model import SphereMarker
with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)


class dummy():
    def __init__(self,  pose = None, big_hole = 2, useFix = False):
        self.dummy_mesh = 'assets\dummy\_dummy_tip.urdf'
        if big_hole == 1:
            print('load _dummy_tip_Hole.urdf')
            self.dummy_mesh= 'assets\dummy\_dummy_tip_Hole.urdf'
        elif big_hole == 2:
            print('load _dummy_tip_Hole.urdf')
            self.dummy_mesh= 'assets\dummy\_dummy_tip_Hole2.urdf'

        # dummy setting
        self._useFixedBase = useFix

        # pose quat setting
        if pose == None: 
            self.pose = [1,1,1]
        else:
            self.pose = pose
        self.quat = [0,0,0,0]

        # dummy concat ur5 flag
        self.ur5_flag = False
        # 고정이 아닌경우 flag -> True 매 step마다 위치 설정
        if useFix == False:
            self.ur5_flag = True
        self.dummy_id = p.loadURDF(self.dummy_mesh, self.pose, p.getQuaternionFromEuler([0,0,0]), useFixedBase=self._useFixedBase)

        self.ur5_pose_quat = p.getLinkState(self.dummy_id,2)[0:2]

        self.tmp_sphere = None
    
    def obj_reset_pose(self, pose=None, quat=None, test=False):
        if pose == None:
            pose =self.pose
        else:
            self.pose = pose
        
        if quat == None:
            quat = self.quat
        else:
            self.quat = quat

        self.tmp_sphere = None
        if len(quat) == 3:
            # ori -> quat
            quat = p.getQuaternionFromEuler(quat)
        p.resetBasePositionAndOrientation(self.dummy_id, pose, quat)
        self.ur5_pose_quat = p.getLinkState(self.dummy_id,2)[0:2]

        if test == True:
            # red
            base_sphere = SphereMarker(position = pose, radius=0.01, rgba_color=(1, 0, 0, 0.5))

            # green
            ur5_sphere  = SphereMarker(position=self.ur5_pose_quat[0], radius=0.01, rgba_color=(0, 1, 0, 0.5))
            self.tmp_sphere = [base_sphere, ur5_sphere]
    
    def change_fix(self, fix = None) -> bool:
        # check input type


        # chekc input fix and self._useFixedBase are same
        # if fix == None:
        #     self._useFixedBase = (self._useFixedBase +1)%2
        
        # else:
        if fix == self._useFixedBase:
            print('fix and self._useFixedBase are same not change')
            return
    
        self._useFixedBase = (self._useFixedBase +1)%2
        




        # if not smae fix and self._useFixedBase delete  and reset
        dummy_pose_quat = p.getBasePositionAndOrientation(self.dummy_id)
        p.removeBody(self.dummy_id)
        
        self.dummy_id = p.loadURDF(self.dummy_mesh, dummy_pose_quat[0],dummy_pose_quat[1], useFixedBase=self._useFixedBase)

    
    def pose_info(self):
        dummy_hole = p.getBasePositionAndOrientation(self.dummy_id)
        dummy_center = p.getLinkState(self.dummy_id,2)[0:2]
        return dummy_hole, dummy_center

            


            

            

        
            


    def return_id(self):
        return self.dummy_id

    
    def __del__(self):
        p.removeBody(self.dummy_id)


        
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

    dummy = dummy()

    print('step simul start')
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.01)
    print('step simul end')
    dummy.obj_reset_pose([1,1,1], [0,0,0], test=True)
    print('change fix')
    # dummy.change_fix(False)

    # test ur5 pose reset
    # for i in range(10):
    #     if i%2 == 0:
    #         print('pose : 1,1,1')
    #         dummy.obj_reset_pose([1,1,1], [0,0,0], test=True)
    #     else:
    #         dummy.obj_reset_pose([1,1,1.5], [0,0,0], test=True)
    #         print('pose : 1,1,1.5')
    #     for _ in range(100):
    #         p.stepSimulation()
        
    #     time.sleep(1)


    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)

