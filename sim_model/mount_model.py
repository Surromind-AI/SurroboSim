import pybullet as p
import pybullet_data
import numpy as np
import time
import json
import sys
sys.path.append('sim_model')
from SphereMarker_model import SphereMarker
with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)


class mount():
    def __init__(self):
        mount_mesh = 'assets\objects\SM_ShelfMetalHold_72_tip.urdf'
        self.pose = [0,0,1]
        self.quat = [-0.7066579277119857, -1.4320831565330634e-06, -1.4339018346182841e-06, -0.707555349918079]
        self.mount_id = p.loadURDF(mount_mesh, self.pose, self.quat, useFixedBase=True)
        # self.ur5_pose_quat = p.getLinkState(self.mount_id,2)[0:2]
        
 
        # shift pose
        tool_tip   = p.getLinkState(self.mount_id,1)[0:2]
        self.shift_pose = np.array(tool_tip[0]) - np.array(self.pose)
        # Confirm shift pose spere 
        self.tmp_sphere = None

    
    def obj_reset_pose(self, pose, quat = None):
        #reset sphre
        self.tmp_sphere = None
        self.pose = pose
        if quat == None:
            # 기본 quat 사용
            quat = self.quat

        if len(quat) == 3:
            # ori -> quat
            quat = p.getQuaternionFromEuler(quat)
        p.resetBasePositionAndOrientation(self.mount_id, pose, quat)
        # self.ur5_pose_quat = p.getLinkState(self.mount_id,2)[0:2]
    
    def obj_reset_tip(self, pose, quat=None, test=False):
        # reset sphere
        self.tmp_sphere = None
        if quat == None:
            # default quat 사용
            quat = self.quat

        if len(quat) == 3:
            # ori -> quat
            quat = p.getQuaternionFromEuler(quat)
        
        shift_pose =  pose - self.shift_pose 
        p.resetBasePositionAndOrientation(self.mount_id, shift_pose, quat)

        if test == True:
            # show Base pose by sphere
            shift_sphere = SphereMarker(position = pose, radius=0.02, rgba_color=(1, 0, 0, 0.1))
            tool_tip_sphere =SphereMarker(p.getLinkState(self.mount_id,1)[0:2][0], radius=0.01, rgba_color=(0, 1, 0, 0.5))
            self.tmp_sphere = [shift_sphere, tool_tip_sphere]

        
    def pose_info(self):
        mount_center   = p.getBasePositionAndOrientation(self.mount_id)
        mount_tool_tip = p.getLinkState(self.mount_id,1)[0:2]

        return mount_center, mount_tool_tip




            
        


        

        

        
    
    
        
    
    def return_id(self):
        return self.mount_id


    
    def __del__(self):
        p.removeBody(self.mount_id)


        
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

    mount = mount()

    # # test mount pose reset
    # for i in range(3):
    #     mount.obj_reset_pose([0,0,1])
    #     print('pose : 0,0,1')
    #     time.sleep(1)
    #     mount.obj_reset_pose([0,0,1.5])
    #     print('pose : 0,0,1.5')

    #     time.sleep(1)

    # test shift mount pose reset
    for i in range(1):       
        mount.obj_reset_tip([0,0,1],    test=True)
        print('pose : 0,0,1')
        time.sleep(1)
        mount.obj_reset_tip([0,0,1.5], test=True)
        print('pose : 0,0,1.5')

        time.sleep(1)

    print('finish')
    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)

