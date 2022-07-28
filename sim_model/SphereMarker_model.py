import pybullet as p
import pybullet_data
import numpy as np
import time
import json


with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)


class SphereMarker:
    def __init__(self, position=[0,0,0], radius=0.01, rgba_color=(1, 0, 0, 0.8), text=None, orientation=None, p_id=0):
        self.p_id = p_id
        position = np.array(position)
        vs_id = p.createVisualShape(
            p.GEOM_SPHERE, radius=radius, rgbaColor=rgba_color, physicsClientId=self.p_id)

        self.marker_id = p.createMultiBody(
            baseMass=0,
            baseInertialFramePosition=[0, 0, 0],
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=vs_id,
            basePosition=position,
            useMaximalCoordinates=False
        )

        self.debug_item_ids = list()
        if text is not None:
            self.debug_item_ids.append(
                p.addUserDebugText(text, position + radius)
            )
        
        if orientation is not None:
            # x axis
            axis_size = 2 * radius
            rotation_mat = np.asarray(p.getMatrixFromQuaternion(orientation)).reshape(3,3)

            # x axis
            x_end = np.array([[axis_size, 0, 0]]).transpose()
            x_end = np.matmul(rotation_mat, x_end)
            x_end = position + x_end[:, 0]
            self.debug_item_ids.append(
                p.addUserDebugLine(position, x_end, lineColorRGB=(1, 0, 0))
            )
            # y axis
            y_end = np.array([[0, axis_size, 0]]).transpose()
            y_end = np.matmul(rotation_mat, y_end)
            y_end = position + y_end[:, 0]
            self.debug_item_ids.append(
                p.addUserDebugLine(position, y_end, lineColorRGB=(0, 1, 0))
            )
            # z axis
            z_end = np.array([[0, 0, axis_size]]).transpose()
            z_end = np.matmul(rotation_mat, z_end)
            z_end = position + z_end[:, 0]
            self.debug_item_ids.append(
                p.addUserDebugLine(position, z_end, lineColorRGB=(0, 0, 1))
            )
    
    def obj_reset_pose(self, pose, quat):
        if len(quat) == 3:
            # ori -> quat
            quat = p.getQuaternionFromEuler(quat)
        p.resetBasePositionAndOrientation(self.marker_id, pose, quat)


    def __del__(self):
        p.removeBody(self.marker_id, physicsClientId=self.p_id)
        for debug_item_id in self.debug_item_ids:
            p.removeUserDebugItem(debug_item_id)

        

        
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



    
    sphere = SphereMarker()

        # test ur5 pose reset
    for i in range(10):
        if i%2 == 0:
            print('pose : 0,0,1')
            sphere.obj_reset_pose([0,0,1], [0,0,0])
        else:
            sphere.obj_reset_pose([0,0,1.5], [0,0,0])
            print('pose : 0,0,1.5')
        for _ in range(100):
            p.stepSimulation()
        
        time.sleep(1)


    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)



