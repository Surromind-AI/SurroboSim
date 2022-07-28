from assets.illias_bson import load_bson
from tqdm import tqdm
import json
import os
import pybullet as p
import math
import numpy as np
import matplotlib.pyplot as plt
import random
import copy

# DMPs
import sys
sys.path.append('./')
sys.path.append('./Trajactory/Movement_Primitives/')
# from Trajactory.Movement_Primitives.movement_primitives.dmp import DMP, CouplingTermPos3DToPos3D
from movement_primitives.dmp import DMP, CouplingTermPos3DToPos3D
from movement_primitives.dmp import DMPWithFinalVelocity
from movement_primitives.dmp import CartesianDMP

with open('assets/object_data.json', 'r', encoding='utf-8') as json_path:
    json_obj_path = json.load(json_path)


scale_factor = json_obj_path['Scalse']['scale_factor']        # 0.01
move_x       = json_obj_path['Scalse']['move_x']              # -500
move_y       = json_obj_path['Scalse']['move_y']              # 0
move_z       = json_obj_path['Scalse']['move_z']              # 0


def make_pose_ori(tmp_pose):
    pose = [(tmp_pose['loc']['x']+ move_x)*scale_factor, (tmp_pose['loc']['y']+ move_y)*scale_factor, (tmp_pose['loc']['z']+ move_z)*scale_factor]
    quat = [tmp_pose['quat']['x'], tmp_pose['quat']['y'], tmp_pose['quat']['z'], tmp_pose['quat']['w']]
    return [pose, quat]

def pose_shift(pose, x = 0, y = 0, z = 0):
    pose[0] += x
    pose[1] += y
    pose[2] += z
    return pose

    

def rotate_quant(quant, x = 0, y = 0, z = 0):
    """
    쿼터니언 입력
    ori로 변환 x y z 회전
    다시 쿼터니언
    """
    ori_Euler = list(p.getEulerFromQuaternion(quant))
    # rotate

    ori_Euler[0] += x
    ori_Euler[1] += y
    ori_Euler[2] += z

    # ori_Euler[2] += math.pi
    return p.getQuaternionFromEuler(ori_Euler)
    


def get_pose(obj_name):
    """
    bson 파일에서 trajactory load
    """
    obj_pose_list = []
    obj_name_list = ['MountingBar', 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy', 'Trajectory_Dummy_Wide', 'Trajectory_Dummy_Long', 'Trajectory_Dummy']
    # scale_factor = 0.005
    

    # BSON scenario path
    # action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'
    # bson_path   = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/SC1_HD_5.bson'

    # 시나리오 및 dummy 선택
    
    SC = json_obj_path['scenario']['SC']
    HD = json_obj_path['scenario']['HD']
    EP = json_obj_path['scenario']['EP']
    base_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items'
    bson_name = f'{SC}_{HD}_{EP}.bson'
    bson_path = os.path.join(base_path, bson_name)

    action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'

    trajact_data = get_bson_data(action_path, bson_path)
    start_pose = trajact_data[0]
    
    # MountingBar
    if obj_name == obj_name_list[0]:
        # obj_mesh = "assets\objects\SM_ShelfMetalHold_72.urdf"
        obj_mesh = json_obj_path['MountingBar']['urdf']
        MountingBar_start_pose = start_pose[obj_name_list[0]]

        for Mount_pose in MountingBar_start_pose:
            """
            MountingBar 가 반대로 되어 있어서 Euler 로 Quaternion 을 변환  Euler로 180도 회전 후 다시 Quaternion 으로 변환 
            """
            # make_pose_ori(Mount_pose)
            pose_ori = make_pose_ori(Mount_pose)
            pose = pose_ori[0]
            # rotate_quant 마운트 바 반대로 회전 방향 잘못되어 있음
            # obj_pose_list.append([pose, rotate_quant(pose_ori[1], x=-math.pi/2, z=-math.pi/2)])

            # pose_ori[1] = rotate_quant(pose_ori[1], x=-math.pi/2)
            
            obj_pose_list.append([pose, rotate_quant(pose_ori[1], x=-math.pi/2, y=-math.pi)])
            # obj_pose_list.append([pose_shift(pose,z = -0.11), rotate_quant(pose_ori[1],x=math.pi, y=-math.pi)])
            # obj_pose_list.append([pose, pose_ori[1]])
            # _ori =  pose_ori[1]
            # _ori_Euler = list(p.getEulerFromQuaternion(_ori))
            # # rotate
            # _ori_Euler[2] += math.pi
            # obj_pose_list.append([pose, p.getQuaternionFromEuler(_ori_Euler)])
            # obj_pose_list.append(make_pose_ori(Mount_pose))
    
    
    # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy' starting point
    elif obj_name == obj_name_list[1] or obj_name == obj_name_list[2] or obj_name == obj_name_list[3]:
        # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy'  obj_name 으로 받는다
        # dummy_pose['id']를 활용하여 3개중 받은 obj_name만 obj_pose에 append 하고 return 한다.
        obj_mesh = 'assets\dummy\dummy_long.obj'
        dummy_long_start_pose = start_pose['object_dummy']


        # obj data 에서 scenario 에서 HD 종류를 선택
        obj_name = json_obj_path['HD_select'][json_obj_path['scenario']['HD']]

                

        for dummy_pose in dummy_long_start_pose:
            if dummy_pose['id'] == obj_name:
                pose_ori = make_pose_ori(dummy_pose)
                obj_pose_list = [pose_ori[0], rotate_quant(pose_ori[1], x = math.pi/2)]
                obj_pose_list = [pose_ori[0], rotate_quant(pose_ori[1])]
        if obj_name == 'HangingDummyWide':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummy"]["urdf"]
        elif obj_name == 'HangingDummyLong':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummyLong"]["urdf"]
        elif obj_name == 'HangingDummy':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummy"]["urdf"]
        else:
            print("cannot load obj")
            obj_mesh = None

    # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy' trajactory     
    elif obj_name == obj_name_list[4] or obj_name == obj_name_list[5] or obj_name == obj_name_list[6]:
        dict_objname = dict()
        dict_objname['Trajectory_Dummy_Wide'] = 'HangingDummyWide'
        dict_objname['Trajectory_Dummy_Long'] = 'HangingDummyLong'
        dict_objname['Trajectory_Dummy'] = 'HangingDummy'
        for time_data in tqdm(trajact_data):
            dummy_poses = time_data['object_dummy']
            for dummy_pose in dummy_poses:
                if dummy_pose['id'] == dict_objname[obj_name]:
                    # pose = [(dummy_pose['loc']['x']+ move_x)*scale_factor, (dummy_pose['loc']['y']+ move_y)*scale_factor, (dummy_pose['loc']['z']+ move_z)*scale_factor]
                    # quat = [dummy_pose['quat']['x']*scale_factor, dummy_pose['quat']['y']*scale_factor, dummy_pose['quat']['z']*scale_factor, dummy_pose['quat']['w']*scale_factor]
                    # obj_pose_list.append([pose,quat])
                    obj_pose_list.append(make_pose_ori(dummy_pose))
        
        if obj_name == 'Trajectory_Dummy_Wide':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        elif obj_name == 'Trajectory_Dummy_Long':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        elif obj_name == 'Trajectory_Dummy':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        else:
            print("cannot load obj")
            obj_mesh = None
    
    # cannot load object 
    else:
        print(f"error return obj_pose_list None   get obj_name {obj_name}  but we have {obj_name_list} ")
        obj_pose_list= None
        obj_mesh = None

    return obj_pose_list, obj_mesh

def get_bson_data(action_path=None, bson_path=None):

    if action_path == None:
        action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'
    if bson_path == None:
        bson_path   = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/SC1_HD_5.bson'
    
    a = load_bson()
    time_data = a.get_timestep_data(bson_path, action_path)
    return time_data





def get_pose2(obj_name, SC='SC2', HD = 'HD', EP = '1'):
    """
    받은 변수를 통하여 데이터 로드
    """
    obj_pose_list = []
    obj_name_list = ['MountingBar', 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy', 'Trajectory_Dummy_Wide', 'Trajectory_Dummy_Long', 'Trajectory_Dummy']
    # scale_factor = 0.005
    

    # BSON scenario path
    # action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'
    # bson_path   = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/SC1_HD_5.bson'

    # 시나리오 및 dummy 선택

    # if scenario == 'SC1' or 'SC2':
    #     SC = scenario
    # else:
    #     assert f'wornd input : {scenario}'
    
    # SC = json_obj_path['scenario']['SC']
    # HD = json_obj_path['scenario']['HD']
    # EP = json_obj_path['scenario']['EP']
    base_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items'
    bson_name = f'{SC}_{HD}_{EP}.bson'
    bson_path = os.path.join(base_path, bson_name)

    action_path = 'ilias_vr_v2/dump/ILIAS_SC1&2_3Items/ILIAS_SC1%262_3Items.meta.bson'

    trajact_data = get_bson_data(action_path, bson_path)
    start_pose = trajact_data[0]
    
    # MountingBar
    if obj_name == obj_name_list[0]:
        # obj_mesh = "assets\objects\SM_ShelfMetalHold_72.urdf"
        obj_mesh = json_obj_path['MountingBar']['urdf']
        MountingBar_start_pose = start_pose[obj_name_list[0]]

        for Mount_pose in MountingBar_start_pose:
            """
            MountingBar 가 반대로 되어 있어서 Euler 로 Quaternion 을 변환  Euler로 180도 회전 후 다시 Quaternion 으로 변환 
            """
            # make_pose_ori(Mount_pose)
            pose_ori = make_pose_ori(Mount_pose)
            pose = pose_ori[0]
            # rotate_quant 마운트 바 반대로 회전 방향 잘못되어 있음
            # obj_pose_list.append([pose, rotate_quant(pose_ori[1], x=-math.pi/2, z=-math.pi/2)])

            # pose_ori[1] = rotate_quant(pose_ori[1], x=-math.pi/2)
            
            obj_pose_list.append([pose, rotate_quant(pose_ori[1], x=-math.pi/2, y=-math.pi)])
            # obj_pose_list.append([pose_shift(pose,z = -0.11), rotate_quant(pose_ori[1],x=math.pi, y=-math.pi)])
            # obj_pose_list.append([pose, pose_ori[1]])
            # _ori =  pose_ori[1]
            # _ori_Euler = list(p.getEulerFromQuaternion(_ori))
            # # rotate
            # _ori_Euler[2] += math.pi
            # obj_pose_list.append([pose, p.getQuaternionFromEuler(_ori_Euler)])
            # obj_pose_list.append(make_pose_ori(Mount_pose))
    
    
    # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy' starting point
    elif obj_name == obj_name_list[1] or obj_name == obj_name_list[2] or obj_name == obj_name_list[3]:
        # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy'  obj_name 으로 받는다
        # dummy_pose['id']를 활용하여 3개중 받은 obj_name만 obj_pose에 append 하고 return 한다.
        obj_mesh = 'assets\dummy\dummy_long.obj'
        dummy_long_start_pose = start_pose['object_dummy']


        # obj data 에서 scenario 에서 HD 종류를 선택
        obj_name = json_obj_path['HD_select'][json_obj_path['scenario']['HD']]

                

        for dummy_pose in dummy_long_start_pose:
            if dummy_pose['id'] == obj_name:
                pose_ori = make_pose_ori(dummy_pose)
                obj_pose_list = [pose_ori[0], rotate_quant(pose_ori[1], x = math.pi/2)]
                obj_pose_list = [pose_ori[0], rotate_quant(pose_ori[1])]
        if obj_name == 'HangingDummyWide':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummy"]["urdf"]
        elif obj_name == 'HangingDummyLong':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummyLong"]["urdf"]
        elif obj_name == 'HangingDummy':
            # obj_mesh = 'assets\dummy\dummy_long.obj'
            obj_mesh = json_obj_path["HangingDummy"]["urdf"]
        else:
            print("cannot load obj")
            obj_mesh = None

    # 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy' trajactory     
    elif obj_name == obj_name_list[4] or obj_name == obj_name_list[5] or obj_name == obj_name_list[6]:
        dict_objname = dict()
        dict_objname['Trajectory_Dummy_Wide'] = 'HangingDummyWide'
        dict_objname['Trajectory_Dummy_Long'] = 'HangingDummyLong'
        dict_objname['Trajectory_Dummy'] = 'HangingDummy'
        for time_data in tqdm(trajact_data):
            dummy_poses = time_data['object_dummy']
            for dummy_pose in dummy_poses:
                if dummy_pose['id'] == dict_objname[obj_name]:
                    # pose = [(dummy_pose['loc']['x']+ move_x)*scale_factor, (dummy_pose['loc']['y']+ move_y)*scale_factor, (dummy_pose['loc']['z']+ move_z)*scale_factor]
                    # quat = [dummy_pose['quat']['x']*scale_factor, dummy_pose['quat']['y']*scale_factor, dummy_pose['quat']['z']*scale_factor, dummy_pose['quat']['w']*scale_factor]
                    # obj_pose_list.append([pose,quat])
                    obj_pose_list.append(make_pose_ori(dummy_pose))
        
        if obj_name == 'Trajectory_Dummy_Wide':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        elif obj_name == 'Trajectory_Dummy_Long':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        elif obj_name == 'Trajectory_Dummy':
            obj_mesh = 'assets\dummy\dummy_long.obj'
        else:
            print("cannot load obj")
            obj_mesh = None
    
    # cannot load object 
    else:
        print(f"error return obj_pose_list None   get obj_name {obj_name}  but we have {obj_name_list} ")
        obj_pose_list= None
        obj_mesh = None

    return obj_pose_list, obj_mesh

def export_dmp2Trajactory(trajactory, Y):
    dmp_trajactory = []
    for idx, tr_data in enumerate(trajactory):
        if len(Y)> idx:
            dmp_trajactory.append([list(Y[idx]), tr_data[1]])
 
        
            
        

    return dmp_trajactory

def trajactory_to_linear(trajactory):
    t = np.linspace(0,(len(trajactory)+1), len(trajactory))
    # pose
    x=[]
    y=[]
    z=[]
    # 쿼터니언
    qw=[]
    qx=[]
    qy=[]
    qz=[]
    for data in trajactory:
        x.append(data[0][0])
        y.append(data[0][1])
        z.append(data[0][2])

        qw.append(data[1][0])
        qx.append(data[1][1])
        qy.append(data[1][2])
        qz.append(data[1][3])

    return x,y,z, qw, qx, qy, qz, t

def dmp_to_linear(trajactory):
    t = np.linspace(0,(len(trajactory)+1), len(trajactory))
    # pose
    x=[]
    y=[]
    z=[]

    for data in trajactory:
        x.append(data[0])
        y.append(data[1])
        z.append(data[2])



    return x,y,z, t

def trajactory_dmp(trajactory, random_xyz= None):
    t = np.linspace(0,(len(trajactory)+1), len(trajactory))
    # pose
    _pose = []
    # 쿼터니언
    _quat = []
    # pose_quat
    _pose_quat = []
    for data in trajactory:
        _pose.append(np.array(data[0]))
        _quat.append(np.array(data[1]))
        _pose_quat.append(np.array(data[0]+data[1]))
    start_pose = copy.deepcopy(_pose[400])
    if random_xyz != None:
        # xyz
        for i in range(3):
            start_pose[i] += random_xyz[i]


    return np.array(_pose), np.array(_quat), _pose_quat, start_pose, t



def trajactory_to_plot(trajactory):
    x,y,z, qw, qx, qy, qz, t = trajactory_to_linear(trajactory)

    plt.subplot(1,2,1)
    plt.title('Pose')
    plt.plot(t, x)
    plt.plot(t, y)
    plt.plot(t, z)
    
    
    plt.subplot(1,2,2)
    plt.title('Quaternion')
    plt.plot(t,qw)
    plt.plot(t,qx)
    plt.plot(t,qy)
    plt.plot(t,qz)

    plt.tight_layout()
    plt.show()

def compare_trajactory_to_plot(trajactory, g_trajactory):
    x,y,z, qw, qx, qy, qz, t = trajactory_to_linear(trajactory)
    g_x,g_y,g_z, g_qw, g_qx, g_qy, g_qz, g_t = trajactory_to_linear(g_trajactory)

    plt.subplot(1,2,1)
    plt.title('Pose')
    plt.plot(t, x, 'y')
    plt.plot(t, y, 'g')
    plt.plot(t, z, 'b')

    plt.plot(t, g_x, 'r')
    plt.plot(t, g_y, 'r')
    plt.plot(t, g_z, 'r')
    
    
    plt.subplot(1,2,2)
    plt.title('Quaternion')
    plt.plot(t,qw)
    plt.plot(t,qx)
    plt.plot(t,qy)
    plt.plot(t,qz)

    plt.tight_layout()
    plt.show()

def compare_dmp_to_plot(_pose, Y):
    x,y,z,t = dmp_to_linear(_pose)
    d_x,d_y,d_z,d_t = dmp_to_linear(Y)

    plt.subplot(1,2,1)
    plt.title('Pose')
    plt.plot(t, x, 'y')
    plt.plot(t, y, 'g')
    plt.plot(t, z, 'b')

    plt.plot(d_t, d_x, 'r')
    plt.plot(d_t, d_y, 'r')
    plt.plot(d_t, d_z, 'r')
    
    
    # plt.subplot(1,2,2)
    # plt.title('Quaternion')
    # plt.plot(t,qw)
    # plt.plot(t,qx)
    # plt.plot(t,qy)
    # plt.plot(t,qz)

    plt.tight_layout()
    plt.show()


def load_noise(len_data, noise_name = 'linear'):
    if noise_name == 'linear':
        selector = 1
    elif noise_name == 'trigonometric_func':
        selector = 2
    elif noise_name == 'test':
        selector = -1
    elif noise_name == 'random':
        selector = random.randint(1,2)
    else:
        print(f'wrond noise name : {noise_name}')

    if selector == 1:
        # linear
        x = list(np.ones(len_data,)*random.uniform(-0.1,0.1))
        y = list(np.ones(len_data,)*random.uniform(-0.1,0.1))
        z = list(np.ones(len_data,)*random.uniform(0.1,0.5))
        noise_data = [x, y, z]
    elif selector == 2:
        noise_data = []
        for i in range(3):
            tri_selector = random.randint(1, 3)
            tri_n_graph = random.randint(1,5)
            if tri_selector == 1:
                # make sin
                tmp_data = np.sin(np.linspace(0,tri_n_graph*np.pi, num = len_data))
            elif tri_selector == 2:
                # make sin
                tmp_data = np.sin(np.linspace(0,tri_n_graph*np.pi, num = len_data))
            else:
                tmp_data = np.ones(len_data,)*random.uniform(-0.1,0.1)
            
            # z pose positive number
            if i == 2:
                if min(tmp_data) <0:
                    tmp_data -= min(tmp_data)
            
            # scaling
            tmp_data *= 0.1/max(tmp_data)
            noise_data.append(list(tmp_data))

    elif selector == -1:
        x = list(np.ones(len_data,)*0.3)
        y = list(np.ones(len_data,)*0.3)
        z = list(np.ones(len_data,)*0.3)
        noise_data = [x, y, z]
    else:
        print(f'wrong noise_name your input : {noise_name}')

    return noise_data


def load_dmp(trajactory, tr_cut, method_dmp = ['dmp1'], show_plot = True, random_xyz = None):
    _pose, _quat, _pose_quat, start_pose, T = trajactory_dmp(trajactory, random_xyz)


    if 'dmp1' in method_dmp:
        # plot_coupling_3d_to_3d
        selector = 1
    
    elif 'dmp2' in method_dmp:
        # plot_dmp_with_final_velocity
        selector = 2
    
    elif 'dmp3' in method_dmp:
        # sim_cartesian_orientation_dmp
        selector = 3
    
    elif 'test_dmp' in method_dmp:
        selector = -1
    
    elif 'dmp_random' in method_dmp:
        selector = random.randint(1,2)
    
    else:
        selector = None
        print(f'Wrong DMP name : {method_dmp} just trajactory out')
        dmp_trajactory = trajactory
    dt = 0.01
    execution_time = 1.0
    execution_time = len(T)*dt

    if selector == 1:
        # 3D DMPs
        
        dmp = DMP(n_dims=3, execution_time=execution_time, dt=dt, n_weights_per_dim=10, int_dt=0.0001)
        # dmp = DMP(n_dims=7, execution_time=execution_time, dt=dt, n_weights_per_dim=10, int_dt=0.0001)
        ct = CouplingTermPos3DToPos3D(desired_distance=np.array([0.1, 0.5, 1.0]), lf=(0.0, 1.0), k=1.0, c1=30.0, c2=100.0)
        Y = _pose
        # Y = _pose_quat
        dmp.imitate(T, Y)
        # dmp.configure(start_y=Y[400], goal_y=Y[-1])
        dmp.configure(start_y = start_pose, goal_y=Y[-1])
        T, Y = dmp.open_loop()
        # T, Y = dmp.open_loop(coupling_term=ct)
        
        
        dmp_trajactory = export_dmp2Trajactory(trajactory, Y)
        # compare_trajactory_to_plot(trajactory, dmp_trajactory)
    
    elif selector == 2:
        Y = _pose
        # DMP with Final Velocity
        # execution_time = 10
        dmp = DMPWithFinalVelocity(n_dims=3, execution_time=execution_time)
        dmp.imitate(T, Y)

        
        dmp.imitate(T, Y)
        dmp.configure(start_y=Y[0], goal_y=Y[-1], goal_yd=np.array([0,0,0]))
        # T, Y = dmp.open_loop()
        T, Y = dmp.open_loop(run_t=execution_time)
        
        
        dmp_trajactory = export_dmp2Trajactory(trajactory, Y)
    elif selector == 3:
        dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=10, int_dt=0.0001)

        




    

    if show_plot == True and selector != None:
        compare_trajactory_to_plot(trajactory, dmp_trajactory)
        # compare_dmp_to_plot(_pose, Y)


    return dmp_trajactory


    

    


# Test util.py
if __name__ == '__main__':
    print("test util.py")
    # 'MountingBar', 'HangingDummyWide', 'HangingDummyLong', 'HangingDummy', 'Trajectory_Dummy_Wide', 'Trajectory_Dummy_Long', 'Trajectory_Dummy']
    tmp = get_pose('Trajectory_Dummy_Wide')


    # test pose
    mount_bar_list  = get_pose('MountingBar')[0]
    last_dummy_pose = get_pose('Trajectory_Dummy')[-1]
    
    print(mount_bar_list)
    print(last_dummy_pose)
    

