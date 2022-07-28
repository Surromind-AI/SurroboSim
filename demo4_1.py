from __future__ import division
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



if __name__ == '__main__':
    random.seed(1)

    # object_shapes = None
    env = sim.PyBulletSim()
    # load gripper
    # env.load_gripper()
    u = urx_move(env)

    #Test UR Move ment
    # u.test_pose(3)





    # Grasping && Grasping with concat

    # set info when step simulation
    env.get_info(False)
    # u.start_concat(2)

    # u.start_concat(5)
    # u.test_Grasping(5)
    # u.start_concat(1)

    #Test UR Move ment
    # u.show_trajactory()

    # # Load Gripper && Grasping
    # u.test_trajactory_pose()
    
    
    # u.ur5_move_dummy(num_trials=2)
    # u.test_trajactory_pose()

    for idx in range(10):
        low = -0.01
        high = 0.01
        rand_xyz= [random.uniform(low, high), random.uniform(low, high), random.uniform(low, high)]
        data = u.ur5_trajactory_pose2(rand_xyz, scenario='SC2',  ep = '1')
        
        UR= UR5_Info_json()
        save_name = f'time_ur_data_{idx}.json'
        UR.data2json(data, json_name = save_name)
        tmp_data = UR.load_json()
        for data in tmp_data:
            print(data)
        
        print('episode end')






    print('end')
    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)