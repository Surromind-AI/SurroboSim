from __future__ import division
from os import link
import sim as sim
import pybullet as p
import random
import numpy as np
import math
import time

from util import *
from sim_model.urx_move_gaussina import ur5_move

# Not load gripper
from sim_model import * 



if __name__ == '__main__':
    random.seed(1)
    env = sim.PyBulletSim()
    u = ur5_move(env)
    


    for idx in range(10):

        data = u.ur5_trajactory_pose(scenario='SC2',  ep = '1')
        
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