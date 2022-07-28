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


if __name__ == '__main__':
    random.seed(1)
    # object_shapes = None
    env = sim.PyBulletSim()

    for i in range(2):
        env.move_tool_shift([0,-1,1], [0,0,0])
        time.sleep(1)
        env.move_tool_shift([-1,-1,1], [0,0,0])
        time.sleep(1)
    # load gripper
    # env.load_gripper()
    # u = urx_move(env)
    t1 = [-3.57959168410233, -1.417154082158029, 1.2634603286177062, -1.819806438693751, -1.7258338316314157, 7.619975082911528e-06]
    t2 = [-5.55121719517353, -1.6677731531849143, 1.8633096443873136, -1.3286941133487384, -3.161662694394624, -1.1906592530637082e-05]
    for i in range(2):
        env.move_joints(t1)
        time.sleep(1)
        env.move_joints(t2)
        time.sleep(1)
    
    while True:
        # 시뮬레이션 유지
        p.stepSimulation()
        time.sleep(0.01)
    


    