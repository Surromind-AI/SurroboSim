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

# tmp_ur_joint = [0.5, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
tmp_ur_joint = [0.08532754708551023, -1.3648456364415258, 2.125011110996939, -3.017104280383585, -2.1702222406938665, 0.6638518050867872]
np_test = np.linspace(0.08532754708551023, np.pi, 100)
temp_step = []
for i in range(100):
    temp_step.append([np_test[i], -1.3648456364415258, 2.125011110996939, -3.017104280383585, -2.1702222406938665, 0.6638518050867872])



def main():
    rl = RL_Base()
    for idx in range(len(np_test)):

        # Test  RL reset
        print(f'test RL Test : {idx}')
        for step in range(20):

            rl.rl_step(temp_step[step])
            print(rl.get_state())
            rl.get_reward()
            time.sleep(1)


        # Test RL get status
        rl.rl_reset()
        


if __name__ == '__main__':
    
    print('Demo RL')
    main()