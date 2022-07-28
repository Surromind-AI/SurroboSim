from __future__ import division
import enum
import sys
sys.path.append('./')
from os import link
import sim as sim
import pybullet as p
import random
import numpy as np
import math
import time

from util import *
from ur5_move import urx_move

class UR5_Info_json():
    """
    ur5 time data load or save
    save default dir : UR_time_data
    """
    def __init__(self, data_dir = None):
        if data_dir == None:
            data_dir = 'UR_time_data'
        self._dir = data_dir
        self._data = None
        self._json_path = None
    
    def data2json(self, time_data, json_name = None):
        if json_name == None:
            json_name ='test_ur_data'

        json_path = os.path.join(self._dir, json_name)
        self._json_path = json_path
        with open(json_path, 'w') as outfile:
            json.dump(time_data, outfile, indent=2)

    def load_json(self, json_path = None):
        if json_path == None:
            json_path = self._json_path
        with open(json_path, "r") as json_file:
            json_data = json.load(json_file)
        return json_data





if __name__ == '__main__':
    random.seed(1)

    tmp = [1,2,3,4]


    time_data =dict()

    for idx, i in enumerate(range(20)):
        tmp[2] += idx
        time_data[idx] = tmp

    

    # time_data = np.array(time_data)
    # x = np.array([0, 1, 2, 3, 4, 'test'])

    time_data = [1,2,3]
    list_data = []
    for idx, i in enumerate(range(10)):
        list_data.append([idx,time_data])

    # ur_info = UR5_Info_npy()
    ur_info = UR5_Info_json()
    ur_info.data2json(list_data, 'data1.json')
    npy_path = 'C:\\Users\\leejm90\\Desktop\\surromind\\ilias_2p2_snu\\data1.npy"'
    data = ur_info.load_json()
    print(data)

