import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import random

# x = np.arange(0,1.01,0.01)
# _x = norm(0.5,3).pdf(x)
# _x -= min(_x)
# _x *= 1/max(_x)

# plt.plot(x, _x)
# plt.show()


def make_gaussian():
    """
    끝단이 0이고 최고 값이 1인 가우시안 변형 함수
    """
    v = random.random()/3 +0.1
    print(f'v : {v}')


    x = np.arange(0,1.01,0.01)
    _x = norm(0.5,v).pdf(x)
    _x -= min(_x)
    _x *= 1/max(_x)
    return x, list(_x)

def xyz_gaussian():
    print('xyz gaussiina start')
    xyz = []
    for i in range(3):
        _, g_tmp = make_gaussian()
        xyz.append(g_tmp)
    print('xyz gaussiina end')
    return xyz


if __name__ == '__main__':
    x, _x = make_gaussian()
    plt.plot(x, _x)
    plt.show()


    
