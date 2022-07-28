"""
Test 
"""
import sys
sys.path.append('DMPs')
from pydmp import DMP, trajectory, potential_field
import numpy as np
import matplotlib.pyplot as plt

def clean_ax(ax, x_range, y_range, spines=False, legend=False):
    if legend:
        ax.legend(loc="upper left")
    if not spines:
        ax.spines["top"].set_visible(False)
        ax.spines["bottom"].set_visible(False)
        ax.spines["left"].set_visible(False)
        ax.spines["right"].set_visible(False)
    plt.setp(ax, xticks=(), yticks=(), xlim=x_range, ylim=y_range)
def main():
    x0 = np.array([0, 0], dtype=np.float64)
    g = np.array([1, 1], dtype=np.float64)
    tau = 1.0
    w = np.array([[-50.0, 100.0, 300.0],
                [-200.0, -200.0, -200.0]])
    o = np.array([1.0, 0.5])
    dt = 0.01

    dmp = DMP()

    x_range = (-0.2, 1.2)
    y_range = (-0.2, 1.2)
    n_tics = 10

    G, _, _ = trajectory(dmp, w, x0, g, tau, dt, o, shape=False, avoidance=False)
    T, _, _ = trajectory(dmp, w, x0, g, tau, dt, o, shape=True, avoidance=False)
    O, _, _ = trajectory(dmp, w, x0, g, tau, dt, o, shape=True, avoidance=True)

    fig = plt.figure(figsize=(5, 5))

    plt.plot(G[:, 0], G[:, 1], lw=3, color="g", label="Goal-directed")
    plt.plot(T[:, 0], T[:, 1], lw=3, color="r", label="Shaped")
    plt.plot(O[:, 0], O[:, 1], lw=3, color="black", label="Obstacle avoidance")

    plt.plot(x0[0], x0[1], "o", color="b", markersize=10)
    plt.plot(g[0], g[1], "o", color="g", markersize=10)
    plt.plot(o[0], o[1], "o", color="y", markersize=10)

    clean_ax(plt.gca(), x_range=x_range, y_range=y_range, legend=True)
    plt.show()


if __name__ == '__main__':
    print("test")
    main()