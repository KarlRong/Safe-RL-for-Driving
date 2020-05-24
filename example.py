import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from fmt.pythonfmt.world import World
from fmt.pythonfmt.fmt import FMTree
from fmt.pythonfmt.doubleintegrator import show_trajectory
import time

obstacle_set = [[0.2, 0.3, 0.2, 0.2],
                [0.5, 0.5, 0.2, 0.3],
                [0.8, 0.3, 0.2, 0.1],
                [0.8, 0.6, 0.15, 0.2],
                [0.2, 0.7, 0.1, 0.4],
                [0.2, 0.7, 0.1, 0.4]]
x_min = [0, 0]
x_max = [1.0, 1.0]
v_min = [-0.5, -0.5]
v_max = [0.5, 0.5]
W = World(x_min, x_max, v_min, v_max, obstacle_set)

s_init = [0.1, 0.5, 0.0, 0.0]
s_goal = [0.9, 0.9, 0.0, 0.0]

Nsample = 800

fmt = FMTree(s_init, s_goal, Nsample, W)


fig, ax = plt.subplots()  # Create a figure and an axes.
fmt.world.show(ax)
plt.pause(0.5)
draw = True
save = False
ts = time.time()
idx_solution = fmt.solve(ax, draw, save)
te = time.time()
print("solving time: ", te - ts)
if idx_solution.size == 2:
    print("solve failed")
else:
    print("solve success")
fmt.show(ax)
for idx in idx_solution:
    s0 = fmt.Pset[fmt.parent[idx]]
    s1 = fmt.Pset[idx]
    tau = fmt.time[idx]
    show_trajectory(s0, s1, tau, 20, ax, 'blue', 1.5)

plt.show()
plt.savefig("./fig/final.png")
