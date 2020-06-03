import os
import sys

print("cwd: ", os.getcwd())
sys.path.append(os.getcwd())

import fmtltl
import numpy as np
import matplotlib as mlp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def testWorldLTL():
    roads = []
    roads.append([0, 100, 0, 3.5])
    roads.append([0, 100, 3.5, 7])
    roads.append([0, 100, 7, 10.5])
    roads.append([0, 100, 10.5, 14])
    cross = [70, 75, 0, 14]
    roadLtl = fmtltl.RoadLTL(roads, cross)

    vehs = {}
    w = 4
    h = 2.5
    states = []
    states.append([15, 1.7, 0])
    states.append([16.4, 1.7, 0.2])
    states.append([17.8, 1.7, 0.4])
    states.append([19.2, 1.7, 0.6])
    states.append([20.6, 1.7, 0.8])
    states.append([22, 1.7, 1])
    states.append([23.8, 1.7, 1.4])
    states.append([25.2, 1.7, 1.6])
    states.append([26.6, 1.7, 1.8])
    states.append([28, 1.7, 2])
    states.append([29.4, 1.7, 2])
    states.append([30.8, 1.7, 2.2])
    states.append([32, 1.7, 2.4])
    obj = fmtltl.ObjLTL(states, w, h)
    vehs[0] = obj
    humans = []

    xmin = 5
    xmax = 30
    ymin = 0
    ymax = 7
    vxmin = -2.5
    vxmax = 7
    vymin = -2
    vymax = 2
    tmin = 0
    tmax = 2.5
    w = 0.01  # ego形状
    h = 0.01
    timestep = 0.2
    traffic_light = False

    world = fmtltl.WorldLTL(vehs, humans, roadLtl, traffic_light, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax,
                            tmin, tmax,
                            timestep, w,
                            h)
    s_init = [8, 1.7, 7, 0, 0]
    s_goal = [27.5, 4.7, 7, 0, 2.5]
    nearby = world.setNearbyVehIds(s_init)
    print("front: ", nearby[0], "right: ", nearby[1], "left", nearby[2])
    pro = world.getProposition(s_goal)

    for i in range(0, 2000):
        state = world.sampleValid()
        pro = world.getProposition(state)


def plotObj(obj, timestep, ax, color="blue"):
    x0 = obj[0] - obj[2] / 2
    x1 = obj[0] + obj[2] / 2
    y0 = obj[1] - obj[3] / 2
    y1 = obj[1] + obj[3] / 2
    z0 = obj[4]
    z1 = obj[4] + timestep
    X = np.array([[x0, x1], [x0, x1]])
    Y = np.array([[y0, y0], [y1, y1]])
    Z = np.array([[z0, z0], [z0, z0]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)
    X = np.array([[x0, x1], [x0, x1]])
    Y = np.array([[y0, y0], [y1, y1]])
    Z = np.array([[z1, z1], [z1, z1]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)
    X = np.array([[x0, x0], [x0, x0]])
    Y = np.array([[y0, y1], [y0, y1]])
    Z = np.array([[z0, z0], [z1, z1]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)
    X = np.array([[x1, x1], [x1, x1]])
    Y = np.array([[y0, y1], [y0, y1]])
    Z = np.array([[z0, z0], [z1, z1]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)
    X = np.array([[x0, x1], [x0, x1]])
    Y = np.array([[y0, y0], [y0, y0]])
    Z = np.array([[z0, z0], [z1, z1]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)
    X = np.array([[x0, x1], [x0, x1]])
    Y = np.array([[y1, y1], [y1, y1]])
    Z = np.array([[z0, z0], [z1, z1]])
    surf = ax.plot_wireframe(X, Y, Z, color=color,
                             linewidth=1, antialiased=True)


def plotWorld(world, ax):
    # Make data.
    for veh in world.vehs:
        for state in world.vehs[veh].states:
            plotObj([state[0], state[1], world.vehs[veh].w, world.vehs[veh].h, state[2]], world.timestep, ax)


def plotTraj(s0, s1, ax, color="gray", linewidth=2, N=20):
    waypoints = fmtltl.gen_trajectory_bvp(s0, s1, 20)
    x = []
    y = []
    t = []
    for waypoint in waypoints:
        x.append(waypoint[0])
        y.append(waypoint[1])
        t.append(waypoint[4])
    ax.plot(x, y, t, color=color, linewidth=linewidth)

def plotStatesXYT(states, ax, color="blue", marker="o", size=2):
    x = []
    y = []
    t = []
    for state in states:
        x.append(state[0])
        y.append(state[1])
        t.append(state[4])
    ax.scatter(x, y, t, color=color, marker=marker, s=size)

def plotFmt(fmt, ax):
    plotWorld(fmt.world, ax)

    opens = []
    for idx in fmt.open:
        opens.append(fmt.Pset[idx])
        plotTraj(fmt.Pset[fmt.parent[idx]], fmt.Pset[idx], ax, color='gray', N=20)

    closeds = []
    for idx in fmt.closed:
        closeds.append(fmt.Pset[idx])
        plotTraj(fmt.Pset[fmt.parent[idx]], fmt.Pset[idx], ax, color='gray', N=20)

    unvisits = []
    for idx in fmt.unvisit:
        unvisits.append(fmt.Pset[idx])

    plotStatesXYT(opens, ax, color="blue", marker="o", size=2)
    plotStatesXYT(closeds, ax, color="black", marker="*", size=2)
    plotStatesXYT(unvisits, ax, color="gray", marker="1", size=1)

    ax.scatter(fmt.s_init[0], fmt.s_init[1], fmt.s_init[4], color="blue", marker="o", s=10)
    plotObj([fmt.s_init[0], fmt.s_init[1], 4, 2.5, fmt.s_init[4]], 0.2, ax, "red")
    ax.scatter(fmt.s_goal[0], fmt.s_goal[1], fmt.s_goal[4], color="blue", marker="o", s=10)
    plotObj([fmt.s_goal[0], fmt.s_goal[1], 4, 2.5, fmt.s_goal[4]], 0.2, ax, "red")
    res = fmt.getResult()
    print("res: ", res)
    for idx in reversed(res):
        plotTraj(fmt.Pset[fmt.parent[idx]], fmt.Pset[idx], ax, color='blue', linewidth=5, N=20)



def testFmtLTL():
    roads = []
    roads.append([0, 200, 43, 46.5])
    roads.append([0, 200, 46.5, 50])
    roads.append([0, 200, 50, 53.5])
    roads.append([0, 200, 53.5, 57])
    cross = [97, 103, 43, 57]
    roadLtl = fmtltl.RoadLTL(roads, cross)

    vehs = {}
    w = 4
    h = 2.5
    states = []
    states.append([15, 1.7, 0])
    states.append([16.4, 1.7, 0.2])
    states.append([17.8, 1.7, 0.4])
    states.append([19.2, 1.7, 0.6])
    states.append([20.6, 1.7, 0.8])
    states.append([22, 1.7, 1])
    states.append([23.8, 1.7, 1.4])
    states.append([25.2, 1.7, 1.6])
    states.append([26.6, 1.7, 1.8])
    states.append([28, 1.7, 2])
    states.append([29.4, 1.7, 2])
    states.append([30.8, 1.7, 2.2])
    states.append([32, 1.7, 2.4])
    obj = fmtltl.ObjLTL(states, w, h)
    vehs[0] = obj
    states2 = []
    for state in states:
        states2.append([state[0] - 15, state[1] + 3.5, state[2]])
    obj = fmtltl.ObjLTL(states2, w, h)
    vehs[1] = obj
    humans = []

    xmin = -5
    xmax = 30
    ymin = 40
    ymax = 53
    vxmin = -2.5
    vxmax = 10
    vymin = -2
    vymax = 2
    tmin = 0
    tmax = 2.5
    w = 5  # ego形状
    h = 2.7
    timestep = 0.2
    traffic_light = False
    vehs = {}
    world = fmtltl.WorldLTL(vehs, humans, roadLtl, traffic_light, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax,
                            tmin, tmax,
                            timestep, w,
                            h)
    s_init = [8, 1.7, 7, 0, 0]
    s_init = [0.6304907442532088, 45.997003932720304, -11.788565566306517, 0.5983146673632878, 0.0]
    s_goal = [5.630490744253208, 44.75, 2.0, 0.0, 2.5]
    nearby = world.setNearbyVehIds(s_init)
    print("front: ", nearby[0], "right: ", nearby[1], "left", nearby[2])

    wfas = fmtltl.WfaLTLs()
    wfas.addSwitch()
    # wfas.addKeepLane()
    wfas.addLcLeftTake()
    # wfas.addLcLeftGive()
    # wfas.addLcRightTake()

    Nsample = 1500
    exp_velo = 2
    lc = 0
    fmt = fmtltl.FMTreeLTL(s_init, exp_velo, lc, Nsample, world, wfas, False)
    fmt.s_init == s_init
    fmt.s_goal == s_goal
    fmt.ux_limit = 5
    fmt.uy_limit = 5
    fmt.T_limit = 5
    fmt.r = 300

    trace, cost = fmt.solve()
    print("s_goal", fmt.s_goal)

    print("trace: ", trace)
    print("total cost: ", fmt.cost[trace[0]])
    print("total ltl cost", cost)
    print("total speed cost", fmt.cost_speed[trace[0]])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plotFmt(fmt, ax)
    # plotWorld(world, ax)
    # ax.scatter(s_set[valid_idx, 0], s_set[valid_idx, 1], s_set[valid_idx, 4])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')

    ax.set_title('reachable box, x y t')
    plt.show()


if __name__ == "__main__":
    # testWorldLTL()
    testFmtLTL()
