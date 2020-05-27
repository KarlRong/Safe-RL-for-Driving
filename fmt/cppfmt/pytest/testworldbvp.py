import os
import sys

print("cwd: ", os.getcwd())
sys.path.append(os.getcwd())

import fmtbvp
import numpy as np
import matplotlib as mlp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def testDoubleBvpConnect():
    s0 = [0, 0, 0, 0, 0]
    s1 = [1, -1, 4, -1, 1]
    waypoints = fmtbvp.gen_trajectory_bvp(s0, s1, 20)
    x = []
    y = []
    t = []
    for waypoint in waypoints:
        x.append(waypoint[0])
        y.append(waypoint[1])
        t.append(waypoint[4])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x, y, t)
    plt.show()


def testDoubleBvpCost():
    N = 10 ** 5
    # x_min = -3
    # x_max = 3
    # y_min = -3
    # y_max = 3
    # vx_min = -3
    # vx_max = 2.5
    # vy_min = -3
    # vy_max = 2.5
    # t_min = 0
    # t_max = 50
    x_min = 0
    x_max = 30
    y_min = 0
    y_max = 10.5
    vx_min = -10
    vx_max = 10
    vy_min = -2
    vy_max = 2
    t_min = 0
    t_max = 2.5

    s_set = np.empty((N, 5))
    s_set[:, 0] = np.random.default_rng().uniform(x_min, x_max, N)
    s_set[:, 1] = np.random.default_rng().uniform(y_min, y_max, N)
    s_set[:, 2] = np.random.default_rng().uniform(vx_min, vx_max, N)
    s_set[:, 3] = np.random.default_rng().uniform(vy_min, vy_max, N)
    s_set[:, 4] = np.random.default_rng().uniform(t_min, t_max, N)

    s_c = np.array([10, 2, 7, 0, 1])
    idxset = [x for x in range(0, N)]

    idx_for, trash, _ = fmtbvp.filter_reachable_bvp(s_set, idxset, s_c, 30, 10, 1, 50, True)
    idx_back, trash, _ = fmtbvp.filter_reachable_bvp(s_set, idxset, s_c, 30, 10, 1, 50, False)
    s_for = []
    for idx in idx_for:
        s_for.append(s_set[idx])
    s_back = []
    for idx in idx_back:
        s_back.append(s_set[idx])

    setA = np.zeros((5, len(idx_for)))
    setB = np.zeros((5, len(idx_back)))
    for i in range(0, len(idx_for)):
        setA[:, i] = s_for[i]

    for i in range(0, len(idx_back)):
        setB[:, i] = s_back[i]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(s_set[:, 0], s_set[:, 1], s_set[:, 4])
    ax.scatter(setA[0], setA[1], setA[4])
    ax.scatter(setB[0], setB[1], setB[4], marker='x')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')

    ax.set_title('reachable box, x y t')
    fig1 = plt.figure()

    ax1 = fig1.add_subplot(111, projection='3d')
    # ax.scatter(s_set[:, 0], s_set[:, 1], s_set[:, 4])
    ax1.scatter(setA[2], setA[3], setA[4])
    ax1.scatter(setB[2], setB[3], setB[4], marker='x')
    ax1.set_xlabel('vx')
    ax1.set_ylabel('vy')
    ax1.set_zlabel('t')

    ax1.set_title('reachable box, vx vy t')
    plt.show()
    return setA


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
    for objshape in world.objShapes:
        plotObj(objshape, world.timestep, ax)


def testWorldBvp():
    obstacle_set = []
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 0])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 1])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 2])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 3])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 4])

    xmin = 0
    xmax = 1
    ymin = 0
    ymax = 1
    vxmin = -0.5
    vxmax = 0.5
    vymin = -0.5
    vymax = 0.5
    tmin = 0
    tmax = 10
    w = 0.01  # ego形状
    h = 0.01
    timestep = 1

    world = fmtbvp.WorldBvp(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w,
                            h)
    N = 3000
    s_set = np.empty((N, 5))
    s_set[:, 0] = np.random.default_rng().uniform(low=xmin, high=xmax, size=N)
    s_set[:, 1] = np.random.default_rng().uniform(low=ymin, high=ymax, size=N)
    s_set[:, 2] = np.random.default_rng().uniform(low=vxmin, high=vxmax, size=N)
    s_set[:, 3] = np.random.default_rng().uniform(low=vymin, high=vymax, size=N)
    s_set[:, 4] = np.random.default_rng().uniform(low=tmin, high=tmax, size=N)

    valid_idx = []
    for i in range(0, N):
        if world.isValidState(s_set[i]):
            valid_idx.append(i)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # plotWorld(world, ax)
    ax.scatter(s_set[valid_idx, 0], s_set[valid_idx, 1], s_set[valid_idx, 4])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('t')

    ax.set_title('reachable box, x y t')
    plt.show()


def plotTraj(s0, s1, ax, color="gray", linewidth=2, N=20):
    waypoints = fmtbvp.gen_trajectory_bvp(s0, s1, 20)
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
    for idx in reversed(res):
        plotTraj(fmt.Pset[fmt.parent[idx]], fmt.Pset[idx], ax, color='blue', linewidth=5, N=20)


def testFmtBvp():
    obstacle_set = []
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 0])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 1])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 2])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 3])
    obstacle_set.append([0.5, 0.5, 0.6, 0.6, 4])

    xmin = 0
    xmax = 5
    ymin = 0
    ymax = 5
    vxmin = -2.5
    vxmax = 3.5
    vymin = -2.5
    vymax = 3.5
    tmin = 0
    tmax = 10
    w = 0.01  # ego形状
    h = 0.01
    timestep = 1

    world = fmtbvp.WorldBvp(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w,
                            h)
    N = 4000
    s_init = [0.1, 0.1, 0, 0, 1]
    s_goal = [2.9, 2.9, 0, 0, 9]

    fmt = fmtbvp.FMTreeBvp(s_init, s_goal, N, world, False)
    fmt.ux_limit = 5
    fmt.uy_limit = 5
    fmt.T_limit = 2
    fmt.r = 20
    fmt.solve()
    print(fmt.getResult())
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plotFmt(fmt, ax)
    plt.show()


def testFmtBvpRoad():
    obstacle_set = []
    obstacle_set.append([15, 1.7, 4, 2.5, 0])
    obstacle_set.append([16.4, 1.7, 4, 2.5, 0.2])
    obstacle_set.append([17.8, 1.7, 4, 2.5, 0.4])
    obstacle_set.append([19.2, 1.7, 4, 2.5, 0.6])
    obstacle_set.append([20.6, 1.7, 4, 2.5, 0.8])
    obstacle_set.append([22, 1.7, 4, 2.5, 1])
    obstacle_set.append([22.4, 1.7, 4, 2.5, 1.2])
    obstacle_set.append([23.8, 1.7, 4, 2.5, 1.4])
    obstacle_set.append([25.2, 1.7, 4, 2.5, 1.6])
    obstacle_set.append([26.6, 1.7, 4, 2.5, 1.8])
    obstacle_set.append([28, 1.7, 4, 2.5, 2])
    obstacle_set.append([29.4, 1.7, 4, 2.5, 2])
    obstacle_set.append([30.8, 1.7, 4, 2.5, 2.2])

    # obstacle_set.append([11, 1.7, 4, 2.5, 0])
    # obstacle_set.append([11, 1.7, 4, 2.5, 0.2])
    # obstacle_set.append([11, 1.7, 4, 2.5, 0.4])
    # obstacle_set.append([11, 1.7, 4, 2.5, 0.6])
    # obstacle_set.append([11, 1.7, 4, 2.5, 0.8])
    # obstacle_set.append([11, 1.7, 4, 2.5, 1])
    # obstacle_set.append([11, 1.7, 4, 2.5, 1.2])
    # obstacle_set.append([11, 1.7, 4, 2.5, 1.4])
    # obstacle_set.append([11, 1.7, 4, 2.5, 1.6])
    # obstacle_set.append([11, 1.7, 4, 2.5, 1.8])
    # obstacle_set.append([11, 1.7, 4, 2.5, 2])
    # obstacle_set.append([11, 1.7, 4, 2.5, 2])
    # obstacle_set.append([11, 1.7, 4, 2.5, 2.2])

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
    w = 4  # ego形状
    h = 2.5
    timestep = 0.2

    world = fmtbvp.WorldBvp(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w,
                            h)
    N = 800
    s_init = [8, 1.7, 7, 0, 0]
    s_goal = [27.5, 4.7, 7, 0, 2.5]

    fmt = fmtbvp.FMTreeBvp(s_init, s_goal, N, world, False)
    fmt.ux_limit = 5
    fmt.uy_limit = 5
    fmt.T_limit =5
    fmt.r = 200
    fmt.solve()
    print(fmt.getResult())
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plotFmt(fmt, ax)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(tmin, tmax)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("t")
    # ax.set_aspect('equal')
    plt.show()


if __name__ == "__main__":
    # testDoubleBvpConnect()
    # testDoubleBvpCost()
    # testWorldBvp()
    # testFmtBvp()
    testFmtBvpRoad()
