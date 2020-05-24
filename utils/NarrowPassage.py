import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
import numpy as np
import torch

def isSampleFree(sample, obs, dimW):
    for o in range(0, obs.shape[0] // (2 * dimW)):
        isFree = 0
        for d in range(0, sample.shape[0]):
            if (sample[d] < obs[2 * dimW * o + d] or sample[d] > obs[2 * dimW * o + d + dimW]):
                isFree = 1
                break
        if isFree == 0:
            return 0
    return 1


def gap2obs(condition):
    dw = 0.1
    dimW = 3
    gap1 = condition[0:3]
    gap2 = condition[3:6]
    gap3 = condition[6:9]

    obs1 = [0, gap1[1] - dw, -0.5, gap1[0], gap1[1], 1.5]
    obs2 = [gap2[0] - dw, 0, -0.5, gap2[0], gap2[1], 1.5]
    obs3 = [gap2[0] - dw, gap2[1] + dw, -0.5, gap2[0], 1, 1.5]
    obs4 = [gap1[0] + dw, gap1[1] - dw, -0.5, gap3[0], gap1[1], 1.5]
    obs5 = [gap3[0] + dw, gap1[1] - dw, -0.5, 1, gap1[1], 1.5]
    obsBounds = [-0.1, -0.1, -0.5, 0, 1.1, 1.5,
                 -0.1, -0.1, -0.5, 1.1, 0, 1.5,
                 -0.1, 1, -0.5, 1.1, 1.1, 1.5,
                 1, -0.1, -0.5, 1.1, 1.1, 1.5, ]
    obs = np.concatenate((obs1, obs2, obs3, obs4, obs5, obsBounds), axis=0)
    return obs, dimW


def getOccGrid(gridSize):
    gridPointsRange = np.linspace(0, 1, num=gridSize)
    occGridSamples = np.zeros([gridSize * gridSize, 2])
    idx = 0
    for i in gridPointsRange:
        for j in gridPointsRange:
            occGridSamples[idx, 0] = i
            occGridSamples[idx, 1] = j
            idx += 1
    return occGridSamples


def gap2occ(conditions, gridSize):
    obs, dimW = gap2obs(conditions)

    occGridSamples = getOccGrid(gridSize)
    occGrid = np.zeros(gridSize * gridSize)
    for i in range(0, gridSize * gridSize):
        occGrid[i] = isSampleFree(occGridSamples[i, :], obs, dimW)
    return occGrid


def plotCondition(condition):
    fig1 = plt.figure(figsize=(10, 6), dpi=80)
    ax1 = fig1.add_subplot(111, aspect='equal')
    obs, dimW = gap2obs(condition)
    for i in range(0, obs.shape[0] // (2 * dimW)):  # plot obstacle patches
        ax1.add_patch(
            patches.Rectangle(
                (obs[i * 2 * dimW], obs[i * 2 * dimW + 1]),  # (x,y)
                obs[i * 2 * dimW + dimW] - obs[i * 2 * dimW],  # width
                obs[i * 2 * dimW + dimW + 1] - obs[i * 2 * dimW + 1],  # height
                alpha=0.6
            ))
    gridSize = 11
    occGrid = gap2occ(condition, gridSize)

    occGridSamples = getOccGrid(gridSize)
    for i in range(0, gridSize * gridSize):  # plot occupancy grid
        if occGrid[i] == 0:
            plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="red", s=70, alpha=0.8)
        else:
            plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="green", s=70, alpha=0.8)
    init = condition[9:15]
    goal = condition[15:21]
    plt.scatter(init[0], init[1], color="red", s=250, edgecolors='black')  # init
    plt.scatter(goal[0], goal[1], color="blue", s=250, edgecolors='black')  # goal
    plt.show()


def plotSample(s, condition):
    fig1 = plt.figure(figsize=(10, 6), dpi=80)
    ax1 = fig1.add_subplot(111, aspect='equal')

    plt.scatter(s[:, 0], s[:, 1], color="green", s=70, alpha=0.1)

    obs, dimW = gap2obs(condition)
    for i in range(0, obs.shape[0] // (2 * dimW)):  # plot obstacle patches
        ax1.add_patch(
            patches.Rectangle(
                (obs[i * 2 * dimW], obs[i * 2 * dimW + 1]),  # (x,y)
                obs[i * 2 * dimW + dimW] - obs[i * 2 * dimW],  # width
                obs[i * 2 * dimW + dimW + 1] - obs[i * 2 * dimW + 1],  # height
                alpha=0.6
            ))
    gridSize = 11
    occGrid = gap2occ(condition, gridSize)
    occGridSamples = getOccGrid(gridSize)
    for i in range(0, gridSize * gridSize):  # plot occupancy grid
        if occGrid[i] == 0:
            plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="red", s=70, alpha=0.8)
        else:
            plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="green", s=70, alpha=0.8)

    init = condition[9:15]
    goal = condition[15:21]
    plt.scatter(init[0], init[1], color="red", s=250, edgecolors='black')  # init
    plt.scatter(goal[0], goal[1], color="blue", s=250, edgecolors='black')  # goal
    plt.show()


def plotSpeed(s, c):
    plt.figure(figsize=(10, 6), dpi=80)
    viz1 = 1
    viz2 = 4
    dim = 6
    plt.scatter(s[:, viz1], s[:, viz2], color="green", s=70, alpha=0.1)
    plt.scatter(c[viz1 + 9], c[viz2 + 9], color="red", s=250, edgecolors='black')  # init
    plt.scatter(c[viz1 + 9 + dim], c[viz2 + 9 + dim], color="blue", s=500, edgecolors='black')  # goal
    plt.show()


def plotSampleAttention(s, condition, occ, attention):
    fig1 = plt.figure(figsize=(30, 6), dpi=80)
    ax1 = fig1.add_subplot(131, aspect='equal')

    plt.scatter(s[:, 0], s[:, 1], color="green", s=70, alpha=0.1)

    obs, dimW = gap2obs(condition)
    for i in range(0, obs.shape[0] // (2 * dimW)):  # plot obstacle patches
        ax1.add_patch(
            patches.Rectangle(
                (obs[i * 2 * dimW], obs[i * 2 * dimW + 1]),  # (x,y)
                obs[i * 2 * dimW + dimW] - obs[i * 2 * dimW],  # width
                obs[i * 2 * dimW + dimW + 1] - obs[i * 2 * dimW + 1],  # height
                alpha=0.6
            ))
    gridSize = 11
    occGrid = gap2occ(condition, gridSize)
    occGridSamples = getOccGrid(gridSize)
#     for i in range(0, gridSize * gridSize):  # plot occupancy grid
#         if occGrid[i] == 0:
#             plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="red", s=70, alpha=0.8)
#         else:
#             plt.scatter(occGridSamples[i, 0], occGridSamples[i, 1], color="green", s=70, alpha=0.8)

    init = condition[9:15]
    goal = condition[15:21]
    plt.scatter(init[0], init[1], color="red", s=250, edgecolors='black')  # init
    plt.scatter(goal[0], goal[1], color="blue", s=250, edgecolors='black')  # goal
    ax1.set_xlim([-0.1, 1.1])
    ax1.set_ylim([-0.1, 1.1])

    # plot attention
    ax2 = fig1.add_subplot(132, aspect='equal')
    img = torch.from_numpy(attention).permute(1, 0)
    ax2.imshow(img)
    ax2.invert_yaxis()

    # plot occ
    ax3 = fig1.add_subplot(133, aspect='equal')
    occ = torch.from_numpy(occ).permute(1, 0)
    ax3.imshow(occ)
    ax3.invert_yaxis()
    plt.show()