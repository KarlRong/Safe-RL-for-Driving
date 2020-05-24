
import torch
from torch.utils.data import Dataset, DataLoader

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

X_dim = 6
c_dim = 21
gridSize = 100
z_dim = 5

bs = 256

from data.AttentionDataset import AttentionDataset

test_loader = DataLoader(AttentionDataset(np_file_data = 'data/NarrowPassage/narrowDataOcc100.npz',
                            sample_dim = X_dim, 
                            condition_dim = c_dim,
                            gridSize = gridSize,
                            train = False),
                          batch_size = bs, shuffle=True, drop_last = True)

test_data = test_loader.dataset
viz_idx =   torch.randint(0,len(test_data),[1]).item()  
viz_idx = 8712
print(viz_idx)

_, con, startend, occ = test_data[viz_idx]
occ = occ[0].reshape(100,100)

def con2obs(con):
    dw = 0.1
    dimW = 3
    gap1 = con[0:3]
    gap2 = con[3:6]
    gap3 = con[6:9]

    obs1 = [0, gap1[1] - dw, -0.5, gap1[0], gap1[1], 1.5]
    obs2 = [gap2[0] - dw, 0, -0.5, gap2[0], gap2[1], 1.5]
    obs3 = [gap2[0] - dw, gap2[1] + dw, -0.5, gap2[0], 1, 1.5]
    obs4 = [gap1[0] + dw, gap1[1] - dw, -0.5, gap3[0], gap1[1], 1.5]
    obs5 = [gap3[0] + dw, gap1[1] - dw, -0.5, 1, gap1[1], 1.5]
    obs = np.concatenate((obs1, obs2, obs3, obs4, obs5), axis=0)
    return obs, dimW

import matplotlib.patches as patches
def pltObs(obs, dimW):
    fig1 = plt.figure(figsize=(30, 6), dpi=80)
    ax1 = fig1.add_subplot(131, aspect='equal')
    for i in range(0, obs.shape[0] // (2 * dimW)):  # plot obstacle patches
        ax1.add_patch(
            patches.Rectangle(
                (obs[i * 2 * dimW], obs[i * 2 * dimW + 1]),  # (x,y)
                obs[i * 2 * dimW + dimW] - obs[i * 2 * dimW],  # width
                obs[i * 2 * dimW + dimW + 1] - obs[i * 2 * dimW + 1],  # height
                alpha=0.6
            ))
    plt.show()

obs, dimW = con2obs(con)
# pltObs(obs, dimW)

import fcl

def obs2fcl(obs, dimW):
    obs_fcl = []
    for i in range(0, obs.shape[0] // (2 * dimW)):  # plot obstacle patches
        width = obs[i * 2 * dimW + dimW] - obs[i * 2 * dimW]
        height = obs[i * 2 * dimW + dimW + 1] - obs[i * 2 * dimW + 1]
    #     width = height = 1
        ob = fcl.Box(width, height, 1)
        x = obs[i * 2 * dimW] + width/2
        y = obs[i * 2 * dimW + 1] + height/2
    #     x = y = 0
        T = np.array([x, y, 0])
        t = fcl.Transform(T)
        co = fcl.CollisionObject(ob, t)
        obs_fcl.append(co)
#         print("x: ", x, " y: ", y, " width: ", width, " height: ", height)

    obs_manager = fcl.DynamicAABBTreeCollisionManager()
    obs_manager.registerObjects(obs_fcl)
    obs_manager.setup()
    return obs_manager

import sys
try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from math import sqrt
import argparse

# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")
        
from utils.NarrowPassage import gap2obs
def plt_ompl_result(condition, start, goal, path, collisionchecker, spaceinfo, planner):
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
    
    plannerdata = ob.PlannerData(spaceinfo)
    planner.getPlannerData(plannerdata)
    print("num edges: ", plannerdata.numEdges())
    print("num vertices: ", plannerdata.numVertices())
    num_ver = plannerdata.numVertices()
    num_edge = plannerdata.numEdges()
    if num_edge > 3000:
        print("too much edges")
        edge_vis = False
    else:
        edge_vis = True
    for i in range(0, num_ver):
        plt.scatter( plannerdata.getVertex(i).getState()[0], plannerdata.getVertex(i).getState()[1], color="green", s=50, edgecolors='black')  # vertice
        if edge_vis:
            for j in range(0, num_ver):
                if plannerdata.edgeExists(i, j):
                    plt.plot([plannerdata.getVertex(i).getState()[0], plannerdata.getVertex(j).getState()[0]], [plannerdata.getVertex(i).getState()[1], plannerdata.getVertex(j).getState()[1]], color='gray')
                
    path.interpolate()
    states = path.getStates()
    for state in states:
        plt.scatter(state[0], state[1], color="green", s=250, edgecolors='black')  # path
#     for state in collisionchecker.states_ok:
#         plt.scatter(state[0], state[1], color="green", s=100, edgecolors='green')  # free sample
#     for state in collisionchecker.states_bad:
#         plt.scatter(state[0], state[1], color="red", s=100, edgecolors='red')  # collision sample
    plt.scatter(start()[0], start()[1], color="blue", s=250, edgecolors='black')  # init
    plt.scatter(goal()[0], goal()[1], color="red", s=250, edgecolors='black')  # goal
    
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()
    
    
class MyStateSampler(ob.StateSampler):
    def __init__(self, space):
        super(MyStateSampler, self).__init__(space)
        self.rng_ = ou.RNG()
        
    def sampleUniform(self, state):

        x = self.rng_.uniformReal(0, 1)
        y = self.rng_.uniformReal(0, 1)
        xv = self.rng_.uniformReal(-1, 1)
        yv = self.rng_.uniformReal(-1, 1)
#         xv = 0
#         yv = 0
        state[0] = (float(x))
        state[1] = (float(y))
        state[2] = float(xv)
        state[3] = float(yv)
        return True
    
    def sampleUniformNear(self, state, nearstate, dis):
        print("sample uniform near")
        self.sampleUniform(state)
        
    def sampleGaussian(self, state, nearstate, dis):
        print("sample guassian")
        self.sampleUniform(state)
        

def allocMyStateSampler(space):
    return MyStateSampler(space,)

class ValidityChecker(ob.StateValidityChecker):
    def __init__(self,  si, fcl_manager, space):
        super(ValidityChecker, self).__init__(si)
        self.space = space
        self.fcl_manager = fcl_manager
        self.count = 0
        self.collision_count = 0
        self.states_ok = []
        self.states_bad = []
        # Returns whether the given state's position overlaps the
        # circular obstacle
    def isValid(self, state):
#         print("collision")
        sample = np.array([state[0], state[1], 0])
#         sample = np.array([state().getX(), state().getY(), state().getYaw()])
        req = fcl.CollisionRequest(num_max_contacts=100, enable_contact=True)
        rdata = fcl.CollisionData(request = req)

        cyl = fcl.Cylinder(0.01, 2)
        t = fcl.Transform(sample)
        agent = fcl.CollisionObject(cyl, t)

        self.fcl_manager.collide(agent, rdata, fcl.defaultCollisionCallback)
#         if(rdata.result.is_collision):
#             print("state: ", sample, " collision: ", rdata.result.is_collision)
#         print ('Collision between manager 1 and agent?: {}'.format(rdata.result.is_collision))
#         print( 'Contacts:')
#         for c in rdata.result.contacts:
#             print( '\tO1: {}, O2: {}'.format(c.o1, c.o2))
        self.count += 1
        if(rdata.result.is_collision):
            self.collision_count += 1
#             self.states_bad.append(sample)
#         else:
#             self.states_ok.append(sample)
#         return not rdata.result.is_collision
        return True

from doubleintegrator import cost_optimal
import math

def myDistance(st0, st1):
#     print("called")
    s0 = np.array([st0[0], st0[1], st0[2], st0[3]])
    s1 = np.array([st1[0], st1[1], st1[2], st1[3]])
    dis, tau = cost_optimal(s0, s1)
#     dis = abs(st1[0] - st0[0]) + abs(st1[1]-st0[0])
    print(dis)
    return float(dis)

class myStateSpace(ob.RealVectorStateSpace):
    
    def __init__(self):
        super().__init__(4)
        
    def distance(self, st0, st1):
        #     print("called")
        s0 = np.array([st0[0], st0[1], st0[2], st0[3]])
        s1 = np.array([st1[0], st1[1], st1[2], st1[3]])
        dis, tau = cost_optimal(s0, s1)
        # dis = abs(st1[0] - st0[0]) + abs(st1[1]-st0[1])
#         print(dis)
#         dis = 1
        return float(dis)

# fmt_star
sucess = 0
total = 0
total_length = 0
total_collision_detection = 0
total_collision = 0

for i in range(0,2):
    print("here")
#     state space
    space = ob.RealVectorStateSpace(4)
    space = myStateSpace()
#     space.distance = myDistance
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(4)
    bounds.setLow(0, 0)
    bounds.setHigh(0, 1)
    bounds.setLow(1, 0)
    bounds.setHigh(1, 1)
    bounds.setLow(2, -1)
    bounds.setHigh(2, 1)
    bounds.setLow(3, -1)
    bounds.setHigh(3, 1)
    space.setBounds(bounds)
    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    # set state validity checking for this space
    obs_manager = obs2fcl(obs, dimW)
    mychecker = ValidityChecker(si, obs_manager, space)
    si.setStateValidityChecker(mychecker)
    si.setStateValidityCheckingResolution(0.03)
    si.getStateSpace().setStateSamplerAllocator(ob.StateSamplerAllocator(allocMyStateSampler))
    si.setup()

# 起点终点
    start = ob.State(space)
    start.random()
    start[0] = float(startend[0])
    start[1] = float(startend[1])
    start[2] = float(0)
    start[3] = float(0)
    # create a random goal state
    goal = ob.State(space)
    goal.random()
    goal[0] = float(startend[6])
    goal[1] = float(startend[7])
    goal[2] = float(0)
    goal[3] = float(0)
    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    optimizingPlanner = allocatePlanner(si, 'fmtstar')
#     限制采1000个点
    optimizingPlanner.setNumSamples(2000)
    optimizingPlanner.setExtendedFMT(True)
#     optimizingPlanner.setFreeSpaceVolume(1)
#     optimizingPlanner.setRadiusMultiplier(5)
    # set the problem we are trying to solve for the planner
    optimizingPlanner.setProblemDefinition(pdef)
    # perform setup steps for the planner
    optimizingPlanner.setup()
    
    solved = optimizingPlanner.solve(60)
    print("collision detection: ", mychecker.count)
    total_collision_detection += mychecker.count
    print("collision num: ", mychecker.collision_count)
    total_collision += mychecker.collision_count

    total += 1
    if solved.asString() == 'Exact solution':
#     print(solved)
#     if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        if path:
            print("path length: ", pdef.getSolutionPath().length())
            total_length += pdef.getSolutionPath().length()
    #     print("Found solution:\n%s" % path)
            plt_ompl_result(con, start, goal,path, mychecker, si, optimizingPlanner)
        sucess += 1
    else:
        print("No solution found")
    optimizingPlanner.setup()

from termcolor import colored
print(colored("success rate: ", 'red'), sucess/total)
print(colored("average collision detection:", 'red'), total_collision_detection/total)
print(colored("average collision: ", "red"), total_collision/total)
if sucess > 0:
    print(colored("average length: ", "red"),  total_length/sucess)