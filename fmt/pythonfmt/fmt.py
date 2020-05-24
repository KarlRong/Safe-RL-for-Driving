import math
import random
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from fmt.pythonfmt.doubleintegrator import filter_reachable, gen_trajectory, show_trajectory
from fmt.pythonfmt.world import World


def dist2(p, q):
    return math.sqrt((p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2)


# FMTree class


class FMTree:
    # s_init::Vec4f
    # s_goal::Vec4f
    # N #number of samples
    # Pset::Vector{Vec4f} # Point set
    # cost::Vector{Float64} #cost
    # time::Vector{Float64} #optimal time to connect one node to its parent node
    # parent::Vector{Int64} #parent node
    # bool_unvisit::BitVector #logical value for Vunvisit
    # bool_open::BitVector #logical value for Open
    # bool_closed::BitVector #logical value for Closed
    # world::World # simulation world config
    # itr::Int64 # iteration num

    def __init__(self, s_init, s_goal, N, world):
        # constructer: sampling valid point from the configurationspace
        print("initializing fmt ...")
        self.s_init = s_init
        self.s_goal = s_goal
        self.N = N
        self.world = world
        self.Pset = np.zeros((N, 4))
        self.Pset[0, :] = np.array(s_init)

        def myrn(min, max):
            return min + (max - min) * random.random()

        # 采样N个点
        n = 1
        while True:
            num_ran = 2*N
            rp = np.empty((4, num_ran))
            rp[0, :] = np.random.default_rng().uniform(self.world.x_min[0], self.world.x_max[0], num_ran)
            rp[1, :] = np.random.default_rng().uniform(self.world.x_min[1], self.world.x_max[1], num_ran)
            rp[2, :] = np.random.default_rng().uniform(self.world.v_min[0], self.world.v_max[0], num_ran)
            rp[3, :] = np.random.default_rng().uniform(self.world.v_min[1], self.world.v_max[1], num_ran)

            # p = np.array([myrn(world.x_min[0], world.x_max[0]),
            #               myrn(world.x_min[1], world.x_max[1]),
            #               myrn(world.v_min[0], world.v_max[0]),
            #               myrn(world.v_min[1], world.v_max[1])])
            for i_rp in range(0, num_ran):
                if self.world.isValid(rp[:, i_rp]):
                    self.Pset[n, :] = rp[:, i_rp]
                    n = n + 1
                    if n == N-1:
                        break
            if n == N-1:
                break

        self.Pset[-1, :] = np.array(s_goal)  # inply idx_goal = N [last] ? 修改為最後一個是終點
        self.cost = np.zeros(N)
        self.time = np.zeros(N)
        self.parent = np.zeros(N, dtype=int)
        self.bool_unvisit = np.ones(N, dtype=np.bool_)
        self.bool_unvisit[0] = False
        self.bool_closed = np.zeros(N, dtype=np.bool_)
        self.bool_open = np.zeros(N, dtype=np.bool_)
        self.bool_open[0] = True
        self.itr = 0
        print("finish initializing")
        # new(s_init, s_goal,
        #     N, Pset, cost, time, parent, bool_unvisit, bool_open, bool_closed, world, 0)

    def show(self, ax):
        print("drawing...")
        # 先画障碍物
        N = len(self.Pset)
        mat = np.zeros((2, N))
        for idx in range(0, N):
            mat[:, idx] = self.Pset[idx, 0:2]

        idxset_open = np.nonzero(self.bool_open)[0]
        idxset_closed = np.nonzero(self.bool_closed)[0]
        idxset_unvisit = np.nonzero(self.bool_unvisit)[0]
        # idxset_tree = setdiff(union(idxset_open, idxset_closed), [1])
        idxset_tree = np.concatenate((idxset_closed, idxset_open))  # 没有和原来一样去除 id 1
        # 起点，重点，open, close
        ax.scatter(mat[0, 0], mat[1, 0], c='blue', s=20, zorder=100)
        ax.scatter(mat[0, -1], mat[1, -1], c='blue', s=20, zorder=101)
        ax.scatter(mat[0, idxset_open], mat[1, idxset_open], c='orange', s=5)
        ax.scatter(mat[0, idxset_closed], mat[1, idxset_closed], c='red', s=5)
        # ax.scatter(mat[0, idxset_unvisit], mat[1, idxset_unvisit], c='khaki', s=2)
        for idx in idxset_tree:
            s0 = self.Pset[self.parent[idx]]
            s1 = self.Pset[idx]
            tau = self.time[idx]
            show_trajectory(s0, s1, tau, N_split=5, ax=ax)

        # 起点重点画了第二次？
        # ax.scatter(mat[0, 1], mat[1, 1], c='blue', s=20, zorder=100)
        # ax.scatter(mat[0, -1], mat[1, -1], c='blue', s=20, zorder=101)

        # plt.xlim(this.world.x_min[1]-0.05, this.world.x_max[1]+0.05)
        # plt.ylim(this.world.x_min[2]-0.05, this.world.x_max[2]+0.05)
        print("finish drawing")

    def solve(self, ax=None, show=False, save=False):
        # keep extending the node until the tree reaches the goal
        print("please set with_savefig=false if you want to measure the computation time")
        print("start solving")
        while True:
            if not self.extend(): # 擴展失敗
                break
            # if ((self.itr < 100) and (self.itr % 20 == 1)) or (self.itr % 200 == 1):
            if self.itr % 40 == 1:
                print("itr: ", self.itr)
                if ax and show:
                        # close()
                        self.show(ax)
                        plt.pause(1)
                if ax and save:
                    plt.savefig("./fig/" + str(self.itr) + ".png")
                    # 这里需要通过传递fig解决
            if not self.bool_unvisit[-1]:
                break
        # 無法連接到終點的情況處理待定
        idx = -1
        idx_solution = [idx]
        while True:
            idx = self.parent[idx]
            idx_solution.append(idx)
            if idx == 0:
                break
        print("finish solving")
        return np.array(idx_solution)

    def extend(self):
        # extend node
        self.itr += 1
        r = 1.0  # 这是什么参数？

        # 此處數據結構可以優化, idxset_open和idxset_unvisit不用每次檢索
        idxset_open = np.nonzero(self.bool_open)[0] #這裡莫名返回一個tuple，需要取第一個
        if idxset_open.size == 0: #無法再繼續擴展
            return False
        idxset_unvisit = np.nonzero(self.bool_unvisit)[0]
        idx_lowest = idxset_open[np.argmin(self.cost[idxset_open])]
        # idx_lowest = idxset_open[findmin(this.cost[idxset_open])[2]]
        s_c = self.Pset[idx_lowest, :]
        idxset_near, _, _ = filter_reachable(self.Pset, idxset_unvisit,
                                             self.Pset[idx_lowest], r, "F")

        for idx_near in idxset_near:
            idxset_cand, distset_cand, timeset_cand = filter_reachable(self.Pset, idxset_open,
                                                                       self.Pset[idx_near], r, "B")
            if len(idxset_cand) == 0:
                return
            idx_costmin = np.argmin(self.cost[idxset_cand] + distset_cand)
            cost_new = self.cost[idxset_cand[idx_costmin]] + distset_cand[idx_costmin]
            # cost_new, idx_costmin = findmin(this.cost[idxset_cand] + distset_cand)
            # optimal time for new connection
            time_new = timeset_cand[idx_costmin]
            idx_parent = idxset_cand[idx_costmin]
            waypoints = gen_trajectory(self.Pset[idx_parent], self.Pset[idx_near], time_new, 10)
            if self.world.isValid(waypoints):
                self.bool_unvisit[idx_near] = False
                self.bool_open[idx_near] = True
                self.cost[idx_near] = cost_new
                self.time[idx_near] = time_new
                self.parent[idx_near] = idx_parent

        # print("nonzero cost idx: ", np.nonzero(self.cost))

        self.bool_open[idx_lowest] = False
        self.bool_closed[idx_lowest] = True

        return True
