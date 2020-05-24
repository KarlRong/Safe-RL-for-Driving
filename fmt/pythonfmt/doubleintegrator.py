import numpy as np
import random
import matplotlib as mpl
import matplotlib.pyplot as plt
import time


def bisection_newton(f, df, left, right, eps=0.05, itr_max=20):
    x_est = (left + right) * 0.5
    for itr in range(1, itr_max):
        f_df = f(x_est) / df(x_est)
        if (right - (x_est - f_df)) * ((x_est - f_df) - left) < 0.0 or abs(f_df) < (right - left) / 4.0:
            if f(x_est) > 0:
                right = x_est
            else:
                left = x_est
            f_df = x_est - (right + left) * 0.5
        x_est -= f_df
        if abs(f_df) < eps:
            break
    return x_est


def cost_optimal_in(x0, v0, x1, v1):
    # print("x0", x0, "v0", v0, "x1", x1, "v1", v1)
    x01 = x1 - x0
    v01 = v1 - v0
    # print("x01", x01, "v01", v01)
    p = -4 * (np.sum(v0 * v0) + np.sum(v1 * v1) + np.sum(v0 * v1))
    q = 24 * np.sum((v0 + v1) * x01)
    r = -36 * np.sum(x01 * x01)

    # print("p", p, "q", q, "r", r)
    def cost(t):
        return t + np.sum(v01 * (4.0 * v01 / t - 6 * (-v0 * t + x01) / t ** 2)) + np.sum(
            (-6 * v01 / np.power(t, 2) + 12 * (x01 - v0 * t) / np.power(t, 3)) * (-v0 * t + x01))

    def d_cost(t):
        return np.power(t, 4) + p * t * t + q * t + r  # df(t)/dt

    def dd_cost(t):
        return 4.0 * t * t * t + 2 * p * t + q  # ddf(t)/dt^2

    t_min = 0.0
    t_max = 10.0
    t_star = bisection_newton(d_cost, dd_cost, t_min, t_max)
    # print("t_star", t_star)
    return cost(t_star), t_star


def cost_optimal(s0, s1):
    a = s0[0:2]
    b = s0[2:4]
    c = s1[0:2]
    d = s1[2:4]
    return cost_optimal_in(a, b, c, d)


def forward_reachable_box(x0, v0, r):
    tau_x_plus = (2. / 3) * (-np.power(v0, 2) + r + v0 * np.sqrt(np.power(v0, 2) + r))
    tau_x_minus = (2. / 3) * (-np.power(v0, 2) + r - v0 * np.sqrt(np.power(v0, 2) + r))

    xmax = v0 * tau_x_plus + x0 + np.sqrt((1. / 3) * np.power(tau_x_plus, 2) * (-tau_x_plus + r))
    xmin = v0 * tau_x_minus + x0 - np.sqrt((1. / 3) * np.power(tau_x_minus, 2) * (-tau_x_minus + r))

    tau_v_plus = 0.5 * r
    vmax = v0 + np.sqrt(np.dot(tau_v_plus, (-tau_v_plus + r)))
    vmin = v0 - np.sqrt(np.dot(tau_v_plus, (-tau_v_plus + r)))
    return xmin, xmax, vmin, vmax


def backward_reachable_box(x0, v0, r):
    tau_x_plus = (2. / 3) * (np.power(v0, 2) - r + v0 * np.sqrt(np.power(v0, 2) + r))
    tau_x_minus = (2. / 3) * (np.power(v0, 2) - r - v0 * np.sqrt(np.power(v0, 2) + r))
    xmax = v0 * tau_x_plus + x0 + np.sqrt((1. / 3) * np.power(tau_x_plus, 2) * (tau_x_plus + r))
    xmin = v0 * tau_x_minus + x0 - np.sqrt((1. / 3) * np.power(tau_x_minus, 2) * (tau_x_minus + r))

    tau_v_plus = 0.5 * r
    vmax = v0 + np.sqrt(np.dot(tau_v_plus, (tau_v_plus + r)))
    vmin = v0 - np.sqrt(np.dot(tau_v_plus, (tau_v_plus + r)))
    return xmin, xmax, vmin, vmax


def filter_reachable(Sset, idxset, s_c, r, ForR):
    # s_set_filtered = []
    x_c = s_c[0:2]
    v_c = s_c[2:4]
    # print('x_c', x_c, 'v_c', v_c)
    xmin, xmax, vmin, vmax = forward_reachable_box(x_c, v_c, r) if ForR == "F" else backward_reachable_box(x_c, v_c, r)

    # print('xmin', xmin, 'xmax', xmax, 'vmin', vmin, 'vmax', vmax)

    def isinside(s):
        if xmin[0] < s[0] < xmax[0] and xmin[1] < s[1] < xmax[1] and vmin[0] < s[2] < vmax[0] and vmin[1] < s[3] < vmax[
            1]:
            return True
        else:
            return False

    idx_filter = []
    dist_filter = []
    time_filter = []
    for idx in idxset:
        s = Sset[idx]
        if isinside(s):
            cost, tau = cost_optimal(s, s_c) if ForR == "B" else cost_optimal(s_c, s)
            # print("cost", cost, "tau", tau)
            if cost < r:
                idx_filter.append(idx)
                dist_filter.append(cost)
                time_filter.append(tau)
    return idx_filter, dist_filter, time_filter


def gen_trajectory(s0, s1, tau, N_split=10):
    if tau == 0:
        return np.array([])
    x0 = s0[0:2]
    v0 = s0[2:4]
    x1 = s1[0:2]
    v1 = s1[2:4]
    # print("x0", x0, "v0", v0, "x1", x1, "v1", v1)
    x01 = x1 - x0
    v01 = v1 - v0
    # print("x01", x01, "v01", v01)
    d = np.concatenate(
        (-6 * v01 / tau ** 2 + 12 * (-tau * v0 + x01) / tau ** 3, 4 * v01 / tau - 6 * (-tau * v0 + x01) / tau ** 2))

    # print("d", d)
    def f(t):
        s = t - tau
        eye = np.identity(2)
        M_left = np.concatenate((np.concatenate((eye, eye * s), axis=1), np.concatenate((eye * 0, eye), axis=1)),
                                axis=0)
        # print("M_left",M_left)
        M_right = np.concatenate((np.concatenate((eye * (-s ** 3) * (1.0 / 6.0), eye * s ** 2 * 0.5), axis=1),
                                  np.concatenate((eye * (-s ** 2 * 0.5), eye * s), axis=1)), axis=0)
        # print("M_right", M_right)
        # print(d.shape)
        # print("res", [np.dot(M_left, s1)])
        return np.dot(M_left, s1.T) + np.dot(M_right, d.T)

    waypoints = []
    for n in range(1, N_split + 2):
        t = (n - 1) * tau / N_split
        waypoints.append(f(t))

    return np.array(waypoints)


def show_trajectory(s0, s1, tau, N_split=20, ax=None, c='gray', linewidth_=0.2):
    if tau == 0:
        return

    waypoints = gen_trajectory(s0, s1, tau, N_split)
    M = np.zeros((N_split + 1, 4))
    for i in range(0, N_split + 1):
        # print(waypoints[i])
        M[i, :] = waypoints[i]
    ax.plot(M[:, 0], M[:, 1], c=c, linewidth=linewidth_)
    # plt.show()


def test():
    N = 10 ** 5
    s_set = np.empty([N, 4])
    for i in range(0, N):
        s_set[i, :] = [random.random() - 0.5, random.random() - 0.5, 3 * random.random() - 0.5,
                       3 * random.random() - 0.5]

    s_c = np.array([0, 0, 0.2, 0.2])
    idxset = [x for x in range(0, N)]

    idx_for, trash, _ = filter_reachable(s_set, idxset, s_c, 1.2, "F")
    idx_back, trash, _ = filter_reachable(s_set, idxset, s_c, 1.2, "B")
    s_for = []
    for idx in idx_for:
        s_for.append(s_set[idx])
    s_back = []
    for idx in idx_back:
        s_back.append(s_set[idx])

    setA = np.zeros((2, len(idx_for)))
    setB = np.zeros((2, len(idx_back)))
    for i in range(0, len(idx_for)):
        setA[:, i] = s_for[i][0:2]

    for i in range(0, len(idx_back)):
        setB[:, i] = s_back[i][0:2]

    fig, ax = plt.subplots()  # Create a figure and an axes.
    ax.scatter(setA[0], setA[1])
    ax.scatter(setB[0], setB[1], marker='x')
    plt.show()
    return setA


def test_connection():
    """
    s0 = Vec4f(0.17, 0.08, 0.19, -0.06)
    s1 = Vec4f(0.22, 0.03, 0.139, -0.2719)
    """
    s0 = np.array([0.0, 0.0, 0.3, 0.0])
    s1 = np.array([0.1, 0.1, -0.1, -0.01])
    fig, ax = plt.subplots()  # Create a figure and an axes.
    show_trajectory(s0, s1, 0.5, ax=ax)
    plt.show()


if __name__ == "__main__":
    start = time.time()
    test()
    test_connection()
    end = time.time()
    print(end - start)
