#include "doublebvp.h"
#include <math.h>
#include <float.h>
#include <iostream>
#include <algorithm>
#include "matplotlibcpp.h"
#include <map>
#include <string>

std::vector<double> solveBVP(double x0, double x1, double v0, double v1, double T)
{
    // 无解的情况没有充分讨论
    double x10 = x1 - x0;
    if (T < 0)
    {
        std::cout << "s1 T < s0 T error" << std::endl;
        return std::vector<double>{DBL_MAX, 0};
    }
    double a = -2 * T;
    double b = 4 * x10;
    double c = T * (v1 * v1 + v0 * v0) - 2 * x10 * (v0 + v1);
    double delta = b * b - 4 * a * c;
    if (delta < 0)
    {
        std::cout << "b^2 - 4*a*c < 0 error" << std::endl;
        return std::vector<double>{DBL_MAX, 0};
    }
    double vtau0 = (-b + std::sqrt(delta)) / (2 * a);
    double vtau1 = (-b - std::sqrt(delta)) / (2 * a);
    const double epsilon = 1e-5;
    double tau0 = (vtau0 - v0) * T / (2 * vtau0 - v0 - v1);
    double tau1 = (vtau1 - v0) * T / (2 * vtau1 - v0 - v1);
    double u0 = (std::abs(tau0 - T) > epsilon) ? (v1 - vtau0) / (tau0 - T) : (vtau0 - v0) / tau0;
    double u1 = (std::abs(tau1 - T) > epsilon) ? (v1 - vtau1) / (tau1 - T) : (vtau1 - v0) / tau1;
    double vtau, tau, u;
    if (tau0 >= 0 and tau0 <= T and tau1 >= 0 and tau1 <= T)
    {
        if (std::abs(u0) < std::abs(u1))
        {
            vtau = vtau0;
            tau = tau0;
            u = u0;
        }
    }
    else if (tau1 >= 0 and tau1 <= T)
    {
        vtau = vtau1;
        tau = tau1;
        u = u1;
    }
    else if (tau0 >= 0 and tau0 <= T)
    {
        vtau = vtau0;
        tau = tau0;
        u = u0;
    }
    else
    {
        // std::cout << "tau > T, no solution" << std::endl;
        return std::vector<double>{DBL_MAX, 0};
    }

    return std::vector<double>{u, tau, vtau};
}

std::vector<double> cost_optimal_bvp(const std::vector<double> &s0, const std::vector<double> &s1)
{
    double T = s1[4] - s0[4];
    if (T < 0)
    {
        std::cout << "s1 T < s0 T error" << std::endl;
        return std::vector<double>{DBL_MAX, 0};
    }
    std::vector<double> u_tau_vtau_x = solveBVP(s0[0], s1[0], s0[2], s1[2], T);
    std::vector<double> u_tau_vtau_y = solveBVP(s0[1], s1[1], s0[3], s1[3], T);
    double ux = u_tau_vtau_x[0], uy = u_tau_vtau_y[0];
    double cost = ux * ux * T + uy * uy * T + T; // cost 系数可以调整
    return std::vector<double>{cost, T};
}

std::vector<double> reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T)
{
    ux = std::abs(ux);
    uy = std::abs(uy);
    double x0 = s0[0], y0 = s0[1], vx0 = s0[2], vy0 = s0[3];
    double xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax;
    xmin = vx0 * T + x0 - 0.5 * ux * T * T;
    xmax = vx0 * T + x0 + 0.5 * ux * T * T;
    vxmin = vx0 - ux * T;
    vxmax = vx0 + ux * T;
    if(vxmin > vxmax)
    {
        std::swap(vxmin, vxmax);
    }
    ymin = vy0 * T + y0 - 0.5 * uy * T * T;
    ymax = vy0 * T + y0 + 0.5 * uy * T * T;
    vymin = vy0 - uy * T;
    vymax = vy0 + uy * T;
        if(vymin > vymax)
    {
        std::swap(vymin, vymax);
    }
    std::vector<double> ret{xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, s0[4]};
    return ret;
}
std::vector<double> forward_reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T)
{
    T = std::abs(T);
    return reachable_box_bvp(s0, ux, uy, T);
}

std::vector<double> backward_reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T)
{
    T = -std::abs(T);
    return reachable_box_bvp(s0, ux, uy, T);
}

bool isinsideBVP(const std::vector<double> &recBox, const std::vector<double> &s, bool ForR)
{
    bool inside = (recBox[0] < s[0] and s[0] < recBox[1]) and (recBox[2] < s[1] and s[1] < recBox[3]) and (recBox[4] < s[2] and s[2] < recBox[5]) and (recBox[6] < s[3] and s[3] < recBox[7]);
    bool timeSeq = ForR ? s[4] > recBox[8] : s[4] < recBox[8];
    return inside and timeSeq;
}

reachFilter_bvp filter_reachable_bvp(const std::vector<std::vector<double>> &Sset, const std::vector<int> &idxset, std::vector<double> &s_c, double ux, double uy, double T, double r, bool ForR)
{
    std::vector<double> recBox = ForR ? forward_reachable_box_bvp(s_c, ux, uy, T) : backward_reachable_box_bvp(s_c, ux, uy, T);
    std::vector<int> idx_filter;
    std::vector<double> dist_filter, time_filter;
    for (const auto &idx : idxset)
    {
        if (isinsideBVP(recBox, Sset[idx], ForR))
        {
            std::vector<double> cost_t = ForR ? cost_optimal_bvp(s_c, Sset[idx]) : cost_optimal_bvp(Sset[idx], s_c);
            if (cost_t[0] < r)
            {
                idx_filter.push_back(idx);
                dist_filter.push_back(cost_t[0]);
                time_filter.push_back(cost_t[1]);
            }
        }
    }
    return std::make_tuple(idx_filter, dist_filter, time_filter);
}
reachFilter_bvp filter_reachable_bvp(const std::vector<std::vector<double>> &Sset, const std::list<int> &idxset, std::vector<double> &s_c, double ux, double uy, double T, double r, bool ForR)
{
    std::vector<double> recBox = ForR ? forward_reachable_box_bvp(s_c, ux, uy, T) : backward_reachable_box_bvp(s_c, ux, uy, T);
    std::vector<int> idx_filter;
    std::vector<double> dist_filter, time_filter;
    for (const auto &idx : idxset)
    {
        if (isinsideBVP(recBox, Sset[idx], ForR))
        {
            std::vector<double> cost_t = ForR ? cost_optimal_bvp(s_c, Sset[idx]) : cost_optimal_bvp(Sset[idx], s_c);
            if (cost_t[0] < r)
            {
                idx_filter.push_back(idx);
                dist_filter.push_back(cost_t[0]);
                time_filter.push_back(cost_t[1]);
            }
        }
    }
    return std::make_tuple(idx_filter, dist_filter, time_filter);
}

std::vector<std::vector<double>> gen_trajectory_bvp(const std::vector<double> &s0, const std::vector<double> &s1, int N_split)
{
    std::vector<std::vector<double>> waypoints;
    waypoints.push_back(s0);
    double T = s1[4] - s0[4];
    if (T < 0)
    {
        std::cout << "s1 T < s0 T error" << std::endl;
        return waypoints;
    }
    if(N_split < 1)
    {
        std::cout << "no split" << std::endl;
        return waypoints;
    }
    std::vector<double> u_tau_vtau_x = solveBVP(s0[0], s1[0], s0[2], s1[2], T);
    std::vector<double> u_tau_vtau_y = solveBVP(s0[1], s1[1], s0[3], s1[3], T);
    double ux = u_tau_vtau_x[0], uy = u_tau_vtau_y[0];
    double taux = u_tau_vtau_x[1], tauy = u_tau_vtau_y[1];
    double x0 = s0[0], y0 = s0[1], vx0 = s0[2], vy0 = s0[3];
    double x1 = s1[0], y1 = s1[1], vx1 = s1[2], vy1 = s1[3];
    for (int i = 1; i < N_split + 1; ++i)
    {
        double t = i * T / N_split;
        double x, y, vx, vy;
        if (t < taux)
        {
            double a = vx0;
            double b = x0;
            vx = ux * t + a;
            x = 0.5 * ux * t * t + a * t + b;
        }
        else
        {
            double a = vx1 + ux * T;
            double b = x1 + 0.5 * ux * T * T - a * T;
            vx = ux * t + a;
            x = -0.5 * ux * t * t + a * t + b;
        }
        if (t < tauy)
        {
            double a = vy0;
            double b = y0;
            vy = uy * t + a;
            y = 0.5 * uy * t * t + a * t + b;
        }
        else
        {
            double a = vy1 + uy * T;
            double b = y1 + 0.5 * uy * T * T - a * T;
            vy = uy * t + a;
            y = -0.5 * uy * t * t + a * t + b;
        }
        waypoints.push_back(std::vector<double>{x, y, vx, vy, s0[4] + t});
    }
    return waypoints;
}

namespace plt = matplotlibcpp;

void show_trajectory_bvp(const std::vector<double> &s0, const std::vector<double> &s1, int N_split, std::string color, std::string linewidth)
{
    std::vector<std::vector<double>> waypoints = gen_trajectory_bvp(s0, s1, N_split);
    // 绘出结果
    std::vector<double> x, y, t;
    for (const auto &state : waypoints)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
        t.push_back(state[4]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", color));
    style.insert(std::make_pair("linewidth", linewidth));

    plt::plot3(x, y, t, style);

    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("t"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
    // plt::plot(t, x, style);
    // plt::plot(t, y, style);
    // plt::show();
}


