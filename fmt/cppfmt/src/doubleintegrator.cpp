#include "doubleintegrator.h"
#include <math.h>
#include "matplotlibcpp.h"

using namespace Eigen;

double bisection_newton(pf_cost f, pf_cost df, double left, double right, double eps, double itr_max)
{
    double x_est = (left + right) / 2;
    for (unsigned int i = 0; i < itr_max; ++i)
    {
        double f_x_est = f(x_est);
        double f_df = f_x_est / df(x_est);
        if ((right - (x_est - f_df)) * ((x_est - f_df) - left) < 0.0 || std::fabs(f_df) < (right - left) / 4)
        {
            if (f_x_est > 0)
            {
                right = x_est;
            }
            else
            {
                left = x_est;
            }
            f_df = x_est - (right + left) / 2;
        }
        x_est -= f_df;
        if (std::fabs(f_df) < eps)
        {
            break;
        }
    }
    return x_est;
}

std::vector<double> cost_optimal(Vector2d &x0, Vector2d &v0, Vector2d &x1, Vector2d &v1)
{
    Vector2d x01 = x1 - x0;
    Vector2d v01 = v1 - v0;

    double p = -4 * (v0.dot(v0) + v1.dot(v1) + v0.dot(v1));
    double q = 24 * x01.dot(v0 + v1);
    double r = -36 * x01.dot(x01);

    double t_min = 0, t_max = 10;
    auto cost = [=](double t) -> double {
        return t + v01.dot(4 * v01 / t - 6 * (-t * v0 + x01) / (std::pow(t, 2))) + (-t * v0 + x01).dot(-6 * v01 / std::pow(t, 2) + 12 * (x01 - v0 * t) / std::pow(t, 3));
    };
    auto d_cost = [=](double t) -> double { return std::pow(t, 4) + p * t * t + q * t + r; };
    auto dd_cost = [=](double t) -> double { return 4.0 * std::pow(t, 3) + 2 * p * t + q; };

    double t_star = bisection_newton(d_cost, dd_cost, t_min, t_max);
    double cost_t = cost(t_star);
    return std::vector<double>{cost_t, t_star};
}

std::vector<double> cost_optimal(const std::vector<double> &s0, const std::vector<double> &s1)
{
    Vector2d a, b, c, d;
    a << s0[0], s0[1];
    b << s0[2], s0[3];
    c << s1[0], s1[1];
    d << s1[2], s1[3];
    return cost_optimal(a, b, c, d);
}

std::vector<double> forward_reachable_box(Vector2d &x0, Vector2d &v0, double r)
{
    Vector2d tau_x_plus = (2. / 3) * (-v0.array().pow(2) + r + v0.array() * (v0.array().pow(2) + r).sqrt()).matrix();
    Vector2d tau_x_minus = (2. / 3) * (-v0.array().pow(2) + r - v0.array() * (v0.array().pow(2) + r).sqrt()).matrix();

    Vector2d xmax = (v0.array() * tau_x_plus.array() + x0.array() + ((1. / 3) * tau_x_plus.array().pow(2) * (-tau_x_plus.array() + r)).sqrt()).matrix();
    Vector2d xmin = (v0.array() * tau_x_minus.array() + x0.array() - ((1. / 3) * tau_x_minus.array().pow(2) * (-tau_x_minus.array() + r)).sqrt()).matrix();

    double tau_v_plus = 0.5 * r;
    Vector2d vmax = (v0.array() + std::sqrt(tau_v_plus * (-tau_v_plus + r))).matrix();
    Vector2d vmin = (v0.array() - std::sqrt(tau_v_plus * (-tau_v_plus + r))).matrix();
    std::vector<double> ret{xmin(0), xmax(0), xmin(1), xmax(1), vmin(0), vmax(0), vmin(1), vmax(1)};
    return ret;
}

std::vector<double> backward_reachable_box(Vector2d &x0, Vector2d &v0, double r)
{
    Vector2d tau_x_plus = (2. / 3) * (-v0.array().pow(2) - r + v0.array() * (v0.array().pow(2) + r).sqrt()).matrix();
    Vector2d tau_x_minus = (2. / 3) * (-v0.array().pow(2) - r - v0.array() * (v0.array().pow(2) + r).sqrt()).matrix();

    Vector2d xmax = (v0.array() * tau_x_plus.array() + x0.array() + ((1. / 3) * tau_x_plus.array().pow(2) * (-tau_x_plus.array() + r)).sqrt()).matrix();
    Vector2d xmin = (v0.array() * tau_x_minus.array() + x0.array() - ((1. / 3) * tau_x_minus.array().pow(2) * (-tau_x_minus.array() + r)).sqrt()).matrix();

    double tau_v_plus = 0.5 * r;
    Vector2d vmax = (v0.array() + std::sqrt(tau_v_plus * (tau_v_plus + r))).matrix();
    Vector2d vmin = (v0.array() - std::sqrt(tau_v_plus * (tau_v_plus + r))).matrix();
    std::vector<double> ret{xmin(0), xmax(0), xmin(1), xmax(1), vmin(0), vmax(0), vmin(1), vmax(1)};
    return ret;
}

bool isinside(const std::vector<double> &recBox, const std::vector<double> &s)
{
    return (recBox[0] < s[0] and s[0] < recBox[1]) and (recBox[2] < s[1] and s[1] < recBox[3]) and (recBox[4] < s[2] and s[2] < recBox[5]) and (recBox[6] < s[3] and s[3] < recBox[7]);
}

reachFilter filter_reachable(const std::vector<std::vector<double>> &Sset, const std::vector<int> &idxset, std::vector<double> &s_c, double r, bool ForR)
{
    Vector2d x_c, v_c;
    x_c << s_c[0], s_c[1];
    v_c << s_c[2], s_c[3];

    std::vector<double> recBox = ForR ? forward_reachable_box(x_c, v_c, r) : backward_reachable_box(x_c, v_c, r);
    std::vector<int> idx_filter;
    std::vector<double> dist_filter, time_filter;
    for (const auto &idx : idxset)
    {
        if (isinside(recBox, Sset[idx]))
        {
            std::vector<double> cost_tau = ForR ? cost_optimal(s_c, Sset[idx]) : cost_optimal(Sset[idx], s_c);
            if (cost_tau[0] < r)
            {
                idx_filter.push_back(idx);
                dist_filter.push_back(cost_tau[0]);
                time_filter.push_back(cost_tau[1]);
            }
        }
    }
    return std::make_tuple(idx_filter, dist_filter, time_filter);
}

std::vector<std::vector<double>> gen_trajectory(const std::vector<double> &s0, const std::vector<double> &s1, double tau, int N_split)
{
    std::vector<std::vector<double>> waypoints;
    if (tau < 0.001)
    {
        return waypoints;
    }
    Vector4d s0M, s1M;
    s0M << s0[0], s0[1], s0[2], s0[3];
    s1M << s1[0], s1[1], s1[2], s1[3];
    Vector2d x0, v0, x1, v1, x01, v01;
    x0 << s0[0], s0[1];
    v0 << s0[2], s0[3];
    x1 << s1[0], s1[1];
    v1 << s1[2], s1[3];
    x01 = x1 - x0;
    v01 = v1 - v0;

    Vector4d d;
    Vector2d du, dd; // d up, d down
    du = -6 * v01 / (tau * tau) + 12 * (-tau * v0 + x01) / (std::pow(tau, 3));
    dd = 4 * v01 / tau - 6 * (-tau * v0 + x01) / (tau * tau);
    d << du(0), du(1), dd(0), dd(1);

    for (int i = 1; i < N_split + 2; ++i)
    {
        double t = (i - 1) * tau / N_split;
        double s = t - tau;
        Matrix2d eye = Matrix2d::Identity();

        Matrix4d M_left, M_right;
        M_left << eye, eye * s,
            eye * 0, eye;
        M_right << eye * (-std::pow(s, 3) * (1. / 6)), eye * (std::pow(s, 2)) * 0.5,
            eye * (-0.5 * std::pow(s, 2)), eye * s;
        Vector4d waypointM;
        waypointM = M_left * s1M + M_right * d;
        waypoints.push_back(std::vector<double>{waypointM[0], waypointM[1], waypointM[2], waypointM[3]});
    }
    return waypoints;
}

namespace plt = matplotlibcpp;

void show_trajectory(const std::vector<double> &s0, const std::vector<double> &s1, double tau, int N_split, std::string color, std::string linewidth)
{
    if (tau < 0.001)
    {
        return;
    }
    std::vector<std::vector<double>> waypoints = gen_trajectory(s0, s1, tau, N_split);
    // 绘出结果
    std::vector<double> x, y;
    for (const auto &state : waypoints)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", color));
    style.insert(std::make_pair("linewidth", linewidth));
    plt::plot(x, y, style);
}
