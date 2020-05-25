#pragma once

#include <functional>
#include "eigen3/Eigen/Dense"
#include <vector>
#include <tuple>
#include <string>
#include <list>

typedef std::function<double(double)> pf_cost;
typedef std::tuple<std::vector<int>, std::vector<double>, std::vector<double>> reachFilter;

double bisection_newton(pf_cost f, pf_cost df, double left, double right, double eps = 0.05, double itr_max = 20);
std::vector<double> cost_optimal(Eigen::Vector2d &x0, Eigen::Vector2d &v0, Eigen::Vector2d &x1, Eigen::Vector2d &v1);
std::vector<double> cost_optimal(const std::vector<double>& s0, const std::vector<double>& s1);
std::vector<double> forward_reachable_box(Eigen::Vector2d &x0, Eigen::Vector2d &v0, double r);
reachFilter filter_reachable(const std::vector<std::vector<double>> &Sset, const std::vector<int> &idxset, std::vector<double> &s_c, double r, bool ForR);
reachFilter filter_reachable(const std::vector<std::vector<double>> &Sset, const std::list<int> &idxset, std::vector<double> &s_c, double r, bool ForR);

std::vector<std::vector<double>> gen_trajectory(const std::vector<double>& s0, const std::vector<double>& s1, double tau, int N_split=10);
void show_trajectory(const std::vector<double> & s0, const std::vector<double> & s1, double tau, int N_split=20, std::string color="gray", std::string linewidth="0.4");






