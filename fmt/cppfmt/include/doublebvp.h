#pragma once

#include <functional>
#include <vector>
#include <tuple>
#include <string>
#include <list>


typedef std::tuple<std::vector<int>, std::vector<double>, std::vector<double>> reachFilter_bvp;

std::vector<double> solveBVP(double x0, double x1, double v0, double v1, double T);
std::vector<double> cost_optimal_bvp(const std::vector<double> &s0, const std::vector<double> &s1);
std::vector<double> reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T);
std::vector<double> backward_reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T);
std::vector<double> forward_reachable_box_bvp(const std::vector<double> &s0, double ux, double uy, double T);
reachFilter_bvp filter_reachable_bvp(const std::vector<std::vector<double>> &Sset, const std::vector<int> &idxset, std::vector<double> &s_c, double ux, double uy, double T, double r, bool ForR);
reachFilter_bvp filter_reachable_bvp(const std::vector<std::vector<double>> &Sset, const std::list<int> &idxset, std::vector<double> &s_c, double ux, double uy, double T, double r, bool ForR);
std::vector<std::vector<double>> gen_trajectory_bvp(const std::vector<double> &s0, const std::vector<double> &s1, int N_split = 10);
void show_trajectory_bvp(const std::vector<double> &s0, const std::vector<double> &s1, int N_split = 20, std::string color = "gray", std::string linewidth = "0.4");
bool isinsideBVP(const std::vector<double> &recBox, const std::vector<double> &s, bool ForR);
