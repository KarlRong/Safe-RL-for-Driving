#include "fmtltl.h"
#include <algorithm>
#include <float.h>
#include <tuple>
#include <iostream>
#include <string>
#include "matplotlibcpp.h"
#include "doublebvp.h"
#include <time.h>
#include "worldltl.h"
#include "doublebvp.h"

std::pair<std::vector<int>, double> FMTreeLTL::solve()
{
    clock_t start, end;
    start = clock();
    auto comparator = [&](int lhs, int rhs) { return cost_[lhs] > cost_[rhs]; };
    std::make_heap(open_.begin(), open_.end(), comparator);
    while (true)
    {
        ++itr_;
        if (!extend())
        {
            break;
        }
        if (goalReached_)
        {
            break;
        }
        if (itr_ == N_/2)
        {
            break;
        }
        if (itr_ % 50 == 0)
        {
            // std::cout << itr_ << std::endl;
        }
    }

    end = clock();
    // std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;
    int idx = N_ - 1;
    if (!goalReached_)
    {
        // std::cout << "fmt failed to reach goal" << std::endl;
        idx = decideFromOpen();
    }
    double costltl = cost_ltl_[idx];
    std::vector<int> idx_solution({idx});
    while (true)
    {
        idx = parent_[idx];
        idx_solution.push_back(idx);
        if (idx == 0)
        {
            break;
        }
    }
    result_ = idx_solution;
    return std::pair<std::vector<int>, double>(idx_solution, costltl);
}

bool FMTreeLTL::extend()
{
    auto comparator = [&](int lhs, int rhs) { return cost_[lhs] > cost_[rhs]; };
    int idx_lowest = 0;
    if (!open_.empty())
    {
        idx_lowest = open_.front();
    }
    else
    {
        return false;
    }
    auto idx_Wfastates_costCon_costLTL_costS = filter_reachable_ltl(unvisit_, idx_lowest, ux_limit_, uy_limit_, T_limit_, r_, true); // 前后 reac box返回的 wfa_states不是同一个概念...
    for (const auto &idx_near : std::get<0>(idx_Wfastates_costCon_costLTL_costS))
    {
        auto idx_Wfastates_costCon_costLTL_costS_near = filter_reachable_ltl(open_, idx_near, ux_limit_, uy_limit_, T_limit_, r_, false);
        std::vector<int> &idxset_cand = std::get<0>(idx_Wfastates_costCon_costLTL_costS_near);
        std::vector<std::vector<unsigned int>> &wfa_states_cand = std::get<1>(idx_Wfastates_costCon_costLTL_costS_near);
        std::vector<double> &costcont_cand = std::get<2>(idx_Wfastates_costCon_costLTL_costS_near);
        std::vector<double> &costLtl_cand = std::get<3>(idx_Wfastates_costCon_costLTL_costS_near);
        std::vector<double> &costS_cand = std::get<4>(idx_Wfastates_costCon_costLTL_costS_near);
        if (idxset_cand.size() == 0)
        {
            continue;
        }

        double min_cost_near = DBL_MAX;
        int idx_costmin = 0;
        for (unsigned int i_near_cost = 0; i_near_cost < idxset_cand.size(); ++i_near_cost)
        {
            double cost_near = cost_[idxset_cand[i_near_cost]] + costcont_cand[i_near_cost] + costLtl_cand[i_near_cost] + costS_cand[i_near_cost]; // parent cost + 这一段cost + 这一段ltl + 这一段speed
            if (cost_near < min_cost_near)
            {
                min_cost_near = cost_near;
                idx_costmin = i_near_cost;
            }
        }
        // 更新 unvisit, open, cost, costltl, wfa_state, parent
        int idx_parent = idxset_cand[idx_costmin];
        std::vector<unsigned int> wfa_state = wfa_states_cand[idx_costmin];
        double costltl = costLtl_cand[idx_costmin];
        double costS = costS_cand[idx_costmin];
        auto waypoints = gen_trajectory_bvp(Pset_[idx_parent], Pset_[idx_near], 5); // waypoints的ltl cost被忽略，出于速度和代码简化
        if (world_->isValidStates(waypoints))
        {
            unvisit_.remove(idx_near);
            open_.push_back(idx_near);
            std::push_heap(begin(open_), end(open_), comparator);
            cost_[idx_near] = min_cost_near;
            cost_ltl_[idx_near] = cost_ltl_[idx_parent] + costltl;
            cost_speed_[idx_near] = cost_speed_[idx_parent] + costS;
            Wfa_states_[idx_near] = wfa_state;
            parent_[idx_near] = idx_parent;
            if (idx_near == N_ - 1)
            {
                goalReached_ = true;
            }
        }
    }
    std::pop_heap(begin(open_), end(open_), comparator);
    open_.pop_back();
    closed_.push_back(idx_lowest);
    // bool_open_[idx_lowest] = false;
    // bool_closed_[idx_lowest] = true;
    return true;
}

std::vector<double> FMTreeLTL::decideGoal()
{
    double T = 2.5; // 规划2.2s的路
    std::vector<double> goal = std::vector<double>({0, 0, exp_speed_, 0, T});
    if (lc_ == 2 or (lc_ == 0 and world_->road_.roads_[0].isInside(s_init_)))
    {
        goal[0] = s_init_[0] + T * exp_speed_;
        goal[1] = (world_->road_.roads_[0].ymin_ + world_->road_.roads_[0].ymax_) / 2;
    }
    else if (lc_ == 1 or (lc_ == 0 and world_->road_.roads_[1].isInside(s_init_)))
    {
        goal[0] = s_init_[0] + T * exp_speed_;
        goal[1] = (world_->road_.roads_[1].ymin_ + world_->road_.roads_[1].ymax_) / 2;
    }
    else
    {
        goal[0] = s_init_[0] + T * exp_speed_;
        goal[1] = (world_->road_.roads_[0].ymin_ + world_->road_.roads_[0].ymax_) / 2;
    }
    return goal;
}

int FMTreeLTL::decideFromOpen()
{
    int idx_ret = 0;
    double min_dis = DBL_MAX;
    for (const auto &idx : open_)
    {
        if(idx == 0)
        {
            continue;
        }
        auto ret = cost_optimal_ltl(Pset_[idx], Wfa_states_[idx], Pset_[N_ - 1]);
        double cost = std::get<0>(ret) + std::get<1>(ret);
        if (cost < min_dis)
        {
            min_dis = cost;
            idx_ret = idx;
        }
    }
    for (const auto &idx : closed_)
    {
        if(idx == 0)
        {
            continue;
        }
        auto ret = cost_optimal_ltl(Pset_[idx], Wfa_states_[idx], Pset_[N_ - 1]);
        double cost = std::get<0>(ret) + std::get<1>(ret);
        if (cost < min_dis)
        {
            min_dis = cost;
            idx_ret = idx;
        }
    }
    if(idx_ret == 0){
        auto ret = cost_optimal_ltl(Pset_[idx_ret], Wfa_states_[idx_ret], Pset_[N_ - 1]);
        min_dis = std::get<0>(ret) + std::get<1>(ret);
    }
    if(min_dis > 200)
    {
        min_dis = 200;
    }
    cost_ltl_[idx_ret] += min_dis; // open点距离终点的距离加到ltl的cost中
    return idx_ret;
}

cost_wfa_ltl FMTreeLTL::cost_optimal_ltl(const std::vector<double> &s0, const std::vector<unsigned int> &wfa_s0, const std::vector<double> &s1)
{
    auto costCont_time = cost_optimal_bvp(s0, s1);

    auto wfaState_costLtl = wfas_.getNextStates(wfa_s0, world_->getProposition(s1));
    return cost_wfa_ltl(costCont_time[0], ltl_factor * wfaState_costLtl.second, wfaState_costLtl.first);
}

reachFilter_ltl FMTreeLTL::filter_reachable_ltl(const std::vector<int> &idxset, int idx_c, double ux, double uy, double T, double r, bool ForR)
{
     std::vector<double> s_c = Pset_[idx_c];
    std::vector<double> recBox = ForR ? forward_reachable_box_bvp(s_c, ux, uy, T) : backward_reachable_box_bvp(s_c, ux, uy, T);
    std::vector<int> idx_filter;
    std::vector<double> cost_control, cost_ltl, cost_speed;
    std::vector<std::vector<unsigned int>> wfa_states;
    for (const auto &idx : idxset)
    {
        if (isinsideBVP(recBox, Pset_[idx], ForR))
        {
            std::vector<double> cost_t = ForR ? cost_optimal_bvp(s_c, Pset_[idx]) : cost_optimal_bvp(Pset_[idx], s_c);
            double costltl = 0;
            std::vector<unsigned int> wfastate;
            if (ForR)
            {
                auto wfaState_costLtl = wfas_.getNextStates(Wfa_states_[idx_c], world_->getProposition(Pset_[idx]));
                wfastate = wfaState_costLtl.first;
                costltl = wfaState_costLtl.second;
            }
            else
            {
                auto wfaState_costLtl = wfas_.getNextStates(Wfa_states_[idx], world_->getProposition(Pset_[idx_c]));
                wfastate = wfaState_costLtl.first;
                costltl = wfaState_costLtl.second;
            }
            costltl = ltl_factor * costltl;
            double costSpeed = speed_factor * (std::abs(Pset_[idx][2] - exp_speed_) + std::abs(Pset_[idx_c][2] - exp_speed_));
            if (cost_t[0] + ltl_factor * costltl  + costSpeed< r)
            {
                idx_filter.push_back(idx);
                cost_control.push_back(cost_t[0]);
                wfa_states.push_back(wfastate);
                cost_ltl.push_back(costltl);
                cost_speed.push_back(costSpeed);
            }
        }
    }
    return reachFilter_ltl(idx_filter, wfa_states, cost_control, cost_ltl, cost_speed);
    // idx, wfa_states, cost_control, cost_ltl
}

reachFilter_ltl FMTreeLTL::filter_reachable_ltl(const std::list<int> &idxset, int idx_c, double ux, double uy, double T, double r, bool ForR)
{
     std::vector<double> s_c = Pset_[idx_c];
    std::vector<double> recBox = ForR ? forward_reachable_box_bvp(s_c, ux, uy, T) : backward_reachable_box_bvp(s_c, ux, uy, T);
    std::vector<int> idx_filter;
    std::vector<double> cost_control, cost_ltl, cost_speed;
    std::vector<std::vector<unsigned int>> wfa_states;
    for (const auto &idx : idxset)
    {
        if (isinsideBVP(recBox, Pset_[idx], ForR))
        {
            std::vector<double> cost_t = ForR ? cost_optimal_bvp(s_c, Pset_[idx]) : cost_optimal_bvp(Pset_[idx], s_c);
            double costltl = 0;
            std::vector<unsigned int> wfastate;
            if (ForR)
            {
                auto wfaState_costLtl = wfas_.getNextStates(Wfa_states_[idx_c], world_->getProposition(Pset_[idx]));
                wfastate = wfaState_costLtl.first;
                costltl = wfaState_costLtl.second;
            }
            else
            {
                auto wfaState_costLtl = wfas_.getNextStates(Wfa_states_[idx], world_->getProposition(Pset_[idx_c]));
                wfastate = wfaState_costLtl.first;
                costltl = wfaState_costLtl.second;
            }
            costltl = ltl_factor * costltl;
            double costSpeed = speed_factor * (std::abs(Pset_[idx][3] - exp_speed_) + std::abs(Pset_[idx_c][3] - exp_speed_));
            if (cost_t[0] + ltl_factor * costltl  + costSpeed< r)
            {
                idx_filter.push_back(idx);
                cost_control.push_back(cost_t[0]);
                wfa_states.push_back(wfastate);
                cost_ltl.push_back(costltl);
                cost_speed.push_back(costSpeed);
            }
        }
    }
    return reachFilter_ltl(idx_filter, wfa_states, cost_control, cost_ltl, cost_speed);
    // idx, wfa_states, cost_control, cost_ltl
}

namespace plt = matplotlibcpp;

void show(const FMTreeLTL &fmt)
{
    // 只绘出最后结果
    std::vector<std::vector<double>> waypoints;
    auto iter = fmt.result_.rbegin();
    for (; iter != fmt.result_.rend(); ++iter)
    {
        int idx = *iter;
        auto traj = gen_trajectory_bvp(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], 20);
        for (const auto &state : traj)
        {
            waypoints.push_back(state);
        }
    }

    //     for (const auto &idx : fmt.result_)
    // {
    //     auto traj = gen_trajectory_bvp(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], 20);
    //     for(const auto& state : traj)
    //     {
    //         waypoints.push_back(state);
    //     }
    // }

    // 绘出结果
    std::vector<double> x, y, t;
    for (const auto &state : waypoints)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
        t.push_back(state[4]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "blue"));
    style.insert(std::make_pair("linewidth", "2"));

    plt::plot3(x, y, t, style);

    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("t"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
    plt::legend();
    plt::show();
    // plt::show();
    // plt::pause(0.7);
}
