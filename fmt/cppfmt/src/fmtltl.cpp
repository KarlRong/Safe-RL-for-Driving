#include "fmtltl.h"
#include <algorithm>
#include <float.h>
#include <tuple>
#include <iostream>
#include <string>
#include "matplotlibcpp.h"
#include "doublebvp.h"
#include <time.h>

std::vector<int> FMTreeLTL::solve()
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
        // if (goalReached_)
        // {
        //     break;
        // }
        if (itr_ == 500)
        {
            break;
        }
        // if (draw_ and itr_ % 20 == 0)
        // {
        //     show(*this);
        // }
        if (itr_ % 50 == 0)
        {
            std::cout << itr_ << std::endl;
        }
    }
    end = clock();
    std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;
    if (!goalReached_)
    {
        std::cout << "fmt failed" << std::endl;
    }
    int idx = N_ - 1;
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
    return idx_solution;
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
    auto idx_Wfastates_costCon_costLTL = filter_reachable_ltl(Pset_, Wfa_states_, unvisit_, Pset_[idx_lowest], ux_limit_, uy_limit_, T_limit_, r_, true); // 前后 reac box返回的 wfa_states不是同一个概念...
    for (const auto &idx_near : std::get<0>(idx_Wfastates_costCon_costLTL))
    {
        auto idx_Wfastates_costCon_costLTL_near = filter_reachable_ltl(Pset_, Wfa_states_, open_, Pset_[idx_near], ux_limit_, uy_limit_, T_limit_, r_, false);
        std::vector<int> &idxset_cand = std::get<0>(idx_Wfastates_costCon_costLTL_near);
        std::vector<std::vector<unsigned int>> &wfa_states_cand = std::get<1>(idx_Wfastates_costCon_costLTL_near);
        std::vector<double> &costcont_cand = std::get<2>(idx_Wfastates_costCon_costLTL_near);
        std::vector<double> &costLtl_cand = std::get<3>(idx_Wfastates_costCon_costLTL_near);
        if (idxset_cand.size() == 0)
        {
            continue;
        }

        double min_cost_near = DBL_MAX;
        int idx_costmin = 0;
        for (unsigned int i_near_cost = 0; i_near_cost < idxset_cand.size(); ++i_near_cost)
        {
            double cost_near = cost_[idxset_cand[i_near_cost]] + costcont_cand[i_near_cost] + costLtl_cand[i_near_cost];
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
        auto waypoints = gen_trajectory_bvp(Pset_[idx_parent], Pset_[idx_near], 5); // waypoints的ltl cost被忽略，出于速度和代码简化
        if (world_->isValidStates(waypoints))
        {
            unvisit_.remove(idx_near);
            open_.push_back(idx_near);
            std::push_heap(begin(open_), end(open_), comparator);
            cost_[idx_near] = min_cost_near;
            cost_ltl_[idx_near] = costltl;
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
