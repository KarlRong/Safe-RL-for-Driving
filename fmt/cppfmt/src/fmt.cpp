#include "fmt.h"
#include <algorithm>
#include <float.h>
#include <tuple>
#include <iostream>
#include <string>
#include "matplotlibcpp.h"
#include "doubleintegrator.h"

std::vector<int> FMTree::solve()
{
    while (true)
    {
        if (!extend())
        {
            break;
        }
    }
    int idx = N_-1;
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

bool FMTree::extend()
{
    double r = 1;
    std::vector<int> idxset_open, idxset_unvisit;
    for (int i = 0; i < N_; ++i)
    {
        if (bool_open_[i])
        {
            idxset_open.push_back(i);
        }
        if (bool_unvisit_[i])
        {
            idxset_unvisit.push_back(i);
        }
    }

    if (idxset_open.size() == 0)
    {
        return false;
    }

    double min_cost = DBL_MAX;
    int idx_lowest = 0;
    for (const auto &idx : idxset_open)
    {
        if (cost_[idx] < min_cost)
        {
            min_cost = cost_[idx];
            idx_lowest = idx;
        }
    }

    auto idx_cost_time = filter_reachable(Pset_, idxset_unvisit, Pset_[idx_lowest], r, true);
    for (const auto &idx_near : std::get<0>(idx_cost_time))
    {
        auto idx_cost_time_near = filter_reachable(Pset_, idxset_open, Pset_[idx_near], r, false);
        if (std::get<0>(idx_cost_time_near).size() == 0)
        {
            continue;
        }

        double min_cost_near = DBL_MAX;
        int idx_lowest_near = 0;
        double time_near = 0;
        for (unsigned int i_near_cost = 0; i_near_cost < std::get<1>(idx_cost_time_near).size(); ++i_near_cost)
        {
            double cost_near = cost_[std::get<0>(idx_cost_time_near)[i_near_cost]] + std::get<1>(idx_cost_time_near)[i_near_cost];
            if (cost_near < min_cost_near)
            {
                min_cost_near = cost_near;
                idx_lowest_near = std::get<0>(idx_cost_time_near)[i_near_cost];
                time_near = std::get<2>(idx_cost_time_near)[i_near_cost];
            }
        }
        auto waypoints = gen_trajectory(Pset_[idx_lowest_near], Pset_[idx_near], time_near, 10);
        if (world_->isValidStates(waypoints))
        {
            bool_unvisit_[idx_near] = false;
            bool_open_[idx_near] = true;
            cost_[idx_near] = min_cost_near;
            time_[idx_near] = time_near;
            parent_[idx_near] = idx_lowest_near;
        }
    }
    bool_open_[idx_lowest] = false;
    bool_closed_[idx_lowest] = true;
    return true;
}

namespace plt = matplotlibcpp;

void show(const FMTree &fmt)
{
    // 绘出 open, close , unvisit
    std::vector<int> closeSet, openSet, unvisitSet;
    for (int i = 1; i < fmt.N_; ++i)
    {
        if (fmt.bool_closed_[i])
        {
            closeSet.push_back(i);
        }
        if (fmt.bool_open_[i])
        {
            openSet.push_back(i);
        }
        if (fmt.bool_unvisit_[i])
        {
            unvisitSet.push_back(i);
        }
    }

    std::vector<double> x, y;
    for (const auto &idx : closeSet)
    {
        x.push_back(fmt.Pset_[idx][0]);
        y.push_back(fmt.Pset_[idx][1]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "black"));
    style.insert(std::make_pair("marker", "*"));
    plt::scatter(x, y, 5, style);

    style["c"] = "r";
    style["marker"] = "*";
    x.clear();
    y.clear();
    for (const auto &idx : openSet)
    {
        x.push_back(fmt.Pset_[idx][0]);
        y.push_back(fmt.Pset_[idx][1]);
    }
    plt::scatter(x, y, 5, style);

    style["c"] = "g";
    style["marker"] = "o";
    x.clear();
    y.clear();
    for (const auto &idx : unvisitSet)
    {
        x.push_back(fmt.Pset_[idx][0]);
        y.push_back(fmt.Pset_[idx][1]);
    }
    plt::scatter(x, y, 2, style);

    style["c"] = "g";
    style["marker"] = "o";
    x.clear();
    y.clear();

    x.push_back(fmt.Pset_[0][0]);
    x.push_back(fmt.Pset_.back()[0]);
    y.push_back(fmt.Pset_[0][1]);
    y.push_back(fmt.Pset_.back()[1]);

    plt::scatter(x, y, 20, style);

    for(const auto& idx : openSet)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20);
    }
    for(const auto& idx : closeSet)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20);
    }

    for(const auto& idx: fmt.result_)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20, "blue", "2");
    }
}
