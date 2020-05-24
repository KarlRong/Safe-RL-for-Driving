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
        if (draw_ and itr_ % 20 == 0)
        {
            show(*this);
        }
        std::cout << itr_ << std::endl;
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

bool FMTree::extend()
{
    double r = 1;

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
    auto idx_cost_time = filter_reachable(Pset_, unvisit_, Pset_[idx_lowest], r, true);
    for (const auto &idx_near : std::get<0>(idx_cost_time))
    {
        auto idx_cost_time_near = filter_reachable(Pset_, open_, Pset_[idx_near], r, false);
        std::vector<int> &idxset_cand = std::get<0>(idx_cost_time_near);
        std::vector<double> &distset_cand = std::get<1>(idx_cost_time_near);
        std::vector<double> &timeset_cand = std::get<2>(idx_cost_time_near);
        if (idxset_cand.size() == 0)
        {
            continue;
        }

        double min_cost_near = DBL_MAX;
        int idx_costmin = 0;
        for (unsigned int i_near_cost = 0; i_near_cost < distset_cand.size(); ++i_near_cost)
        {
            double cost_near = cost_[idxset_cand[i_near_cost]] + distset_cand[i_near_cost];
            if (cost_near < min_cost_near)
            {
                min_cost_near = cost_near;
                idx_costmin = i_near_cost;
            }
        }
        int idx_parent = idxset_cand[idx_costmin];
        double time_near = timeset_cand[idx_costmin];
        auto waypoints = gen_trajectory(Pset_[idx_parent], Pset_[idx_near], time_near, 10);
        if (world_->isValidStates(waypoints))
        {
            unvisit_.remove(idx_near);
            open_.push_back(idx_near);
            std::push_heap(begin(open_), end(open_), comparator);
            cost_[idx_near] = min_cost_near;
            time_[idx_near] = time_near;
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

void show(const FMTree &fmt)
{
    // 绘出 open, close , unvisit

    std::vector<double> x, y;
    for (const auto &idx : fmt.closed_)
    {
        x.push_back(fmt.Pset_[idx][0]);
        y.push_back(fmt.Pset_[idx][1]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "black"));
    style.insert(std::make_pair("marker", "*"));
    plt::scatter(x, y, 2, style);

    style["c"] = "r";
    style["marker"] = "*";
    x.clear();
    y.clear();
    for (const auto &idx : fmt.open_)
    {
        x.push_back(fmt.Pset_[idx][0]);
        y.push_back(fmt.Pset_[idx][1]);
    }
    plt::scatter(x, y, 2, style);

    style["c"] = "g";
    style["marker"] = "o";
    x.clear();
    y.clear();
    for (const auto &idx : fmt.unvisit_)
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

    for (const auto &idx : fmt.open_)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20);
    }
    for (const auto &idx : fmt.closed_)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20);
    }

    for (const auto &idx : fmt.result_)
    {
        show_trajectory(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], fmt.time_[idx], 20, "blue", "2");
    }
    // plt::show();
    plt::pause(0.7);
}
