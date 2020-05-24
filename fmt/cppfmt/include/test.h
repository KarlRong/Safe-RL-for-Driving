#pragma once

#include <iostream>
#include <string>
#include <time.h>
#include <vector>
#include <map>
#include <tuple>
#include <memory>
#include <utility>
#include <math.h>
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include "matplotlibcpp.h"
#include "world.h"
#include "doubleintegrator.h"
#include "fmt.h"

namespace plt = matplotlibcpp;

void testFcl()
{
    // 障碍物
    std::vector<std::vector<double>> obstacle_set;
    obstacle_set.push_back(std::vector<double>({0.2, 0.3, 0, 0.2, 0.2}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0, 0.2, 0.3}));
    obstacle_set.push_back(std::vector<double>({0.8, 0.3, 0, 0.2, 0.1}));
    obstacle_set.push_back(std::vector<double>({0.8, 0.6, 0, 0.15, 0.2}));
    obstacle_set.push_back(std::vector<double>({0.2, 0.7, 0, 0.1, 0.4}));
    obstacle_set.push_back(std::vector<double>({0.2, 0.7, 0, 0.1, 0.4}));
    double xmin = 0, xmax = 1, ymin = 0, ymax = 1, vxmin = -0.5, vxmax = 0.5, vymin = -0.5, vymax = 0.5;

    World world(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax);

    // 随机点生成
    int n = 5000;
    std::vector<std::vector<double>> states;
    std::default_random_engine e(1);
    std::uniform_real_distribution<double> xdis(xmin, xmax);
    std::uniform_real_distribution<double> ydis(ymin, ymax);
    std::uniform_real_distribution<double> vxdis(vxmin, vxmax);
    std::uniform_real_distribution<double> vydis(vymin, vymax);
    for (int i = 0; i < n; ++i)
    {
        states.push_back(std::vector<double>({xdis(e), ydis(e), 0, vxdis(e), vydis(e)}));
    }

    // 碰撞检测
    std::vector<std::vector<double>> validStates;
    std::vector<std::vector<double>> invalidStates;

    for (const auto &state : states)
    {
        bool isvalid = world.isValidState(state);
        if (isvalid)
        {
            validStates.push_back(state);
        }
        else
        {
            invalidStates.push_back(state);
        }
    }
    // 绘出结果
    std::vector<double> x, y, z;
    for (const auto &state : validStates)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "b"));
    style.insert(std::make_pair("marker", "*"));
    plt::scatter(x, y, 5, style);

    style["c"] = "r";
    style["marker"] = "*";
    x.clear();
    y.clear();
    for (const auto &state : invalidStates)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
    }
    plt::scatter(x, y, 5, style);

    // plt::plot(x, w,"r--");
    plt::show();
    // Save the image (file format is determined by the extension)
    plt::save("./basic.png");
}

void testDoubleintegratorCost()
{
    clock_t start, end;
    start = clock();

    // 随机点生成
    int n = 50000;
    std::vector<std::vector<double>> s_set;
    std::default_random_engine e(1);
    double xmin = -0.5, xmax = 0.5, ymin = -0.5, ymax = 0.5, vxmin = -0.5, vxmax = 2.5, vymin = -0.5, vymax = 2.5;
    std::uniform_real_distribution<double> xdis(xmin, xmax);
    std::uniform_real_distribution<double> ydis(ymin, ymax);
    std::uniform_real_distribution<double> vxdis(vxmin, vxmax);
    std::uniform_real_distribution<double> vydis(vymin, vymax);
    for (int i = 0; i < n; ++i)
    {
        s_set.push_back(std::vector<double>({xdis(e), ydis(e), 0, vxdis(e), vydis(e)}));
        // s_set.push_back(std::vector<double>({0.2, 0.3, 0.5, 0.6}));
    }
    std::vector<double> s_c{0, 0, 0.2, 0.2};
    std::vector<int> idxset;
    for (int i = 0; i < n; ++i)
    {
        idxset.push_back(i);
    }
    auto idx_cost_time_for = filter_reachable(s_set, idxset, s_c, 1.2, true);
    auto idx_cost_time_back = filter_reachable(s_set, idxset, s_c, 1.2, false);

    end = clock();
    std::cout << "cost time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;

    std::vector<std::vector<double>> s_for, s_back;
    for (const auto &idx : std::get<0>(idx_cost_time_for))
    {
        s_for.push_back(s_set[idx]);
    }
    for (const auto &idx : std::get<0>(idx_cost_time_back))
    {
        s_back.push_back(s_set[idx]);
    }

    // 绘出结果
    std::vector<double> x, y;
    for (const auto &state : s_for)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "b"));
    style.insert(std::make_pair("marker", "*"));
    plt::scatter(x, y, 5, style);

    style["c"] = "r";
    style["marker"] = "*";
    x.clear();
    y.clear();
    for (const auto &state : s_back)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
    }
    plt::scatter(x, y, 5, style);

    plt::show();
}

void testDoubleintegratorConnect()
{
    std::vector<double> s0{0, 0, 0.3, 0};
    std::vector<double> s1{0.1, 0.1, -0.1, -0.01};
    show_trajectory(s0, s1, 0.5);
    plt::show();
}

void testFMT()
{
    // 障碍物
    std::vector<std::vector<double>> obstacle_set;
    obstacle_set.push_back(std::vector<double>({0.2, 0.3, 0, 0.2, 0.2}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0, 0.2, 0.3}));
    obstacle_set.push_back(std::vector<double>({0.8, 0.3, 0, 0.2, 0.1}));
    obstacle_set.push_back(std::vector<double>({0.8, 0.6, 0, 0.15, 0.2}));
    obstacle_set.push_back(std::vector<double>({0.2, 0.7, 0, 0.1, 0.4}));
    obstacle_set.push_back(std::vector<double>({0.2, 0.7, 0, 0.1, 0.4}));
    double xmin = 0, xmax = 1, ymin = 0, ymax = 1, vxmin = -0.5, vxmax = 0.5, vymin = -0.5, vymax = 0.5;

    std::shared_ptr<World> world = std::make_shared<World>(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax);

    std::vector<double> s_init{0.1, 0.5, 0, 0};
    std::vector<double> s_goal{0.9, 0.9, 0, 0};
    int Nsample = 400;

    clock_t start, end;
    start = clock();

    auto fmt = FMTree(s_init, s_goal, Nsample, world);
    fmt.solve();
    end = clock();
    std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;

    show(fmt);
    plt::show();
    return;
}