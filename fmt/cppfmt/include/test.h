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
#include <unordered_map>
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
#include "matplotlibcpp.h"
#include "world.h"
#include "worldbvp.h"
#include "worldltl.h"
#include "doubleintegrator.h"
#include "fmtltl.h"
#include "fmtbvp.h"
#include "fmt.h"
#include "doublebvp.h"
#include "wfaltl.h"

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
        // s_set.push_back(std::vector<double>({xdis(e), ydis(e), vxdis(e), vydis(e), 0}));
        s_set.push_back(std::vector<double>({0.2, 0.3, 0.5, 0.6}));
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
    int Nsample = 2000;

    clock_t start, end;
    start = clock();

    auto fmt = FMTree(s_init, s_goal, Nsample, world, false);
    fmt.solve();
    end = clock();
    std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;

    show(fmt);
    plt::show();
    return;
}

void testDoubleBvpConnect()
{
    std::vector<double> s0{0, 0, 0, 0, 0};
    std::vector<double> s1{1, -1, 4, -1, 1};
    show_trajectory_bvp(s0, s1);
    // plt::show();
    // plt::show();
}

void testDoubleBvpCost()
{
    clock_t start, end;
    start = clock();

    // 随机点生成
    int n = 5000000;
    std::vector<std::vector<double>> s_set;
    std::default_random_engine e(1);
    double xmin = -3, xmax = 3, ymin = -3, ymax = 3, vxmin = -3, vxmax = 2.5, vymin = -3, vymax = 2.5, tmin = 0, tmax = 50;
    std::uniform_real_distribution<double> xdis(xmin, xmax);
    std::uniform_real_distribution<double> ydis(ymin, ymax);
    std::uniform_real_distribution<double> vxdis(vxmin, vxmax);
    std::uniform_real_distribution<double> vydis(vymin, vymax);
    std::uniform_real_distribution<double> tdis(tmin, tmax);
    for (int i = 0; i < n; ++i)
    {
        s_set.push_back(std::vector<double>({xdis(e), ydis(e), vxdis(e), vydis(e), tdis(e)}));
        // s_set.push_back(std::vector<double>({0.2, 0.3, 0.5, 0.6}));
    }
    std::vector<double> s_c{0, 0, 0.5, -0.5, 20};
    std::vector<int> idxset;
    for (int i = 0; i < n; ++i)
    {
        idxset.push_back(i);
    }
    auto idx_cost_time_for = filter_reachable_bvp(s_set, idxset, s_c, 1, 1, 1, 4, true);
    auto idx_cost_time_back = filter_reachable_bvp(s_set, idxset, s_c, 1, 1, 1, 4, false);

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
    std::vector<double> x, y, vx, vy, t;
    for (const auto &state : s_for)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
        vx.push_back(state[2]);
        vy.push_back(state[3]);
        t.push_back(state[4]);
    }
    std::map<std::string, std::string> style;
    style.insert(std::make_pair("c", "b"));
    style.insert(std::make_pair("marker", "*"));
    plt::scatter(x, t, 5, style);

    style["c"] = "r";
    style["marker"] = "*";
    x.clear();
    y.clear();
    vx.clear();
    vy.clear();
    t.clear();
    for (const auto &state : s_back)
    {
        x.push_back(state[0]);
        y.push_back(state[1]);
        vx.push_back(state[2]);
        vy.push_back(state[3]);
        t.push_back(state[4]);
    }
    plt::scatter(x, t, 5, style);

    plt::show();
}

void testFMTBvp()
{
    // 障碍物
    std::vector<std::vector<double>> obstacle_set;
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 0}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 1}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 2}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 3}));
    obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 4}));
    // obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 5}));
    // obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 6}));
    // obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 7}));
    // obstacle_set.push_back(std::vector<double>({0.5, 0.5, 0.6, 0.6, 8}));

    double xmin = 0, xmax = 1, ymin = 0, ymax = 1, vxmin = -0.5, vxmax = 0.5, vymin = -0.5, vymax = 0.5, tmin = 0, tmax = 10;

    std::shared_ptr<WorldBvp> world = std::make_shared<WorldBvp>(obstacle_set, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax);

    std::vector<double> s_init{0.1, 0.1, 0, 0, 1};
    std::vector<double> s_goal{0.9, 0.9, 0, 0, 9};
    int Nsample = 2000;

    clock_t start, end;
    start = clock();

    FMTreeBvp fmt = FMTreeBvp(s_init, s_goal, Nsample, world, false);
    fmt.solve();
    end = clock();
    std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;

    show(fmt);
    return;
}

void testProposotionWorld()
{
    std::vector<std::vector<double>> roads;
    roads.push_back(std::vector<double>({0, 100, 0, 3.5}));
    roads.push_back(std::vector<double>({0, 100, 3.5, 7}));
    roads.push_back(std::vector<double>({0, 100, 7, 10.5}));
    roads.push_back(std::vector<double>({0, 100, 10.5, 14}));
    std::vector<double> cross({70, 75, 0, 14});
    RoadLTL roadLtl = RoadLTL(roads, cross);
    std::unordered_map<unsigned int, ObjLTL> vehs;
    std::vector<std::vector<double>> states;
    double w = 4, h = 2.5;
    ObjLTL obj;
    states.push_back({15, 1.7, 0});
    states.push_back({16.4, 1.7, 0.2});
    states.push_back({17.8, 1.7, 0.4});
    states.push_back({19.2, 1.7, 0.6});
    states.push_back({20.6, 1.7, 0.8});
    states.push_back({22, 1.7, 1});
    states.push_back({22.4, 1.7, 1.2});
    states.push_back({23.8, 1.7, 1.4});
    states.push_back({25.2, 1.7, 1.6});
    states.push_back({26.6, 1.7, 1.8});
    states.push_back({28, 1.7, 2});
    states.push_back({29.4, 1.7, 2});
    states.push_back({30.8, 1.7, 2.2});
    states.push_back({32, 1.7, 2.4});
    obj = ObjLTL(states, w, h);
    vehs[0] = obj;
    for (auto &state : states)
    {
        state[0] = state[0] - 10;
        state[1] = state[1] + 3.5;
    }
    obj = ObjLTL(states, w, h);
    vehs[1] = obj;
    std::vector<ObjLTL> humans;

    double xmin = 5, xmax = 30, ymin = 0, ymax = 7, vxmin = -2.5, vxmax = 7, vymin = -2, vymax = 2, tmin = 0, tmax = 2.5;
    double timestep = 0.2;
    bool traffic_light = false;

    std::shared_ptr<WorldLTL> world = std::make_shared<WorldLTL>(vehs, humans, roadLtl, traffic_light, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w, h);
    std::vector<double> s_init({8, 1.7, 7, 0, 0});
    std::vector<double> s_goal({27.5, 4.7, 7, 0, 2.5});
    std::vector<unsigned int> nearby = world->setNearbyVehIds(s_init); //必要
    std::cout << "front car id:" << nearby[0] << std::endl
              << "right car id:" << nearby[1] << std::endl
              << "left car id:" << nearby[2] << std::endl;

    PropositionsLTL pro;
    // pro = world->getProposition(s_goal);
    // std::cout << std:: endl << pro << std::endl << std::endl; // 正确

    // s_goal = std::vector<double> ({15, 4.7, 7, 0, 2.5});
    // pro = world->getProposition(s_goal);
    // std::cout << std:: endl << pro << std::endl << std::endl; // 正确

    s_goal = std::vector<double>({20, 5.8, 0, 0, 2.01});
    s_goal = std::vector<double>({11, 6.8, 0, 0, 1.67});
    std::cout << world->isValidState(s_goal) << std::endl;
    pro = world->getProposition(s_goal);
    std::cout << std::endl
              << pro << std::endl
              << std::endl;

    //密集测试
    for (unsigned int i = 0; i < 2000; ++i)
    {
        std::vector<double> state = world->sampleValid();
        pro = world->getProposition(state);
        //             std::cout << "state: " << std::endl << state[0] << " " << state[1] << " " << state[4] << std:: endl;
        // std::cout << std:: endl << pro << std::endl << std::endl;
    }
    //碰撞检测测试
    //     std::uniform_real_distribution<double> xdis(xmin, xmax);
    // std::uniform_real_distribution<double> ydis(ymin, ymax);
    // std::uniform_real_distribution<double> vxdis(vxmin, vxmax);
    // std::uniform_real_distribution<double> vydis(vymin, vymax);
    // std::uniform_real_distribution<double> tdis(tmin, tmax);
    // std::vector<std::vector<double>> s_set, valid_set;
    // int n = 2000;
    // std::default_random_engine e(1);

    // for (int i = 0; i < n; ++i)
    // {
    //     // s_set.push_back(std::vector<double>({xdis(e), ydis(e), vxdis(e), vydis(e), tdis(e)}));
    //     s_set.push_back(std::vector<double>({xdis(e), ydis(e), vxdis(e), vydis(e), 2.0}));
    //     // s_set.push_back(std::vector<double>({0.2, 0.3, 0.5, 0.6}));
    // }
    // for(const auto state : s_set)
    // {
    //     if(world->isValidState(state))
    //     {
    //         valid_set.push_back(state);
    //     }
    // }
    //     std::vector<double> x, y, vx, vy, t;
    // for (const auto &state : valid_set)
    // {
    //     x.push_back(state[0]);
    //     y.push_back(state[1]);
    //     vx.push_back(state[2]);
    //     vy.push_back(state[3]);
    //     t.push_back(state[4]);
    // }
    // std::map<std::string, std::string> style;
    // style.insert(std::make_pair("c", "b"));
    // style.insert(std::make_pair("marker", "*"));
    // plt::scatter(x, y, 5, style);
    // plt::show();
}

void testProposotionLTL()
{
    std::vector<std::vector<double>> roads;
    roads.push_back(std::vector<double>({0, 100, 0, 3.5}));
    roads.push_back(std::vector<double>({0, 100, 3.5, 7}));
    roads.push_back(std::vector<double>({0, 100, 7, 10.5}));
    roads.push_back(std::vector<double>({0, 100, 10.5, 14}));
    std::vector<double> cross({70, 75, 0, 14});
    RoadLTL roadLtl = RoadLTL(roads, cross);
    std::unordered_map<unsigned int, ObjLTL> vehs;
    std::vector<std::vector<double>> states;
    double w = 4, h = 2.5;
    ObjLTL obj;
    states.push_back({15, 1.7, 0});
    states.push_back({16.4, 1.7, 0.2});
    states.push_back({17.8, 1.7, 0.4});
    states.push_back({19.2, 1.7, 0.6});
    states.push_back({20.6, 1.7, 0.8});
    states.push_back({22, 1.7, 1});
    states.push_back({22.4, 1.7, 1.2});
    states.push_back({23.8, 1.7, 1.4});
    states.push_back({25.2, 1.7, 1.6});
    states.push_back({26.6, 1.7, 1.8});
    states.push_back({28, 1.7, 2});
    states.push_back({29.4, 1.7, 2});
    states.push_back({30.8, 1.7, 2.2});
    states.push_back({32, 1.7, 2.4});
    obj = ObjLTL(states, w, h);
    vehs[0] = obj;
    for (auto &state : states)
    {
        state[0] = state[0] - 12;
        state[1] = state[1] + 3.5;
    }
    obj = ObjLTL(states, w, h);
    vehs[1] = obj;
    std::vector<ObjLTL> humans;

    double xmin = 5, xmax = 30, ymin = 0, ymax = 7, vxmin = -2.5, vxmax = 7, vymin = -2, vymax = 2, tmin = 0, tmax = 2.5;
    double timestep = 0.2;
    bool traffic_light = false;

    std::shared_ptr<WorldLTL> world = std::make_shared<WorldLTL>(vehs, humans, roadLtl, traffic_light, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w, h);
    std::vector<double> s_init({8, 1.75, 7, 0, 0});
    std::vector<double> s_goal({27.5, 4.7, 7, 0, 2.5});
    std::vector<unsigned int> nearby = world->setNearbyVehIds(s_init); //必要
    std::cout << "front car id:" << nearby[0] << std::endl
              << "right car id:" << nearby[1] << std::endl
              << "left car id:" << nearby[2] << std::endl;

    int Nsample = 1200;

    clock_t start, end;
    start = clock();

    WfaLTLs wfas;
    wfas.addWfa(std::shared_ptr<WfaLTL>(new SwitchlaneLTL()));
    wfas.addWfa(std::shared_ptr<WfaLTL>(new LanekeepLTL())); // 18
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcLeftTakeLTL())); // 1.6
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcLeftGiveLTL())); // 18
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcRightGiveLTL())); // 1.6
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcRightTakeLTL()));

    double exp_velo = 6;
    unsigned int lc = 0; // 保持，左变道，右变道
    FMTreeLTL fmt = FMTreeLTL(s_init, exp_velo, lc, Nsample, world, wfas, false);
    fmt.ux_limit_ = 5;
    fmt.uy_limit_ = 5;
    fmt.T_limit_ = 5;
    fmt.r_ = 200;
    fmt.solve();
    end = clock();
    std::cout << "fmt solving time: " << (double)(end - start) / CLOCKS_PER_SEC << "S" << std::endl;
    show(fmt);

    std::vector<std::vector<double>> waypoints;
    auto result = fmt.getResult();
    auto iter = result.rbegin();
    std::cout << "cost: " << std::endl;
    for (; iter != result.rend(); ++iter)
    {
        int idx = *iter;
        std::cout << "idx: " << idx << " " << fmt.cost_[idx] << " ";
        auto traj = gen_trajectory_bvp(fmt.Pset_[fmt.parent_[idx]], fmt.Pset_[idx], 3);
        for (const auto &state : traj)
        {
            waypoints.push_back(state);
        }
    }
    std::cout << std::endl;

    std::vector<unsigned int> wfa_states = wfas.getInitialStates();
    double cost = 0;
    PropositionsLTL pro;
    for (const auto &state : waypoints)
    {
        pro = world->getProposition(state);
        auto ret = wfas.getNextStates(wfa_states, pro);
        wfa_states = ret.first;
        // std::cout << std::endl
        //           << pro << std::endl
        //           << "Cost: " << ret.second << std::endl;
        cost += ret.second;
    }
    std::cout << std::endl;
    std::cout << "Total cost: " << cost << std::endl;
}

void testFMTLTL()
{
    std::vector<std::vector<double>> roads;
    roads.push_back(std::vector<double>({0, 100, 0, 3.5}));
    roads.push_back(std::vector<double>({0, 100, 3.5, 7}));
    roads.push_back(std::vector<double>({0, 100, 7, 10.5}));
    roads.push_back(std::vector<double>({0, 100, 10.5, 14}));
    std::vector<double> cross({70, 75, 0, 14});
    RoadLTL roadLtl = RoadLTL(roads, cross);
    std::unordered_map<unsigned int, ObjLTL> vehs;
    std::vector<std::vector<double>> states;
    double w = 4, h = 2.5;
    ObjLTL obj;
    states.push_back({15, 1.7, 0});
    states.push_back({16.4, 1.7, 0.2});
    states.push_back({17.8, 1.7, 0.4});
    states.push_back({19.2, 1.7, 0.6});
    states.push_back({20.6, 1.7, 0.8});
    states.push_back({22, 1.7, 1});
    states.push_back({22.4, 1.7, 1.2});
    states.push_back({23.8, 1.7, 1.4});
    states.push_back({25.2, 1.7, 1.6});
    states.push_back({26.6, 1.7, 1.8});
    states.push_back({28, 1.7, 2});
    states.push_back({29.4, 1.7, 2});
    states.push_back({30.8, 1.7, 2.2});
    states.push_back({32, 1.7, 2.4});
    obj = ObjLTL(states, w, h);
    vehs[0] = obj;
    for (auto &state : states)
    {
        state[0] = state[0] - 12;
        state[1] = state[1] + 3.5;
    }
    obj = ObjLTL(states, w, h);
    vehs[1] = obj;
    std::vector<ObjLTL> humans;

    double xmin = 5, xmax = 30, ymin = 0, ymax = 7, vxmin = -2.5, vxmax = 7, vymin = -2, vymax = 2, tmin = 0, tmax = 2.5;
    double timestep = 0.2;
    bool traffic_light = false;

    std::shared_ptr<WorldLTL> world = std::make_shared<WorldLTL>(vehs, humans, roadLtl, traffic_light, xmin, xmax, ymin, ymax, vxmin, vxmax, vymin, vymax, tmin, tmax, timestep, w, h);
    std::vector<double> s_init({8, 1.75, 7, 0, 0});
    std::vector<double> s_goal({27.5, 4.7, 7, 0, 2.5});
    std::vector<unsigned int> nearby = world->setNearbyVehIds(s_init); //必要
    std::cout << "front car id:" << nearby[0] << std::endl
              << "right car id:" << nearby[1] << std::endl
              << "left car id:" << nearby[2] << std::endl;

    WfaLTLs wfas;
    wfas.addWfa(std::shared_ptr<WfaLTL>(new SwitchlaneLTL()));
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LanekeepLTL())); // 18
    wfas.addWfa(std::shared_ptr<WfaLTL>(new LcLeftTakeLTL())); // 1.6
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcLeftGiveLTL())); // 18
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcRightGiveLTL())); // 1.6
    // wfas.addWfa(std::shared_ptr<WfaLTL>(new LcRightTakeLTL()));

    int Nsample = 1200;
    double exp_velo = 3; // 期待速度
    unsigned int lc = 1; // 保持，左变道，右变道
    FMTreeLTL fmt = FMTreeLTL(s_init, exp_velo, lc, Nsample, world, wfas, false);
    fmt.ux_limit_ = 5;
    fmt.uy_limit_ = 5;
    fmt.T_limit_ = 5;
    fmt.r_ = 200;
    auto ret = fmt.solve();
    std::cout << "total cost: " << fmt.cost_[ret.first.front()] << std::endl;
    std::cout << "total ltl cost: " << ret.second << std::endl;
    std::cout << "total speed cost: " << fmt.cost_speed_[ret.first.front()] << std::endl;

    show(fmt);

    for (const auto &idx : ret.first)
    {
        std::cout << "cost: " << fmt.cost_[idx] << std::endl;
        std::cout << "ltl cost: " << fmt.cost_ltl_[idx] << std::endl;
        std::cout << "speed cost: " << fmt.cost_speed_[idx] << std::endl;
    }
    // std::vector<std::vector<double>> waypoints;
    // auto result = fmt.getResult();
    // auto iter = result.rbegin();
    // double ltlcost = 0;
    // for (; iter != result.rend(); ++iter)
    // {
    //     int idx = *iter;
    //     waypoints.push_back(fmt.Pset_[idx]);
    // }
    // std::cout << std::endl;

    // std::vector<unsigned int> wfa_states = wfas.getInitialStates();
    // double cost = 0;
    // PropositionsLTL pro;
    // for (const auto &state : waypoints)
    // {
    //     pro = world->getProposition(state);
    //     auto ret = wfas.getNextStates(wfa_states, pro);
    //     wfa_states = ret.first;
    //     // std::cout << std::endl
    //     //           << pro << std::endl
    //     //           << "Cost: " << ret.second << std::endl;
    //     cost += ret.second;
    // }
    // std::cout << std::endl;
    // std::cout << "Total cost: " << cost << std::endl;
}
