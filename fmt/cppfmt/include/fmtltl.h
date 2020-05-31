#pragma once

#include <vector>
#include <map>
#include <memory>
#include <random>
#include <list>
#include "worldltl.h"
#include "wfaltl.h"

class FMTreeLTL
{
    friend void show(const FMTreeLTL &fmt);

public:
    FMTreeLTL(const std::vector<double> &s_init, const std::vector<double> &s_goal, int N, std::shared_ptr<WorldLTL> world, WfaLTLs wfas, bool draw = false) : world_(world), s_init_(s_init),
                                                                                                                                                               s_goal_(s_goal),
                                                                                                                                                               N_(N),
                                                                                                                                                               wfas_(wfas),
                                                                                                                                                               draw_(draw)
    {
        Pset_.push_back(s_init_);
        // 采样 N-2个点
        for (int i = 1; i < N_ - 1; ++i)
        {
            Pset_.push_back(world_->sampleValid());
        }
        Pset_.push_back(s_goal_);
        // 初始化cost, time, parent
        cost_ = std::vector<double>(N_, 0);
        cost_ltl_ = std::vector<double>(N_, 0);
        time_ = std::vector<double>(N_, 0);
        parent_ = std::vector<int>(N_, 0);
        // 起点加入open_,其他加入unvisit_
        for (int i = 1; i < N; ++i)
        {
            unvisit_.push_back(i);
        }
        open_.push_back(0);
        // 初始化所有wfa的states
        for (int i = 0; i < N; ++i)
        {
            Wfa_states_.push_back(wfas_.getInitialStates());
        }
        // 根据s_init_处理前车左车后车
        world_->setNearbyVehIds(s_init_);
        // 处理起点的wfa state
        auto state_cost = wfas_.getNextStates(wfas_.getInitialStates(), world->getProposition(s_init_));
        Wfa_states_[0] = state_cost.first;
        cost_ltl_[0] = state_cost.second;
    };

    std::vector<int> solve();
    std::vector<int> getResult()
    {
        return result_;
    };
    bool goalReached()
    {
        return goalReached_;
    };
    std::shared_ptr<WorldLTL> world_;
    double ux_limit_ = 1;
    double uy_limit_ = 1;
    double T_limit_ = 1;
    double r_ = 10;

    std::vector<double> s_init_;
    std::vector<double> s_goal_;
    int N_ = 0;
    std::vector<std::vector<double>> Pset_;
    WfaLTLs wfas_;
    std::vector<std::vector<unsigned int>> Wfa_states_;
    bool draw_ = false;
    std::vector<double> cost_ltl_;
    std::vector<double> cost_;
    std::vector<double> time_;
    std::vector<int> parent_;
    std::list<int> unvisit_;
    std::vector<int> open_;
    std::list<int> closed_;

private:
    bool extend();
    int itr_ = 0;

    std::vector<int> result_;
    bool goalReached_ = false;
};

void show(const FMTreeLTL &fmt);
