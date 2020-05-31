#pragma once

#include <vector>
#include <map>
#include <memory>
#include <random>
#include <list>
#include "worldltl.h"
#include "wfaltl.h"

typedef std::tuple<std::vector<int>, std::vector<std::vector<unsigned int>>, std::vector<double>, std::vector<double>, std::vector<double>> reachFilter_ltl; // idx, wfa_states, cost_control, cost_ltl, cost_speed
typedef std::tuple<double, double, std::vector<unsigned int>> cost_wfa_ltl;                                                             // cost_control, cost_ltl, wfa_states

class FMTreeLTL
{
    friend void show(const FMTreeLTL &fmt);

public:
    FMTreeLTL(const std::vector<double> &s_init, double exp_speed, unsigned int lc, int N, std::shared_ptr<WorldLTL> world, WfaLTLs wfas, bool draw = false) : world_(world), s_init_(s_init),
                                                                                                                                                               exp_speed_(exp_speed),
                                                                                                                                                               lc_(lc), // 0保持，1左变道，2右变道
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
        s_goal_ = decideGoal();
        Pset_.push_back(s_goal_);
        // 初始化cost, time, parent
        cost_ = std::vector<double>(N_, 0);
        cost_ltl_ = std::vector<double>(N_, 0);
        cost_speed_ = std::vector<double>(N_, 0);
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
        cost_ltl_[0] = state_cost.second * ltl_factor;
        cost_speed_[0] =  std::abs(Pset_[0][3] - exp_speed_) * speed_factor;
    };

    std::pair<std::vector<int>, double> solve();
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
    double exp_speed_ = 0;
    unsigned int lc_ = 0;
    std::vector<double> s_goal_;
    int N_ = 0;
    std::vector<std::vector<double>> Pset_;
    WfaLTLs wfas_;
    std::vector<std::vector<unsigned int>> Wfa_states_;
    bool draw_ = false;
    std::vector<double> cost_ltl_;
    std::vector<double> cost_speed_;
    std::vector<double> cost_;
    std::vector<double> time_;
    std::vector<int> parent_;
    std::list<int> unvisit_;
    std::vector<int> open_;
    std::list<int> closed_;
    double ltl_factor=10;
    double speed_factor=1;

    cost_wfa_ltl cost_optimal_ltl(const std::vector<double> &s0, const std::vector<unsigned int> &wfa_s0, const std::vector<double> &s1);
    reachFilter_ltl filter_reachable_ltl(const std::vector<int> &idxset, int idx_c, double ux, double uy, double T, double r, bool ForR);
    reachFilter_ltl filter_reachable_ltl(const std::list<int> &idxset, int idx_c, double ux, double uy, double T, double r, bool ForR);

    std::vector<double> decideGoal();
    int decideFromOpen();

private:
    bool extend();
    int itr_ = 0;

    std::vector<int> result_;
    bool goalReached_ = false;
};

void show(const FMTreeLTL &fmt);
