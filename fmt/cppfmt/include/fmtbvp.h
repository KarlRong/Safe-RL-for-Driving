#pragma once

#include <vector>
#include <map>
#include <memory>
#include <random>
#include <list>
#include "worldbvp.h"

class FMTreeBvp
{
    friend void show(const FMTreeBvp &fmt);

public:
    FMTreeBvp(const std::vector<double> &s_init, const std::vector<double> &s_goal, int N, std::shared_ptr<WorldBvp> world, bool draw = false) : world_(world), s_init_(s_init),
                                                                                                                                                 s_goal_(s_goal),
                                                                                                                                                 N_(N),

                                                                                                                                                 draw_(draw)
    {
        Pset_.push_back(s_init_);
        for (int i = 1; i < N_ - 1; ++i)
        {
            Pset_.push_back(world_->sampleValid());
        }
        Pset_.push_back(s_goal_);
        cost_ = std::vector<double>(N_, 0);
        time_ = std::vector<double>(N_, 0);
        parent_ = std::vector<int>(N_, 0);
        for (int i = 1; i < N; ++i)
        {
            unvisit_.push_back(i);
        }
        open_.push_back(0);
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
    std::shared_ptr<WorldBvp> world_;
    double ux_limit_ = 1;
    double uy_limit_ = 1;
    double T_limit_ = 1;
    double r_ = 10;

    std::vector<double> s_init_;
    std::vector<double> s_goal_;
    int N_ = 0;
    bool draw_ = false;
    std::vector<std::vector<double>> Pset_;
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

void show(const FMTreeBvp &fmt);
