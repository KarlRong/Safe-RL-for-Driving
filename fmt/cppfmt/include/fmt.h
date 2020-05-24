#pragma once

#include <vector>
#include <map>
#include <memory>
#include <random>
#include "world.h"

class FMTree
{
    friend void show(const FMTree &fmt);

public:
    FMTree(const std::vector<double> &s_init, const std::vector<double> &s_goal, int N, std::shared_ptr<World> world, bool draw=false) : s_init_(s_init),
                                                                                                                        s_goal_(s_goal),
                                                                                                                        N_(N),
                                                                                                                        world_(world),
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
        bool_unvisit_ = std::vector<bool>(N_, true);
        bool_unvisit_[0] = false;
        bool_closed_ = std::vector<bool>(N_, false);
        bool_open_ = std::vector<bool>(N_, false);
        bool_open_[0] = true;
    };

    std::vector<int> solve();
    std::vector<int> getResult()
    {
        return result_;
    }

private:
    bool extend();
    std::vector<double> s_init_;
    std::vector<double> s_goal_;
    std::vector<std::vector<double>> Pset_;
    std::vector<double> cost_;
    std::vector<double> time_;
    std::vector<int> parent_;
    std::vector<bool> bool_unvisit_;
    std::vector<bool> bool_open_;
    std::vector<bool> bool_closed_;
    int N_ = 0;
    std::shared_ptr<World> world_;
    bool draw_=false;
    int itr_ = 0;

    std::vector<int> result_;
};

void show(const FMTree &fmt);
