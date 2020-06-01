#pragma once

#include "worldltl.h"
#include <vector>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>

class WfaLTL
{
public:
    WfaLTL() = default;
    virtual std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro)
    {
        return std::pair<unsigned int, double>(cur, 0);
    }
    virtual double inputProposition(const PropositionsLTL &pro)
    {
        std::pair<unsigned int, double> ret = getNextState(state_, pro);
        state_ = ret.first;
        return ret.second;
    };

    unsigned int state_ = 0;
    unsigned int num_ = 0;
};

class WfaLTLs
{
public:
    std::vector<std::shared_ptr<WfaLTL>> wfas_;

    void addWfa(std::shared_ptr<WfaLTL> wfa)
    {
        wfas_.push_back(wfa);
    }

    void addSwitch();
    void addKeepLane();
    void addLcLeftTake();
    void addLcLeftGive();
    void addLcRightTake();
    void addLcRightGive();
    double inputProposition(const PropositionsLTL &pro);
    std::vector<unsigned int> getInitialStates()
    {
        return std::vector<unsigned int>(wfas_.size(), 0);
    }
    std::pair<std::vector<unsigned int>, double> getNextStates(const std::vector<unsigned int> &states, const PropositionsLTL &pro);
};

class SwitchlaneLTL : public WfaLTL
{
public:
    SwitchlaneLTL()
    {
        num_ = 6; // state数量
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};

class LanekeepLTL : public WfaLTL
{
public:
    LanekeepLTL()
    {
        num_ = 7;
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};

class LcLeftTakeLTL : public WfaLTL
{
public:
    LcLeftTakeLTL()
    {
        num_ = 4;
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};

class LcLeftGiveLTL : public WfaLTL
{
public:
    LcLeftGiveLTL()
    {
        num_ = 4;
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};

class LcRightTakeLTL : public WfaLTL
{
public:
    LcRightTakeLTL()
    {
        num_ = 4;
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};

class LcRightGiveLTL : public WfaLTL
{
public:
    LcRightGiveLTL()
    {
        num_ = 4;
    }
    std::pair<unsigned int, double> getNextState(unsigned int cur, const PropositionsLTL &pro);
};
