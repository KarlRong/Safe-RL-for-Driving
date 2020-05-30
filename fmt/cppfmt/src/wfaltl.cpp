#include "wfaltl.h"
#include <iostream>

double WfaLTLs::inputProposition(const PropositionsLTL &pro)
{
    double cost = 0;
    for (const auto &wfa : wfas_)
    {
        double this_cost = wfa->inputProposition(pro);
        cost += this_cost;
    }
    return cost;
}

std::pair<std::vector<unsigned int>, double> WfaLTLs::getNextStates(const std::vector<unsigned int> &states, const PropositionsLTL &pro)
{
    double cost = 0;
    std::vector<unsigned int> next_states;
    for (unsigned int i = 0; i < wfas_.size(); ++i)
    {
        auto ret = wfas_[i]->getNextState(states[i], pro);
        next_states.push_back(ret.first);
        cost += ret.second;
    }
    return std::pair<std::vector<unsigned int>, double>(next_states, cost);
}

std::pair<unsigned int, double> SwitchlaneLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double c_out = 1;
    double c_out_keep = 0.1;
    double c_dash = 0.4;
    double c_solid = 0.8;
    if (pro.num_roads == 4) //限制为4车道
    {
        switch (cur)
        {
        case 0:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = 0;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = 0;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = 0;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = 0;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out;
            }
            break;

        case 1:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = 0;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = c_dash;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out;
            }
            break;

        case 2:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = c_dash;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = 0;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = c_solid;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out;
            }
            break;
        case 3:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = c_solid;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = 0;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = c_dash;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out;
            }
            break;
            break;
        case 4:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = c_dash + c_solid;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = c_dash;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = 0;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out;
            }
            break;
        case 5:
            if (pro.occ_roads[0])
            {
                next_state = 1;
                cost = 0;
            }
            else if (pro.occ_roads[1])
            {
                next_state = 2;
                cost = c_dash;
            }
            else if (pro.occ_roads[2])
            {
                next_state = 3;
                cost = c_dash;
            }
            else if (pro.occ_roads[3])
            {
                next_state = 4;
                cost = 0;
            }
            else if (pro.occ_roads[4])
            {
                next_state = 5;
                cost = c_out_keep;
            }
            break;
        default:
            std::cerr << "SwitchlaneLTL wft state error" << std::endl;
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}

std::pair<unsigned int, double> LanekeepLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double cost_notkeep = 3;
    if (pro.num_roads == 4) //限制为4车道
    {
        if (pro.frontVeh.exist and pro.frontVeh.fron)
        {
            next_state = 6;
            cost = cost_notkeep;
        }
        else
        {
            switch (cur)
            {
            case 0:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else if (pro.occ_roads[2])
                {
                    next_state = 3;
                    cost = 0;
                }
                else
                {
                    next_state = 5;
                    cost = cost_notkeep;
                }
                break;

            case 1:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 3;
                    cost = cost_notkeep;
                }
                else
                {
                    next_state = 5;
                    cost = cost_notkeep;
                }
                break;

            case 2:
                if (pro.occ_roads[0])
                {
                    next_state = 4;
                    cost = cost_notkeep;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 5;
                    cost = cost_notkeep;
                }
                break;
            case 3:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_notkeep;
                }
                break;
            case 4:
                if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_notkeep;
                }
                break;
            case 5:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 5;
                    cost = cost_notkeep;
                }
                break;

            default:
                std::cerr << "LanekeepLTL wft state error" << std::endl;
            }
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}

std::pair<unsigned int, double> LcLeftTakeLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double cost_stay = 0.1;
    double cost_outrange = 3;
    if (pro.num_roads == 4) //限制为4车道
    {
        if (pro.leftVeh.exist)
        {
            switch (cur)
            {
            case 0:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.leftVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 3:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcLeftGiveLTL wft state error" << std::endl;
            }
        }
        else
        {
            switch (cur)
            {
            case 0:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 3;
                    cost = cost_stay;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 3:
                next_state = 3;
                cost = cost_stay;
                break;
            case 4:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcLeftGiveLTL wft state error" << std::endl;
            }
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}

std::pair<unsigned int, double> LcLeftGiveLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double cost_stay = 0.1;
    double cost_outrange = 3;
    if (pro.num_roads == 4) //限制为4车道
    {
        if (pro.leftVeh.exist)
        {
            switch (cur)
            {
            case 0:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.leftVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 3:
                if (pro.leftVeh.right)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.leftVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcLeftGiveLTL wft state error" << std::endl;
            }
        }
        else
        {
            switch (cur)
            {
            case 0:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 3;
                    cost = cost_stay;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 3:
                next_state = 3;
                cost = cost_stay;
                break;
            case 4:
                if (pro.occ_roads[0])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[1])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcLeftGiveLTL wft state error" << std::endl;
            }
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}

std::pair<unsigned int, double> LcRightTakeLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double cost_stay = 0.1;
    double cost_outrange = 3;
    if (pro.num_roads == 4) //限制为4车道
    {
        if (pro.rightVeh.exist)
        {
            switch (cur)
            {
            case 0:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.rightVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 3:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.fron)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcRightTakeLTL wft state error" << std::endl;
            }
        }
        else
        {
            switch (cur)
            {
            case 0:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 3;
                    cost = cost_stay;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 3:
                next_state = 3;
                cost = cost_stay;
                break;
            case 4:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcRightTakeLTL wft state error" << std::endl;
            }
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}

std::pair<unsigned int, double> LcRightGiveLTL::getNextState(unsigned int cur, const PropositionsLTL &pro)
{
    unsigned int next_state = cur;
    double cost = 0;
    double cost_stay = 0.1;
    double cost_outrange = 3;
    if (pro.num_roads == 4) //限制为4车道
    {
        if (pro.rightVeh.exist)
        {
            switch (cur)
            {
            case 0:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.rightVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            case 3:
                if (pro.rightVeh.left)
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.rightVeh.back)
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 3;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcLeftGiveLTL wft state error" << std::endl;
            }
        }
        else
        {
                       switch (cur)
            {
            case 0:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 3;
                    cost = cost_stay;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 1:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 2:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = cost_stay;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            case 3:
                next_state = 3;
                cost = cost_stay;
                break;
            case 4:
                if (pro.occ_roads[1])
                {
                    next_state = 1;
                    cost = 0;
                }
                else if (pro.occ_roads[0])
                {
                    next_state = 2;
                    cost = 0;
                }
                else
                {
                    next_state = 4;
                    cost = cost_outrange;
                }
                break;
            default:
                std::cerr << "LcRightTakeLTL wft state error" << std::endl;
            }
        }
    }
    return std::pair<unsigned int, double>(next_state, cost);
}
