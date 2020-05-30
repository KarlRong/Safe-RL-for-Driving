#include "worldltl.h"
#include "fcl_utility.h"
#include "matplotlibcpp.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include <float.h>
#include <math.h>

std::vector<double> WorldLTL::randomSample()
{
    return std::vector<double>({xdis_(e_), ydis_(e_), vxdis_(e_), vydis_(e_), tdis_(e_)});
}

std::vector<double> WorldLTL::sampleValid()
{
    int trial = 0;
    std::vector<double> sample;
    while (true)
    {
        ++trial;
        sample = randomSample();
        if (isValidState(sample))
        {
            break;
        }
    }
    return sample;
}

bool WorldLTL::isValidState(const std::vector<double> &state)
{
    if (state.size() < 5)
    {
        std::cout << "state 需要為5" << std::endl;
        return false;
    }
    bool collision = true;
    CollisionData<double> collision_data;
    fcl::Transform3d ego_tf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], state[4]))); //没有旋转
    // fcl::Transform3d ego_tf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], 0)));
    fcl::CollisionObjectd *egoCol = new fcl::CollisionObjectd(egoBox_, ego_tf);
    manager_->collide(egoCol, &collision_data, defaultCollisionFunction);
    collision = collision_data.result.isCollision();
    return !collision;
}

bool WorldLTL::isValidStates(const std::vector<std::vector<double>> &states)
{
    // if (startT + states.size() > T_)
    // {
    //     return false;
    // }
    // if (startT > T_)
    // {
    //     return false;
    // }
    // states 的 T未考虑特殊情况
    bool collision = true;
    CollisionData<double> collision_data;

    fcl::DynamicAABBTreeCollisionManagerd *statesManager = new fcl::DynamicAABBTreeCollisionManagerd();
    std::vector<fcl::CollisionObjectd *> egos;
    for (const auto &state : states)
    {
        fcl::Transform3d ego_tf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], state[4])));
        // fcl::Transform3d ego_tf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], state[2])));
        fcl::CollisionObjectd *egoCol = new fcl::CollisionObjectd(egoBox_, ego_tf);
        egos.push_back(egoCol);
    }
    statesManager->registerObjects(egos);
    statesManager->setup();
    manager_->collide(statesManager, &collision_data, defaultCollisionFunction<double>);

    collision = collision_data.result.isCollision();
    for (auto &ego : egos)
    {
        delete ego;
    }
    delete statesManager;
    return !collision;
}

bool isFrontLTL(const std::vector<double> &s0, const std::vector<double> &s1)
{
    double frontdis = s1[0] - s0[0];
    double latdis = s1[1] - s0[1];

    if (frontdis > 4 and frontdis < 15 and latdis > -1.75 and latdis < 1.75) //这里需要写死车辆长度
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool isRightLTL(const std::vector<double> &s0, const std::vector<double> &s1)
{
    double latdis = s1[1] - s0[1];
    if (latdis > -5.3 and latdis < -1.75)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool isBackLTL(const std::vector<double> &s0, const std::vector<double> &s1)
{
    double frontdis = s1[0] - s0[0];
    double latdis = s1[1] - s0[1];

    if (frontdis > -15 and frontdis < -5 and latdis > -1.75 and latdis < 1.75)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool isLeftLTL(const std::vector<double> &s0, const std::vector<double> &s1)
{
    double latdis = s1[1] - s0[1];
    if (latdis > 1.75 and latdis < 5.3)
    {
        return true;
    }
    else
    {
        return false;
    }
}

unsigned int WorldLTL::findClosest(const std::vector<double> &s0, std::vector<unsigned int> idxs)
{
    unsigned int ret = UINT_MAX;
    if (idxs.size() == 0)
    {
        return ret;
    }
    if (idxs.size() == 1)
    {
        return idxs[0];
    }
    else
    {
        unsigned int idx_time = std::floor((s0[4] - tmin_) / timestep_);
        double min_dis = DBL_MAX;
        for (const auto &idx : idxs)
        {
            double dis = std::fabs(vehs_[idx].states_[idx_time][0] - s0[0]) + std::fabs(vehs_[idx].states_[idx_time][1] - s0[1]);
            if (dis < min_dis)
            {
                ret = idx;
                min_dis = dis;
            }
        }
    }
    return ret;
}
std::vector<unsigned int> WorldLTL::setNearbyVehIds(const std::vector<double> &state)
{
    PropositionsVehLTL ret;
    std::vector<unsigned int> front_list, right_list, left_list;
    unsigned int idx_time = std::floor((state[4] - tmin_) / timestep_);

    for (const auto &veh : vehs_)
    {
        std::vector<double> s0 = vehs_[veh.first].states_[idx_time];
        if (road_.roads_[0].isInside(s0) or road_.roads_[1].isInside(s0)) //只考虑下方两车道的车作为目标
        {
            if (isFrontLTL(state, s0))
            {
                front_list.push_back(veh.first);
            }
            if (isRightLTL(state, s0))
            {
                right_list.push_back(veh.first);
            }
            if (isLeftLTL(state, s0))
            {
                left_list.push_back(veh.first);
            }
        }
    }
    frontId_ = findClosest(state, front_list);
    rightId_ = findClosest(state, right_list);
    leftId_ = findClosest(state, left_list);
    return std::vector<unsigned int>({frontId_, rightId_, leftId_});
}

std::ostream &operator<<(std::ostream &out, PropositionsVehLTL &pro)
{
    if (pro.exist)
    {
        if (pro.back)
        {
            out << "behind ";
        }
        if (pro.right)
        {
            out << "right";
        }
        if (pro.left)
        {
            out << "left";
        }
        if (pro.fron)
        {
            out << "front";
        }
    }
    else
    {
        out << "not exist";
    }

    return out;
}

std::ostream &operator<<(std::ostream &out, PropositionsLTL &pro)
{
    out << "Roads: " << std::endl;
    for (unsigned int i = 0; i < pro.num_roads; ++i)
    {
        out << "road " << i << ": " << pro.occ_roads[i] << std::endl;
    }
    out << "Crosswalk: " << pro.occ_cross << std::endl;
    out << "Front car:  " << pro.frontVeh << std::endl;
    out << "Left car:   " << pro.leftVeh << std::endl;
    out << "Right car:  " << pro.rightVeh << std::endl;
    out << "Right direction: " << pro.direction;
    return out;
}

PropositionsVehLTL WorldLTL::getVehPro(const std::vector<double> &state, unsigned int vehId)
{
    PropositionsVehLTL ret;
    if (vehId == UINT_MAX)
    {
        return ret;
    }
    ret.exist = true;
    unsigned int idx_time = std::floor((state[4] - tmin_) / timestep_);
    std::vector<double> s0 = vehs_[vehId].states_[idx_time];
    if (isFrontLTL(s0, state))
    {
        ret.fron = true;
    }
    if (isRightLTL(s0, state))
    {
        ret.right = true;
    }
    if (isLeftLTL(s0, state))
    {
        ret.left = true;
    }
    if (isBackLTL(s0, state))
    {
        ret.back = true;
    }
    return ret;
}

PropositionsLTL WorldLTL::getProposition(const std::vector<double> &state)
{
    PropositionsLTL pro;
    pro.num_roads = road_.num_roads_;
    auto road_pro = road_.getProposition(state);
    pro.occ_roads = road_pro.first;
    pro.occ_cross = road_pro.second;

    pro.frontVeh = getVehPro(state, frontId_);
    pro.rightVeh = getVehPro(state, rightId_);
    pro.leftVeh = getVehPro(state, leftId_);

    if (state[3] >= 0) //只向右行驶
    {
        pro.direction = true;
    }
    return pro;
}

std::vector<PropositionsLTL> WorldLTL::getPropositions(const std::vector<std::vector<double>> &states)
{
    std::vector<PropositionsLTL> ret;
    for (const auto &state : states)
    {
        ret.push_back(getProposition(state));
    }
    return ret;
}
