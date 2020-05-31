#pragma once

#include "fcl/fcl.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <random>
#include <time.h>
#include <utility>
#include <climits>
// #include "wfaltl.h"

class RoadBoxLTL
{
public:
    RoadBoxLTL() {}
    RoadBoxLTL(double xmin, double xmax, double ymin, double ymax) : xmin_(xmin),
                                                                     xmax_(xmax),
                                                                     ymin_(ymin),
                                                                     ymax_(ymax)
    {
    }
    bool isInside(std::vector<double> s)
    {
        return (xmin_ < s[0] and s[0] < xmax_) and (ymin_ < s[1] and s[1] < ymax_);
    }
    double xmin_ = 0;
    double xmax_ = 0;
    double ymin_ = 0;
    double ymax_ = 0;
};

class RoadLTL
{
public:
    RoadLTL(){};
    RoadLTL(const std::vector<std::vector<double>>& roads, const std::vector<double>& cross)
    {
        num_roads_ = roads.size();
        for (const auto &road : roads)
        {
            RoadBoxLTL roadbox = RoadBoxLTL(road[0], road[1], road[2], road[3]);
            roads_.push_back(roadbox);
        }
        if (!cross.empty())
        {
            exist_cross_ = true;
            cross_ = RoadBoxLTL(cross[0], cross[1], cross[2], cross[3]);
        }
    }

    std::pair<std::vector<bool>, bool> getProposition(std::vector<double> state)
    {
        std::vector<bool> occ_roads = std::vector<bool>(num_roads_, false);
        occ_roads[num_roads_ - 1] = true; //默认在路面外
        for (unsigned i = 0; i < num_roads_; ++i)
        {
            if (roads_[i].isInside(state))
            {
                occ_roads[i] = true;
                occ_roads[num_roads_ - 1] = false;
                break;
            }
        }
        bool occ_cross = cross_.isInside(state);
        return std::make_pair(occ_roads, occ_cross);
    }

    unsigned int num_roads_ = 0;
    bool exist_cross_ = false;
    std::vector<RoadBoxLTL> roads_;
    RoadBoxLTL cross_;
};

class PropositionsVehLTL
{
public:
    bool exist = false;
    bool back = false;
    bool right = false;
    bool left = false;
    bool fron = false;
};

class PropositionsLTL
{
public:
    unsigned int num_roads = 0;
    std::vector<bool> occ_roads;
    bool occ_cross = false;

    PropositionsVehLTL frontVeh;
    PropositionsVehLTL leftVeh;
    PropositionsVehLTL rightVeh;

    bool direction = false;
};

std::ostream  &operator<<(std::ostream &out, PropositionsVehLTL &pro);

std::ostream  &operator<<(std::ostream &out, PropositionsLTL &pro);

class ObjLTL
{
public:
    ObjLTL(){}
    ObjLTL(const std::vector<std::vector<double>> &states, double w, double h) : w_(w), h_(h), states_(states)
    {
    }
    double w_=0;
    double h_=0;
    std::vector<std::vector<double>> states_;
};

class WorldLTL
{
public:
    WorldLTL(const std::unordered_map<unsigned int, ObjLTL> &vehs, const std::vector<ObjLTL> &humans, const RoadLTL &road, bool traffic_light, double xmin, double xmax, double ymin, double ymax, double vxmin, double vxmax, double vymin, double vymax, double tmin, double tmax, double timestep = 0.2, double w = 0.01, double h = 0.01) : vehs_(vehs),
                                                                                                                                                                                                                                                                                                                           humans_(humans),
                                                                                                                                                                                                                                                                                                                           road_(road),
                                                                                                                                                                                                                                                                                                                           traffic_light_(traffic_light),
                                                                                                                                                                                                                                                                                                                           timestep_(timestep),
                                                                                                                                                                                                                                                                                                                           xmin_(xmin),
                                                                                                                                                                                                                                                                                                                           xmax_(xmax),
                                                                                                                                                                                                                                                                                                                           ymin_(ymin),
                                                                                                                                                                                                                                                                                                                           ymax_(ymax),
                                                                                                                                                                                                                                                                                                                           vxmin_(vxmin),
                                                                                                                                                                                                                                                                                                                           vxmax_(vxmax),
                                                                                                                                                                                                                                                                                                                           vymin_(vymin),
                                                                                                                                                                                                                                                                                                                           vymax_(vymax),
                                                                                                                                                                                                                                                                                                                           tmin_(tmin),
                                                                                                                                                                                                                                                                                                                           tmax_(tmax),
                                                                                                                                                                                                                                                                                                                           xdis_(xmin, xmax),
                                                                                                                                                                                                                                                                                                                           ydis_(ymin, ymax),
                                                                                                                                                                                                                                                                                                                           vxdis_(vxmin, vxmax),
                                                                                                                                                                                                                                                                                                                           vydis_(vymin, vymax),
                                                                                                                                                                                                                                                                                                                           tdis_(tmin, tmax),
                                                                                                                                                                                                                                                                                                                           w_(w),
                                                                                                                                                                                                                                                                                                                           h_(h)

    {
        e_.seed(std::time(0));

        manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        for (const auto &veh : vehs_)
        {
            for (const auto &state : veh.second.states_)
            {
                std::shared_ptr<fcl::CollisionGeometryd> objBox = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(veh.second.w_, veh.second.h_, timestep_)); // 長寬高
                fcl::Transform3d objTf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], state[2])));                                                    //中心位置
                fcl::CollisionObjectd *objCol = new fcl::CollisionObjectd(objBox, objTf);
                objs_.push_back(objCol);
            }
        }
        for (const auto &human : humans_)
        {
            for (const auto &state : human.states_)
            {
                std::shared_ptr<fcl::CollisionGeometryd> objBox = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(human.w_, human.h_, timestep_)); // 長寬高
                fcl::Transform3d objTf(fcl::Translation3d(fcl::Vector3d(state[0], state[1], state[2])));                                                        //中心位置
                fcl::CollisionObjectd *objCol = new fcl::CollisionObjectd(objBox, objTf);
                objs_.push_back(objCol);
            }
        }
        manager_->registerObjects(objs_);
        manager_->setup();
        egoBox_ = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(w, h, timestep_));
    }
    ~WorldLTL()
    {
        for (auto &obj : objs_)
        {
            delete obj;
        }
    }

    std::vector<double> randomSample();
    std::vector<double> sampleValid();
    bool isValidState(const std::vector<double> &state);
    bool isValidStates(const std::vector<std::vector<double>> &states);
    PropositionsLTL getProposition(const std::vector<double> &state);
    std::vector<PropositionsLTL> getPropositions(const std::vector<std::vector<double>> &states);

    unsigned int findClosest(const std::vector<double> & s0, std::vector<unsigned int> idxs);
    std::vector<unsigned int> setNearbyVehIds(const std::vector<double> & state);
    PropositionsVehLTL getVehPro(const std::vector<double> &state, unsigned int vehId);


    std::unordered_map<unsigned int, ObjLTL> vehs_;
    unsigned int frontId_=UINT_MAX;
    unsigned int rightId_=UINT_MAX;
    unsigned int leftId_=UINT_MAX;

    std::vector<ObjLTL> humans_;
    RoadLTL road_;
    bool traffic_light_ = false;
    std::vector<std::vector<double>> objShapes_;
    double timestep_ = 0;

private:
    double xmin_ = 0;
    double xmax_ = 0;
    double ymin_ = 0;
    double ymax_ = 0;
    double vxmin_ = 0;
    double vxmax_ = 0;
    double vymin_ = 0;
    double vymax_ = 0;
    double tmin_ = 0;
    double tmax_ = 0;

    std::default_random_engine e_;
    std::uniform_real_distribution<double> xdis_;
    std::uniform_real_distribution<double> ydis_;
    std::uniform_real_distribution<double> vxdis_;
    std::uniform_real_distribution<double> vydis_;
    std::uniform_real_distribution<double> tdis_;
    std::vector<fcl::CollisionObjectd *> objs_;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerd> manager_;
    double w_ = 0;
    double h_ = 0;
    std::shared_ptr<fcl::CollisionGeometryd> egoBox_;
};

bool isFrontLTL(const std::vector<double> &s0, const std::vector<double> &s1);
bool isRightLTL(const std::vector<double> &s0, const std::vector<double> &s1);
bool isBackLTL(const std::vector<double> &s0, const std::vector<double> &s1);
bool isLeftLTL(const std::vector<double> &s0, const std::vector<double> &s1);
