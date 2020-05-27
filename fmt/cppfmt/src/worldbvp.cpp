#include "worldbvp.h"
#include "fcl_utility.h"
#include "matplotlibcpp.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"


std::vector<double> WorldBvp::randomSample()
{
    return std::vector<double>({xdis_(e_), ydis_(e_), vxdis_(e_), vydis_(e_), tdis_(e_)});
}

std::vector<double> WorldBvp::sampleValid()
{
    int trial = 0;
    std::vector<double> sample;
    while(true)
    {
        ++trial;
        sample = randomSample();
        if(isValidState(sample))
        {
            break;
        }
    }
    return sample;
}

bool WorldBvp::isValidState(const std::vector<double> &state)
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

bool WorldBvp::isValidStates(const std::vector<std::vector<double>> &states)
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

    fcl::DynamicAABBTreeCollisionManagerd* statesManager = new fcl::DynamicAABBTreeCollisionManagerd();
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

