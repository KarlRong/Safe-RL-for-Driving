#pragma once

#include "fcl/fcl.h"
#include <vector>
#include <memory>
#include <iostream>
#include <random>
#include <time.h>

class World
{
public:
    World(std::vector<std::vector<double>> &objShapes, double xmin, double xmax, double ymin, double ymax, double vxmin, double vxmax, double vymin, double vymax, double w = 0.01, double h = 0.01) : objShapes_(objShapes),
                                                                                                                                                                                                       xmin_(xmin),
                                                                                                                                                                                                       xmax_(xmax),
                                                                                                                                                                                                       ymin_(ymin),
                                                                                                                                                                                                       ymax_(ymax),
                                                                                                                                                                                                       vxmin_(vxmin),
                                                                                                                                                                                                       vxmax_(vxmax),
                                                                                                                                                                                                       vymin_(vymin),
                                                                                                                                                                                                       vymax_(vymax),
                                                                                                                                                                                                       xdis_(xmin, xmax),
                                                                                                                                                                                                       ydis_(ymin, ymax),
                                                                                                                                                                                                       vxdis_(vxmin, vxmax),
                                                                                                                                                                                                       vydis_(vymin, vymax),
                                                                                                                                                                                                       w_(w),
                                                                                                                                                                                                       h_(h)
    {
        e_.seed(std::time(0));
        T_ = objShapes_.size();
        manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        for (const auto &objShape : objShapes_)
        {
            if (objShape.size() < 5)
            {
                std::cout << "objShape 需要為5" << std::endl;
                continue;
            }
            std::shared_ptr<fcl::CollisionGeometryd> objBox = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(objShape[3], objShape[4], 1)); // 長寬高
            fcl::Transform3d objTf(fcl::Translation3d(fcl::Vector3d(objShape[0], objShape[1], objShape[2])));                                              //中心位置
            fcl::CollisionObjectd *objCol = new fcl::CollisionObjectd(objBox, objTf);
            objs_.push_back(objCol);
        }
        manager_->registerObjects(objs_);
        manager_->setup();
        egoBox_ = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(w, h, 1));
    }
    ~World()
    {
        for (auto &obj : objs_)
        {
            delete obj;
        }
    }

    std::vector<double> randomSample();
    std::vector<double> sampleValid();
    bool isValidState(const std::vector<double> &state, int startT = 0);
    bool isValidStates(const std::vector<std::vector<double>> &states, int startT = 0);

private:
    std::vector<std::vector<double>> objShapes_;
    double xmin_=0;
    double xmax_=0;
    double ymin_=0;
    double ymax_=0;
    double vxmin_=0;
    double vxmax_=0;
    double vymin_=0;
    double vymax_=0;
    std::default_random_engine e_;
    std::uniform_real_distribution<double> xdis_;
    std::uniform_real_distribution<double> ydis_;
    std::uniform_real_distribution<double> vxdis_;
    std::uniform_real_distribution<double> vydis_;
    std::vector<fcl::CollisionObjectd *> objs_;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerd> manager_;
    double w_=0;
    double h_=0;
    std::shared_ptr<fcl::CollisionGeometryd> egoBox_;
    int T_ = 0;
};

