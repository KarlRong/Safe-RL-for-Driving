#pragma once

#include "fcl/fcl.h"
#include <vector>
#include <memory>
#include <iostream>
#include <random>
#include <time.h>

class WorldBvp
{
public:
    WorldBvp(std::vector<std::vector<double>> &objShapes, double xmin, double xmax, double ymin, double ymax, double vxmin, double vxmax, double vymin, double vymax, double tmin, double tmax, double timestep = 0.2, double w = 0.01, double h = 0.01) : objShapes_(objShapes),
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
        T_ = objShapes_.size();
        manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
        for (const auto &objShape : objShapes_)
        {
            if (objShape.size() < 5)
            {
                std::cout << "objShape 需要為5" << std::endl;
                continue;
            }
            std::shared_ptr<fcl::CollisionGeometryd> objBox = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(objShape[2], objShape[3], timestep)); // 長寬高
            fcl::Transform3d objTf(fcl::Translation3d(fcl::Vector3d(objShape[0], objShape[1], objShape[4])));                                                     //中心位置
            fcl::CollisionObjectd *objCol = new fcl::CollisionObjectd(objBox, objTf);
            objs_.push_back(objCol);
        }
        manager_->registerObjects(objs_);
        manager_->setup();
        egoBox_ = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Box<double>(w, h, 1));
    }
    ~WorldBvp()
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
    int T_ = 0;
};
