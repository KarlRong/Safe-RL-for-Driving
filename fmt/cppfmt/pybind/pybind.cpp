#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "worldbvp.h"
#include "fmtbvp.h"
#include "doublebvp.h"

namespace py = pybind11;

PYBIND11_MODULE(fmtbvp, m)
{
    m.doc() = "Fast marching tree with double integrator"; // optional module docstring

    m.def("solveBVP", &solveBVP);
    m.def("cost_optimal_bvp", &cost_optimal_bvp);
    m.def("reachable_box_bvp", &reachable_box_bvp);
    m.def("backward_reachable_box_bvp", &backward_reachable_box_bvp);
    m.def("forward_reachable_box_bvp", &forward_reachable_box_bvp);
    m.def("filter_reachable_bvp", py::overload_cast<const std::vector<std::vector<double>> &, const std::vector<int> &, std::vector<double> &, double, double, double, double, bool>(&filter_reachable_bvp));
    m.def("filter_reachable_bvp", py::overload_cast<const std::vector<std::vector<double>> &, const std::list<int> &, std::vector<double> &,  double, double, double, double, bool>(&filter_reachable_bvp));
    m.def("gen_trajectory_bvp", &gen_trajectory_bvp);

    py::class_<WorldBvp, std::shared_ptr<WorldBvp>>(m, "WorldBvp")
        .def(py::init<std::vector<std::vector<double>> &, double, double, double, double, double, double, double, double, double, double,double , double, double>())
        .def("randomSample", &WorldBvp::randomSample)
        .def("sampleValid", &WorldBvp::sampleValid)
        .def("isValidState", &WorldBvp::isValidState)
        .def("isValidStates", &WorldBvp::isValidStates)
        .def_readwrite("objShapes", &WorldBvp::objShapes_)
        .def_readwrite("timestep", &WorldBvp::timestep_);
    py::class_<FMTreeBvp>(m, "FMTreeBvp")
        .def(py::init<const std::vector<double> &, const std::vector<double> &, int, std::shared_ptr<WorldBvp>, bool>())
        .def("solve", &FMTreeBvp::solve)
        .def("getResult", &FMTreeBvp::getResult)
        .def("goalReached", &FMTreeBvp::goalReached)
        .def_readwrite("world", &FMTreeBvp::world_)
        .def_readwrite("ux_limit", &FMTreeBvp::ux_limit_)
        .def_readwrite("uy_limit", &FMTreeBvp::uy_limit_)
        .def_readwrite("T_limit", &FMTreeBvp::T_limit_)
        .def_readwrite("r", &FMTreeBvp::r_)
        .def_readwrite("s_init", &FMTreeBvp::s_init_)
        .def_readwrite("s_goal", &FMTreeBvp::s_goal_)
        .def_readwrite("N", &FMTreeBvp::N_)
        .def_readwrite("Pset", &FMTreeBvp::Pset_)
        .def_readwrite("cost", &FMTreeBvp::cost_)
        .def_readwrite("time", &FMTreeBvp::time_)
        .def_readwrite("parent", &FMTreeBvp::parent_)
        .def_readwrite("unvisit", &FMTreeBvp::unvisit_)
        .def_readwrite("open", &FMTreeBvp::open_)
        .def_readwrite("closed", &FMTreeBvp::closed_);
}
