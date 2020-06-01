#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "worldltl.h"
#include "fmtltl.h"
#include "doublebvp.h"
#include "wfaltl.h"

namespace py = pybind11;

PYBIND11_MODULE(fmtltl, m)
{
    m.doc() = "Fast marching tree with ltl guiding"; // optional module docstring

    m.def("gen_trajectory_bvp", &gen_trajectory_bvp);


    py::class_<WfaLTL, std::shared_ptr<WfaLTL>> wfaltl(m, "WfaLTL");
    wfaltl.def(py::init<>())
        .def("getNextState", &WfaLTL::getNextState)
        .def("inputProposition", &WfaLTL::inputProposition)
        .def_readwrite("state", &WfaLTL::state_)
        .def_readwrite("num", &WfaLTL::num_);

    py::class_<SwitchlaneLTL>(m, "SwitchlaneLTL")
        .def(py::init<>())
        .def("getNextState", &SwitchlaneLTL::getNextState);

    py::class_<LanekeepLTL, std::shared_ptr<LanekeepLTL>>(m, "LanekeepLTL", wfaltl)
        .def(py::init<>())
        .def("getNextState", &LanekeepLTL::getNextState);

    py::class_<LcLeftTakeLTL, std::shared_ptr<LcLeftTakeLTL>>(m, "LcLeftTakeLTL", wfaltl)
        .def(py::init<>())
        .def("getNextState", &LcLeftTakeLTL::getNextState);

    py::class_<LcLeftGiveLTL, std::shared_ptr<LcLeftGiveLTL>>(m, "LcLeftGiveLTL", wfaltl)
        .def(py::init<>())
        .def("getNextState", &LcLeftGiveLTL::getNextState);

    py::class_<LcRightTakeLTL, std::shared_ptr<LcRightTakeLTL>>(m, "LcRightTakeLTL", wfaltl)
        .def(py::init<>())
        .def("getNextState", &LcRightTakeLTL::getNextState);

    py::class_<LcRightGiveLTL, std::shared_ptr<LcRightGiveLTL>>(m, "LcRightGiveLTL", wfaltl)
        .def(py::init<>())
        .def("getNextState", &LcRightGiveLTL::getNextState);

    py::class_<WfaLTLs>(m, "WfaLTLs")
        .def(py::init<>())
        .def("addWfa", &WfaLTLs::addWfa)
        .def("addSwitch", &WfaLTLs::addSwitch)
        .def("addKeepLane", &WfaLTLs::addKeepLane)
        .def("addLcLeftTake", &WfaLTLs::addLcLeftTake)
        .def("addLcLeftGive", &WfaLTLs::addLcLeftGive)
        .def("addLcRightTake", &WfaLTLs::addLcRightTake)
        .def("addLcRightGive", &WfaLTLs::addLcRightGive)
        .def("inputProposition", &WfaLTLs::inputProposition)
        .def("getInitialStates", &WfaLTLs::getInitialStates)
        .def("getNextStates", &WfaLTLs::getNextStates)
        .def_readwrite("wfas", &WfaLTLs::wfas_);

    py::class_<RoadBoxLTL>(m, "RoalBoxLTL")
        .def(py::init<double, double, double, double>())
        .def(py::init<>())
        .def("isInside", &RoadBoxLTL::isInside)
        .def_readwrite("xmin", &RoadBoxLTL::xmin_)
        .def_readwrite("xmax", &RoadBoxLTL::xmax_)
        .def_readwrite("ymin", &RoadBoxLTL::ymin_)
        .def_readwrite("ymax", &RoadBoxLTL::ymax_);

    py::class_<RoadLTL>(m, "RoadLTL")
        .def(py::init<>())
        .def(py::init<const std::vector<std::vector<double>> &, const std::vector<double> &>())
        .def("getProposition", &RoadLTL::getProposition)
        .def_readwrite("num_roads", &RoadLTL::num_roads_)
        .def_readwrite("exist_cross", &RoadLTL::exist_cross_)
        .def_readwrite("roads", &RoadLTL::roads_)
        .def_readwrite("cross", &RoadLTL::cross_);

    py::class_<PropositionsVehLTL>(m, "PropositionsVehLTL")
        .def_readwrite("exist", &PropositionsVehLTL::exist)
        .def_readwrite("back", &PropositionsVehLTL::back)
        .def_readwrite("right", &PropositionsVehLTL::right)
        .def_readwrite("left", &PropositionsVehLTL::left)
        .def_readwrite("fron", &PropositionsVehLTL::fron);

    py::class_<PropositionsLTL>(m, "PropositionsLTL")
        .def_readwrite("num_roads", &PropositionsLTL::num_roads)
        .def_readwrite("occ_roads", &PropositionsLTL::occ_roads)
        .def_readwrite("occ_cross", &PropositionsLTL::occ_cross)
        .def_readwrite("frontVeh", &PropositionsLTL::frontVeh)
        .def_readwrite("leftVeh", &PropositionsLTL::leftVeh)
        .def_readwrite("rightVeh", &PropositionsLTL::rightVeh)
        .def_readwrite("direction", &PropositionsLTL::direction);

    py::class_<ObjLTL>(m, "ObjLTL")
        .def(py::init<>())
        .def(py::init<const std::vector<std::vector<double>> &, double, double>())
        .def_readwrite("w", &ObjLTL::w_)
        .def_readwrite("h", &ObjLTL::h_)
        .def_readwrite("states", &ObjLTL::states_);

    py::class_<WorldLTL, std::shared_ptr<WorldLTL>>(m, "WorldLTL")
        .def(py::init<const std::unordered_map<unsigned int, ObjLTL> &, const std::vector<ObjLTL> &, const RoadLTL &, bool, double, double, double, double, double, double, double, double, double, double, double, double, double>())
        .def("randomSample", &WorldLTL::randomSample)
        .def("sampleValid", &WorldLTL::sampleValid)
        .def("isValidState", &WorldLTL::isValidState)
        .def("isValidStates", &WorldLTL::isValidStates)
        .def("getProposition", &WorldLTL::getProposition)
        .def("getPropositions", &WorldLTL::getPropositions)
        .def("findClosest", &WorldLTL::findClosest)
        .def("setNearbyVehIds", &WorldLTL::setNearbyVehIds)
        .def("getVehPro", &WorldLTL::getVehPro)
        .def_readwrite("vehs", &WorldLTL::vehs_)
        .def_readwrite("humans", &WorldLTL::humans_)
        .def_readwrite("frontId", &WorldLTL::frontId_)
        .def_readwrite("rightId", &WorldLTL::rightId_)
        .def_readwrite("leftId", &WorldLTL::leftId_)
        .def_readwrite("road", &WorldLTL::road_)
        .def_readwrite("traffic_light", &WorldLTL::traffic_light_)
        .def_readwrite("objShapes", &WorldLTL::objShapes_)
        .def_readwrite("timestep", &WorldLTL::timestep_);

    py::class_<FMTreeLTL>(m, "FMTreeLTL")
        .def(py::init<const std::vector<double> &, double, unsigned int, int, std::shared_ptr<WorldLTL>, WfaLTLs, bool>())
        .def("solve", &FMTreeLTL::solve)
        .def("getResult", &FMTreeLTL::getResult)
        .def("goalReached", &FMTreeLTL::goalReached)
        .def("cost_optimal_ltl", &FMTreeLTL::cost_optimal_ltl)
        .def("filter_reachable_ltl", py::overload_cast<const std::vector<int> &, int, double, double, double, double, bool>(&FMTreeLTL::filter_reachable_ltl))
        .def("filter_reachable_ltl", py::overload_cast<const std::list<int> &, int, double, double, double, double, bool>(&FMTreeLTL::filter_reachable_ltl))
        .def("decideGoal", &FMTreeLTL::decideGoal)
        .def("decideFromOpen", &FMTreeLTL::decideFromOpen)
        .def_readwrite("world", &FMTreeLTL::world_)
        .def_readwrite("ux_limit", &FMTreeLTL::ux_limit_)
        .def_readwrite("uy_limit", &FMTreeLTL::uy_limit_)
        .def_readwrite("T_limit", &FMTreeLTL::T_limit_)
        .def_readwrite("r", &FMTreeLTL::r_)
        .def_readwrite("s_init", &FMTreeLTL::s_init_)
        .def_readwrite("exp_speed", &FMTreeLTL::exp_speed_)
        .def_readwrite("lc", &FMTreeLTL::lc_)
        .def_readwrite("s_goal", &FMTreeLTL::s_goal_)
        .def_readwrite("N", &FMTreeLTL::N_)
        .def_readwrite("Pset", &FMTreeLTL::Pset_)
        .def_readwrite("wfas", &FMTreeLTL::wfas_)
        .def_readwrite("Wfa_states", &FMTreeLTL::Wfa_states_)
        .def_readwrite("cost", &FMTreeLTL::cost_)
        .def_readwrite("cost_ltl", &FMTreeLTL::cost_ltl_)
        .def_readwrite("cost_speed", &FMTreeLTL::cost_speed_)
        .def_readwrite("time", &FMTreeLTL::time_)
        .def_readwrite("parent", &FMTreeLTL::parent_)
        .def_readwrite("unvisit", &FMTreeLTL::unvisit_)
        .def_readwrite("open", &FMTreeLTL::open_)
        .def_readwrite("closed", &FMTreeLTL::closed_)
        .def_readwrite("ltl_factor", &FMTreeLTL::ltl_factor)
        .def_readwrite("speed_factor", &FMTreeLTL::speed_factor)
        .def_readwrite("closed", &FMTreeLTL::closed_);
}
