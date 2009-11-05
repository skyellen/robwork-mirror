#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

QToQPlannerPtr getQToQPlanner(
    DevicePtr device,
    const PlannerConstraint constraint,
    const std::string& type)
{
    if (type == "SBL") {
        return SBLPlanner::makeQToQPlanner(
            SBLSetup::make(constraint, device));
    } else if (type == "RRT") {
        return RRTPlanner::makeQToQPlanner(constraint, device);
    } else if (type == "ARW") {
        return ARWPlanner::makeQToQPlanner(constraint, device);
    } else {
        return NULL;
    }
}
