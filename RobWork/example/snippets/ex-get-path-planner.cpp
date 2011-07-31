#include <rw/rw.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::pathplanners;

QToQPlanner::Ptr getQToQPlanner(
    Device::Ptr device,
    const PlannerConstraint constraint,
    const std::string& type)
{
    if (type == "SBL") {
        //return SBLPlanner::makeQToQPlanner(
        //    SBLSetup::make(constraint, device));
    } else if (type == "RRT") {
        return RRTPlanner::makeQToQPlanner(constraint, device);
    } else if (type == "ARW") {
        return ARWPlanner::makeQToQPlanner(constraint, device);
    } else {
        return NULL;
    }
}
