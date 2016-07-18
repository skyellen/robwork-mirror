#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>

using rw::models::Device;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

QToQPlanner::Ptr getQToQPlanner(
    Device::Ptr device,
    const PlannerConstraint constraint,
    const std::string& type)
{
    if (type == "SBL") {
    	QConstraint::Ptr qconstraint = constraint.getQConstraintPtr();
        return SBLPlanner::makeQToQPlanner(SBLSetup::make(qconstraint, QEdgeConstraintIncremental::makeDefault(qconstraint,device), device));
    } else if (type == "RRT") {
        return RRTPlanner::makeQToQPlanner(constraint, device);
    } else if (type == "ARW") {
        return ARWPlanner::makeQToQPlanner(constraint, device);
    } else {
        return NULL;
    }
}
