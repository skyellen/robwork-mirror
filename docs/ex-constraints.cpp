#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyOpcode.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/math/Q.hpp>
#include <rw/use_robwork_namespace.hpp>
#include <rwlibs/use_robwork_namespace.hpp>
using namespace robwork;

void constraintExample(WorkCell& workcell)
{
    Device* device = workcell.getDevices().front();

    const PlannerConstraint constraint = PlannerConstraint::make(
        CollisionDetector::make(
            &workcell, ProximityStrategyOpcode::make()),
        device,
        workcell.getDefaultState());

    const Q start = device->getBounds().first;
    const Q end = device->getBounds().second;

    std::cout
        << "Start configuration is in collision: "
        << constraint.getQConstraint().inCollision(start)
        << "\n"
        << "End configuration is in collision: "
        << constraint.getQConstraint().inCollision(end)
        << "\n"
        << "Edge from start to end is in collision: "
        << constraint.getQEdgeConstraint().inCollision(start, end)
        << "\n";
}
