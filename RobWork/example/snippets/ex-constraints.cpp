#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::proximitystrategies;

void constraintExample(WorkCell& workcell)
{
    Device::Ptr device = workcell.getDevices().front();

    const PlannerConstraint constraint = PlannerConstraint::make(
        ownedPtr( new CollisionDetector(
            &workcell, ProximityStrategyYaobi::make() ) ),
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
