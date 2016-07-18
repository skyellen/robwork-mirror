#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>

using rw::common::ownedPtr;
using rw::math::Q;
using namespace rw::models;
using rw::pathplanning::PlannerConstraint;
using rw::proximity::CollisionDetector;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;

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
