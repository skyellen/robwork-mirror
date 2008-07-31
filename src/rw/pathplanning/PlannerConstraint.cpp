/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PlannerConstraint.hpp"

using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;

PlannerConstraint::PlannerConstraint(
    QConstraintPtr constraint,
    QEdgeConstraintPtr edge)
    :
    _constraint(constraint),
    _edge(edge)
{
    RW_ASSERT(_constraint);
    RW_ASSERT(_edge);
}

PlannerConstraint PlannerConstraint::make(QConstraintPtr constraint, QEdgeConstraintPtr edge)
{
    return PlannerConstraint(constraint, edge);
}

PlannerConstraint PlannerConstraint::make(
    CollisionDetectorPtr detector,
    DevicePtr device,
    const State& state)
{
    QConstraintPtr constraint =
        QConstraint::make(detector, device, state);

    QEdgeConstraintPtr edge =
        QEdgeConstraint::makeDefault(constraint, device);

    return make(constraint, edge);
}

PlannerConstraint PlannerConstraint::make(
    CollisionStrategyPtr strategy,
    WorkCellPtr workcell,
    DevicePtr device,
    const State& state)
{
    return make(
        CollisionDetector::make(workcell, strategy),
        device,
        state);
}

PlannerConstraint PlannerConstraint::make(
    CollisionStrategyPtr strategy,
    const CollisionSetup& setup,
    WorkCellPtr workcell,
    DevicePtr device,
    const State& state)
{
    return make(
        CollisionDetector::make(workcell, strategy, setup),
        device,
        state);
}
