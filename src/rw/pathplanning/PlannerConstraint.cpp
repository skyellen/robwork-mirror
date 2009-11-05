/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


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
