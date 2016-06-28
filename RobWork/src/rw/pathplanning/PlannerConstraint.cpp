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

#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rw/proximity/CollisionDetector.hpp>

using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::common;

PlannerConstraint::PlannerConstraint() {

}

PlannerConstraint::PlannerConstraint(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge)
    :
    _constraint(constraint),
    _edge(edge)
{
    //RW_ASSERT(_constraint);
    //RW_ASSERT(_edge);
}

PlannerConstraint PlannerConstraint::make(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge)
{
    return PlannerConstraint(constraint, edge);
}

PlannerConstraint PlannerConstraint::make(CollisionDetector::Ptr detector,
		rw::common::Ptr<Device> device,
		const State& state)
{
	QConstraint::Ptr constraint =
        QConstraint::make(detector, device, state);

	QEdgeConstraint::Ptr edge =
        QEdgeConstraint::makeDefault(constraint, device);

    return make(constraint, edge);
}

PlannerConstraint PlannerConstraint::make(
	rw::common::Ptr<CollisionStrategy> strategy,
	rw::common::Ptr<WorkCell> workcell,
	rw::common::Ptr<Device> device,
    const State& state)
{
	CollisionDetector::Ptr cdect = ownedPtr(new CollisionDetector(workcell, strategy));
    return make(cdect, device, state);
}

PlannerConstraint PlannerConstraint::make(
	rw::common::Ptr<CollisionStrategy> strategy,
    const CollisionSetup& setup,
	rw::common::Ptr<WorkCell> workcell,
	rw::common::Ptr<Device> device,
    const State& state)
{
	BasicFilterStrategy::Ptr bpfilter = ownedPtr(new BasicFilterStrategy(workcell, setup));
	CollisionDetector::Ptr cdect = ownedPtr(new CollisionDetector(workcell, strategy, bpfilter));

	return make(cdect, device, state);
}


bool PlannerConstraint::inCollision(const rw::math::Q& q) {
	return _constraint->inCollision(q);
}

bool PlannerConstraint::inCollision(const rw::math::Q& q1, const rw::math::Q& q2) {
	return _edge->inCollision(q1, q2);
}
