/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtTactileSensor.hpp"
#include "BtConstraint.hpp"
#include "BtUtil.hpp"

#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.h>

#include <boost/foreach.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsimlibs::bullet;

BtTactileSensor::BtTactileSensor(rw::common::Ptr<SimulatedTactileSensor> sensor):
	_rwSensor(sensor)
{
}

BtTactileSensor::~BtTactileSensor() {
	BOOST_FOREACH(btJointFeedback* const feedback, _feedbacks) {
		delete feedback;
	}
	_feedbacks.clear();
}

void BtTactileSensor::update(const Simulator::UpdateInfo& info, State& state) const {
	// Remember to check if it should be added positive or negative (relative to parent or child?)
	RW_ASSERT(_feedbacks.size() == _constraints.size());
	for (std::size_t i = 0; i < _feedbacks.size(); i++) {
		btJointFeedback* const feedback = _feedbacks[i];
		BtConstraint* const constraint = _constraints[i];
		rw::common::Ptr<const Constraint> rwConstraint = constraint->getRWConstraint();
		if(const rw::common::Ptr<SimulatedFTSensor> tsensor = _rwSensor.cast<SimulatedFTSensor>()){
			Vector3D<> force, torque;
			if (rwConstraint->getBody1() == tsensor->getBody1() && rwConstraint->getBody2() == tsensor->getBody2()) {
				force = BtUtil::toVector3D(feedback->m_appliedForceBodyB);
				torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyB);
				const Vector3D<> pos = constraint->getRefB();
				_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,tsensor->getBody2());
				_rwSensor->addForceW(pos,force,Vector3D<>::zero(),state,tsensor->getBody2());
			} else {
				force = BtUtil::toVector3D(feedback->m_appliedForceBodyA);
				torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyA);
				const Vector3D<> pos = constraint->getRefA();
				_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,tsensor->getBody2());
				_rwSensor->addForceW(pos,force,Vector3D<>::zero(),state,tsensor->getBody2());
			}
		}
	}
	_rwSensor->update(info, state);
}

void BtTactileSensor::addFeedback(BtConstraint* const constraint) {
	_constraints.push_back(constraint);
	_feedbacks.push_back(new btJointFeedback());
	constraint->getBtConstraint()->setJointFeedback(_feedbacks.back());
}
