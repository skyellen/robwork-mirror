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
#include "BtBody.hpp"

#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
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
}

void BtTactileSensor::addConstraintsFeedback(const Simulator::UpdateInfo& info, State& state) const {
	// Remember to check if it should be added positive or negative (relative to parent or child?)
	for (std::size_t i = 0; i < _constraints.size(); i++) {
		BtConstraint* const constraint = _constraints[i];
		btJointFeedback* const feedback = constraint->getFeedback();
		rw::common::Ptr<const Constraint> rwConstraint = constraint->getRWConstraint();
		if(const rw::common::Ptr<SimulatedTactileSensor> tsensor = _rwSensor.cast<SimulatedTactileSensor>()){
			if(const rw::common::Ptr<SimulatedFTSensor> ftsensor = tsensor.cast<SimulatedFTSensor>()){
				// Add constraints
				if (rwConstraint->getBody1() == ftsensor->getBody1() && rwConstraint->getBody2() == ftsensor->getBody2()) {
					const Vector3D<> force = BtUtil::toVector3D(feedback->m_appliedForceBodyB);
					const Vector3D<> torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyB);
					const Vector3D<> pos = constraint->getRefB();
					_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,ftsensor->getBody2());
					_rwSensor->addForceW(pos,force,Vector3D<>::zero(),state,ftsensor->getBody2());
				} else {
					const Vector3D<> force = BtUtil::toVector3D(feedback->m_appliedForceBodyA);
					const Vector3D<> torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyA);
					const Vector3D<> pos = constraint->getRefA();
					_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,ftsensor->getBody2());
					_rwSensor->addForceW(pos,force,Vector3D<>::zero(),state,ftsensor->getBody2());
				}
			} else {
				// Add constraints
				if (rwConstraint->getBody1()->getBodyFrame() == tsensor->getFrame()) {
					const Vector3D<> force = BtUtil::toVector3D(feedback->m_appliedForceBodyA);
					const Vector3D<> torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyA);
					const Vector3D<> pos = constraint->getRefA();
					_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,rwConstraint->getBody1());
					_rwSensor->addForceW(pos,force,Vector3D<>::zero(),state,rwConstraint->getBody1());
				} else {
					const Vector3D<> force = BtUtil::toVector3D(feedback->m_appliedForceBodyB);
					const Vector3D<> torque = BtUtil::toVector3D(feedback->m_appliedTorqueBodyB);
					const Vector3D<> pos = constraint->getRefB();
					_rwSensor->addWrenchWToCOM(Vector3D<>::zero(),torque,state,rwConstraint->getBody2());
					_rwSensor->addForceW(pos,-force,Vector3D<>::zero(),state,rwConstraint->getBody2());
				}
			}
		}
	}
	_rwSensor->update(info, state);
}

void BtTactileSensor::addContactManifold(const Simulator::UpdateInfo& info, State& state, const btPersistentManifold* manifold, const BtBody* bodyA, const BtBody* bodyB) const {
	if(const rw::common::Ptr<SimulatedTactileSensor> tsensor = _rwSensor.cast<SimulatedTactileSensor>()){
		if(const rw::common::Ptr<SimulatedFTSensor> ftsensor = tsensor.cast<SimulatedFTSensor>()){
			// Add contacts
			if (bodyA->getRwBody() == ftsensor->getBody1() && bodyB->getRwBody() == ftsensor->getBody2()) {
				for (int i = 0; i < manifold->getNumContacts(); i++) {
					const btManifoldPoint& point = manifold->getContactPoint(i);
					const btScalar force = point.getAppliedImpulse()/info.dt;
					const Vector3D<> pos = BtUtil::toVector3D(point.getPositionWorldOnB());
					const Vector3D<> n = BtUtil::toVector3D(point.m_normalWorldOnB);
					_rwSensor->addForceW(pos,force*n,n,state,bodyB->getRwBody());
				}
			} else if (bodyA->getRwBody() == ftsensor->getBody2() && bodyB->getRwBody() == ftsensor->getBody1()) {
				for (int i = 0; i < manifold->getNumContacts(); i++) {
					const btManifoldPoint& point = manifold->getContactPoint(i);
					const btScalar force = point.getAppliedImpulse()/info.dt;
					const Vector3D<> pos = BtUtil::toVector3D(point.getPositionWorldOnA());
					const Vector3D<> n = -BtUtil::toVector3D(point.m_normalWorldOnB);
					_rwSensor->addForceW(pos,force*n,n,state,bodyA->getRwBody());
				}
			}
		} else {
			// Add contacts
			if (bodyA->getRwBody()->getBodyFrame() == tsensor->getFrame()) {
				for (int i = 0; i < manifold->getNumContacts(); i++) {
					const btManifoldPoint& point = manifold->getContactPoint(i);
					const btScalar force = point.getAppliedImpulse()/info.dt;
					const Vector3D<> pos = BtUtil::toVector3D(point.getPositionWorldOnA());
					const Vector3D<> n = -BtUtil::toVector3D(point.m_normalWorldOnB);
					_rwSensor->addForceW(pos,force*n,n,state,bodyA->getRwBody());
				}
			} else if (bodyB->getRwBody()->getBodyFrame() == tsensor->getFrame()) {
				for (int i = 0; i < manifold->getNumContacts(); i++) {
					const btManifoldPoint& point = manifold->getContactPoint(i);
					const btScalar force = point.getAppliedImpulse()/info.dt;
					const Vector3D<> pos = BtUtil::toVector3D(point.getPositionWorldOnB());
					const Vector3D<> n = BtUtil::toVector3D(point.m_normalWorldOnB);
					_rwSensor->addForceW(pos,force*n,n,state,bodyB->getRwBody());
				}
			}
		}
		//_rwSensor->update(info, state);
	}
}

void BtTactileSensor::addFeedback(BtConstraint* const constraint) {
	_constraints.push_back(constraint);
	constraint->getFeedback();
}
