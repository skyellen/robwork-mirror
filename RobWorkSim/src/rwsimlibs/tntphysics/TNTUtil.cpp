/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTUtil.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTIslandState.hpp"

#include "TNTKinematicBody.hpp"
#include "TNTRigidBody.hpp"
#include "TNTIntegrator.hpp"
#include "TNTContact.hpp"
#include "TNTRWConstraint.hpp"

#include <rw/kinematics/FramePairMap.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsimlibs::tntphysics;

const ContactStrategyTracking::UserData::Ptr TNTUtil::MARK_NEW = getNewMark();

const ContactStrategyTracking::UserData::Ptr TNTUtil::MARK_RAW = NULL;

std::vector<std::size_t> TNTUtil::getMarkedContacts(const std::vector<Contact>& input, const ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<std::size_t> res;
	for (std::size_t i = 0; i < input.size(); i++) {
		if (tracking.getUserData(i) == mark)
			res.push_back(i);
	}
	return res;
}

std::vector<Contact> TNTUtil::getContacts(const std::vector<Contact>& contacts, const std::vector<std::size_t>& ids) {
	std::vector<Contact> res(ids.size());
	for (std::size_t k = 0; k < ids.size(); k++) {
		res[k] = contacts[ids[k]];
	}
	return res;
}

void TNTUtil::removeNonPenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == mark) {
			if ((*it).getDepth() < 0) {
				it = contacts.erase(it);
				it--;
				tracking.remove(i);
				i--;
			}
		}
		i++;
	}
}

void TNTUtil::removePenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == mark) {
			if ((*it).getDepth() > 0) {
				it = contacts.erase(it);
				it--;
				tracking.remove(i);
				i--;
			}
		}
		i++;
	}
}

void TNTUtil::removeKnown(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate) {
	RW_ASSERT(contacts.size() == tracking.getInfo().size());
	const TNTBodyConstraintManager::ConstraintList constraints = bc->getTemporaryConstraints(&tntstate);
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		const ContactStrategyTracking::UserData::Ptr data = tracking.getUserData(i);
		if (data == MARK_RAW || data == MARK_NEW) {
			i++;
			continue;
		}
		bool found = false;
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			const TNTUtil::TNTUserData* const tntData = dynamic_cast<const TNTUtil::TNTUserData*>(data.get());
			if (tntData) {
				if (constraint == tntData->contact) {
					it = contacts.erase(it);
					it--;
					tracking.remove(i);
					found = true;
					break;
				}
			}
		}
		if (!found)
			i++;
	}
}

void TNTUtil::remove(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == mark) {
			it = contacts.erase(it);
			it--;
			tracking.remove(i);
		} else {
			i++;
		}
	}
}

void TNTUtil::mark(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr oldMark, ContactStrategyTracking::UserData::Ptr newMark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == oldMark) {
			tracking.setUserData(i,newMark);
		}
		i++;
	}
}

void TNTUtil::step(double dt, const Vector3D<>& gravity, const TNTBodyConstraintManager* bc, TNTIslandState& tntstate, State& rwstate) {
	const TNTBodyConstraintManager::DynamicBodyList rbodies = bc->getDynamicBodies();
	BOOST_FOREACH(const TNTRigidBody* rbody, rbodies) {
		TNTRigidBody::RigidConfiguration* config = dynamic_cast<TNTRigidBody::RigidConfiguration*>(tntstate.getConfiguration(rbody));
		rbody->getIntegrator()->integrate(bc->getConstraints(rbody, tntstate),gravity,dt,*config,tntstate,rwstate);
	}
	const TNTBodyConstraintManager::KinematicBodyList kbodies = bc->getKinematicBodies();
	BOOST_FOREACH(const TNTKinematicBody* kbody, kbodies) {
		kbody->integrate(dt,tntstate,rwstate);
	}
	const TNTBodyConstraintManager::BodyList bodies = bc->getBodies();
	BOOST_FOREACH(const TNTBody* body, bodies) {
		body->updateRW(rwstate,tntstate);
	}
}

double TNTUtil::minDistance(const std::vector<Contact>& contacts) {
	if (contacts.size() == 0)
		RW_THROW("TNTUtil (minDistance): no contacts given!");
	double minDist = -contacts[0].getDepth();
	BOOST_FOREACH(const Contact &c, contacts) {
		if (-c.getDepth() < minDist)
			minDist = -c.getDepth();
	}
	return minDist;
}

double TNTUtil::maxDistance(const std::vector<Contact>& contacts) {
	if (contacts.size() == 0)
		RW_THROW("TNTUtil (maxDistance): no contacts given!");
	double maxDist = -contacts[0].getDepth();
	BOOST_FOREACH(const Contact &c, contacts) {
		if (-c.getDepth() > maxDist)
			maxDist = -c.getDepth();
	}
	return maxDist;
}

void TNTUtil::removeContactsOutsideThreshold(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, double threshold, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (!(tracking.getUserData(i) == mark)) {
			i++;
			continue;
		}
		const Contact& c = *it;
		if (-c.getDepth() > threshold) {
			it = contacts.erase(it);
			it--;
			tracking.remove(i);
		} else {
			i++;
		}
	}
}

void TNTUtil::updateTemporaryContacts(const std::vector<Contact>& contacts, const ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, TNTIslandState& tntstate, const State &rwstate) {
	std::size_t i = 0;
	TNTBodyConstraintManager::ConstraintList constraints = bc->getTemporaryConstraints(&tntstate);
	BOOST_FOREACH(const Contact &c, contacts) {
		const ContactStrategyTracking::UserData::Ptr data = tracking.getUserData(i);
		TNTContact* contact = NULL;
		if (const TNTUtil::TNTUserData* tntData = dynamic_cast<const TNTUtil::TNTUserData*>(data.get())) {
			const TNTContact* const tntcontact = tntData->contact;
			TNTBodyConstraintManager::ConstraintList::iterator it;
			for (it = constraints.begin(); it != constraints.end(); it++) {
				TNTConstraint* constraint = *it;
				if (constraint == tntcontact) {
					if (TNTContact* const contactCast = dynamic_cast<TNTContact*>(constraint))
						contact = contactCast;
					constraints.erase(it);
					break;
				}
			}
		}
		if (contact != NULL) {
			contact->setContact(c,rwstate);
		} else {
			//RW_THROW("TNTUtil (updateTemporaryContacts): new TNTContact should be created here (implementation missing).");
		}
		i++;
	}
	// Remove the TNTContacts that was not found in updated set of contacts
	BOOST_FOREACH(TNTConstraint* const constraint, constraints) {
		tntstate.removeTemporaryConstraint(constraint);
	}
}

void TNTUtil::updateSensors(const std::list<SimulatedSensor::Ptr>& sensors, double time, double dt, double dt_prev, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate, State& rwstate) {
	Simulator::UpdateInfo info;
	info.dt = dt;
	info.dt_prev = dt_prev;
	info.time = time;
	info.rollback = false;

	BOOST_FOREACH(const SimulatedSensor::Ptr sensor, sensors) {
		SimulatedSensor* const ssensor = sensor.get();
		ssensor->reset(rwstate);
		if(SimulatedTactileSensor* const tsensor = dynamic_cast<SimulatedTactileSensor*>(ssensor) ){
			TNTBodyConstraintManager::ConstraintListConst constraints;
			const TNTBody* tntbody;
			if(SimulatedFTSensor* const ftsensor = dynamic_cast<SimulatedFTSensor*>(tsensor) ){
				const Body::Ptr b1 = ftsensor->getBody1();
				const Body::Ptr b2 = ftsensor->getBody2();
				RW_ASSERT(b1 != NULL);
				RW_ASSERT(b2 != NULL);
				const TNTBody* const tntbody1 = bc->getBody(b1->getBodyFrame());
				const TNTBody* const tntbody2 = bc->getBody(b2->getBodyFrame());
				RW_ASSERT(tntbody1 != NULL);
				RW_ASSERT(tntbody2 != NULL);
				constraints = bc->getConstraints(tntbody1,tntbody2,tntstate);
				tntbody = tntbody2;
			} else {
				const Frame* const bframe = tsensor->getFrame();
				const TNTBody* const body = bc->getBody(bframe);
				constraints = bc->getConstraints(body,tntstate);
				tntbody = body;
			}
			BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
				const Wrench6D<> wrench = constraint->getWrench(tntstate);
				if(const TNTRWConstraint* const rwconstraint = dynamic_cast<const TNTRWConstraint*>(constraint) ){
					if (constraint->getParent() == tntbody) {
						const Vector3D<> pos = constraint->getPositionParentW(tntstate);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),wrench.torque(),rwstate,tntbody->get());
						tsensor->addForceW(pos,wrench.force(),Vector3D<>::zero(),rwstate,tntbody->get());
					} else {
						const Vector3D<> pos = constraint->getPositionChildW(tntstate);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),-wrench.torque(),rwstate,tntbody->get());
						tsensor->addForceW(pos,-wrench.force(),Vector3D<>::zero(),rwstate,tntbody->get());
					}
				} else if(const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint) ){
					const Vector3D<> n = contact->getNormalW(tntstate);
					if (constraint->getParent() == tntbody) {
						const Vector3D<> pos = constraint->getPositionParentW(tntstate);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),wrench.torque(),rwstate,tntbody->get());
						tsensor->addForceW(pos,wrench.force(),n,rwstate,tntbody->get());
					} else {
						const Vector3D<> pos = constraint->getPositionChildW(tntstate);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),-wrench.torque(),rwstate,tntbody->get());
						tsensor->addForceW(pos,-wrench.force(),-n,rwstate,tntbody->get());
					}
				} else {
					RW_THROW("TNTUtil (updateSensors): unknown type of TNTConstraint encountered!");
				}
			}
		}
		ssensor->update(info,rwstate);
	}
}

ContactStrategyTracking::UserData::Ptr TNTUtil::getNewMark() {
	static const ContactStrategyTracking::UserData::Ptr MARK = ownedPtr(new DummyMark());
	return MARK;
}
