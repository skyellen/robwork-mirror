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

#include <rw/kinematics/FramePairMap.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyKinematic.hpp"
#include "RWPEConstraintGeneric.hpp"
#include "RWPEIntegrator.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"
#include "RWPEUtil.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsimlibs::rwpe;

const ContactStrategyTracking::UserData::Ptr RWPEUtil::MARK_NEW = getNewMark();

const ContactStrategyTracking::UserData::Ptr RWPEUtil::MARK_RAW = NULL;

std::vector<std::size_t> RWPEUtil::getMarkedContacts(const std::vector<Contact>& input, const ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
	std::vector<std::size_t> res;
	for (std::size_t i = 0; i < input.size(); i++) {
		if (tracking.getUserData(i) == mark)
			res.push_back(i);
	}
	return res;
}

std::vector<Contact> RWPEUtil::getContacts(const std::vector<Contact>& contacts, const std::vector<std::size_t>& ids) {
	std::vector<Contact> res(ids.size());
	for (std::size_t k = 0; k < ids.size(); k++) {
		res[k] = contacts[ids[k]];
	}
	return res;
}

void RWPEUtil::removeNonPenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
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

void RWPEUtil::removePenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
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

void RWPEUtil::removeKnown(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState) {
	RW_ASSERT(contacts.size() == tracking.getInfo().size());
	const RWPEBodyConstraintGraph::ConstraintList constraints = bc->getTemporaryConstraints(&islandState);
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		const ContactStrategyTracking::UserData::Ptr data = tracking.getUserData(i);
		if (data == MARK_RAW || data == MARK_NEW) {
			i++;
			continue;
		}
		bool found = false;
		BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
			const RWPEUtil::RWPEUserData* const rwpeData = dynamic_cast<const RWPEUtil::RWPEUserData*>(data.get());
			if (rwpeData) {
				if (constraint == rwpeData->contact) {
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

void RWPEUtil::remove(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr mark) {
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

void RWPEUtil::mark(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, ContactStrategyTracking::UserData::Ptr oldMark, ContactStrategyTracking::UserData::Ptr newMark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == oldMark) {
			tracking.setUserData(i,newMark);
		}
		i++;
	}
}

void RWPEUtil::step(double dt, const Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, State& rwstate) {
	const RWPEBodyConstraintGraph::DynamicBodyList rbodies = bc->getDynamicBodies();
	BOOST_FOREACH(const RWPEBodyDynamic* rbody, rbodies) {
		RWPEBodyDynamic::RigidConfiguration* config = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(islandState.getConfiguration(rbody));
		const RWPEIntegrator* integrator = rbody->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		integrator->integrate(bc->getConstraints(rbody, islandState),gravity,dt,*config,islandState,rwstate);
	}
	const RWPEBodyConstraintGraph::KinematicBodyList kbodies = bc->getKinematicBodies();
	BOOST_FOREACH(const RWPEBodyKinematic* kbody, kbodies) {
		kbody->integrate(dt,islandState,rwstate);
	}
	const RWPEBodyConstraintGraph::BodyList bodies = bc->getBodies();
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		body->updateRW(rwstate,islandState);
	}
}

void RWPEUtil::positionUpdate(double dt, const Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, State& rwstate) {
	const RWPEBodyConstraintGraph::DynamicBodyList rbodies = bc->getDynamicBodies();
	BOOST_FOREACH(const RWPEBodyDynamic* rbody, rbodies) {
		RWPEBodyDynamic::RigidConfiguration* const config = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(islandState.getConfiguration(rbody));
		const RWPEBodyConstraintGraph::ConstraintListConst constraints = bc->getConstraints(rbody, islandState);
		const RWPEIntegrator* integrator = rbody->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		integrator->positionUpdate(constraints,gravity,dt,*config,islandState,rwstate);
	}
	const RWPEBodyConstraintGraph::KinematicBodyList kbodies = bc->getKinematicBodies();
	BOOST_FOREACH(const RWPEBodyKinematic* kbody, kbodies) {
		kbody->integrate(dt,islandState,rwstate);
	}
	const RWPEBodyConstraintGraph::BodyList bodies = bc->getBodies();
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		body->updateRW(rwstate,islandState);
	}
}

void RWPEUtil::velocityUpdate(double dt, const Vector3D<>& gravity, bool discontinuity, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState0, RWPEIslandState& islandStateH, const State& rwstate, RWPELogUtil* log) {
	const RWPEBodyConstraintGraph::DynamicBodyList rbodies = bc->getDynamicBodies();
	BOOST_FOREACH(const RWPEBodyDynamic* rbody, rbodies) {
		log->beginSection(rbody->get()->getName(),RWPE_LOCATION);
		const RWPEBodyDynamic::RigidConfiguration* const config0 = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(islandState0.getConfiguration(rbody));
		RWPEBodyDynamic::RigidConfiguration* const configH = dynamic_cast<RWPEBodyDynamic::RigidConfiguration*>(islandStateH.getConfiguration(rbody));
		const RWPEIntegrator* integrator = rbody->getIntegrator();
		if (discontinuity)
			integrator = integrator->getDiscontinuityIntegrator();
		integrator->velocityUpdate(bc->getConstraints(rbody, islandState0),gravity,dt,*config0,*configH,islandState0,islandStateH,rwstate,*log);
		log->endSection(__LINE__);
	}
}

double RWPEUtil::minDistance(const std::vector<Contact>& contacts) {
	if (contacts.size() == 0)
		RW_THROW("RWPEUtil (minDistance): no contacts given!");
	double minDist = -contacts[0].getDepth();
	BOOST_FOREACH(const Contact &c, contacts) {
		if (-c.getDepth() < minDist)
			minDist = -c.getDepth();
	}
	return minDist;
}

double RWPEUtil::maxDistance(const std::vector<Contact>& contacts) {
	if (contacts.size() == 0)
		RW_THROW("RWPEUtil (maxDistance): no contacts given!");
	double maxDist = -contacts[0].getDepth();
	BOOST_FOREACH(const Contact &c, contacts) {
		if (-c.getDepth() > maxDist)
			maxDist = -c.getDepth();
	}
	return maxDist;
}

void RWPEUtil::removeContactsOutsideThreshold(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, double threshold, ContactStrategyTracking::UserData::Ptr mark) {
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

void RWPEUtil::updateTemporaryContacts(const std::vector<Contact>& contacts, const ContactDetectorTracking& tracking, const RWPEBodyConstraintGraph* bc, RWPEIslandState& islandState, const State &rwstate) {
	std::size_t i = 0;
	RWPEBodyConstraintGraph::ConstraintList constraints = bc->getTemporaryConstraints(&islandState);
	BOOST_FOREACH(const Contact &c, contacts) {
		const ContactStrategyTracking::UserData::Ptr data = tracking.getUserData(i);
		RWPEContact* contact = NULL;
		if (const RWPEUtil::RWPEUserData* rwpeData = dynamic_cast<const RWPEUtil::RWPEUserData*>(data.get())) {
			const RWPEContact* const rwpecontact = rwpeData->contact;
			RWPEBodyConstraintGraph::ConstraintList::iterator it;
			for (it = constraints.begin(); it != constraints.end(); it++) {
				RWPEConstraint* constraint = *it;
				if (constraint == rwpecontact) {
					if (RWPEContact* const contactCast = dynamic_cast<RWPEContact*>(constraint))
						contact = contactCast;
					constraints.erase(it);
					break;
				}
			}
		}
		if (contact != NULL) {
			contact->setContact(c,rwstate);
		} else {
			//RW_THROW("RWPEUtil (updateTemporaryContacts): new RWPEContact should be created here (implementation missing).");
		}
		i++;
	}
	// Remove the RWPEContacts that was not found in updated set of contacts
	BOOST_FOREACH(RWPEConstraint* const constraint, constraints) {
		islandState.removeTemporaryConstraint(constraint);
	}
}

void RWPEUtil::updateSensors(const std::list<SimulatedSensor::Ptr>& sensors, double time, double dt, double dt_prev, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& islandState0, const RWPEIslandState& islandStateH, State& rwstate, RWPELogUtil& log) {
	const bool doLog = log.doLog();
	Simulator::UpdateInfo info;
	info.dt = dt;
	info.dt_prev = dt_prev;
	info.time = time;
	info.rollback = false;

	BOOST_FOREACH(const SimulatedSensor::Ptr sensor, sensors) {
		log.beginSection(sensor->getName(),RWPE_LOCATION);
		SimulatedSensor* const ssensor = sensor.get();
		ssensor->reset(rwstate);
		if(SimulatedTactileSensor* const tsensor = dynamic_cast<SimulatedTactileSensor*>(ssensor) ){
			RWPEBodyConstraintGraph::ConstraintListConst constraints;
			const RWPEBody* rwpebody;
			if(SimulatedFTSensor* const ftsensor = dynamic_cast<SimulatedFTSensor*>(tsensor) ){
				const Body::Ptr b1 = ftsensor->getBody1();
				const Body::Ptr b2 = ftsensor->getBody2();
				RW_ASSERT(b1 != NULL);
				RW_ASSERT(b2 != NULL);
				const RWPEBody* const rwpebody1 = bc->getBody(b1->getBodyFrame());
				const RWPEBody* const rwpebody2 = bc->getBody(b2->getBodyFrame());
				RW_ASSERT(rwpebody1 != NULL);
				RW_ASSERT(rwpebody2 != NULL);
				constraints = bc->getConstraints(rwpebody1,rwpebody2,islandStateH);
				rwpebody = rwpebody2;
				if (doLog) {
					std::stringstream name;
					name << "Bodies: " << rwpebody1->get()->getName() << " and " << rwpebody1->get()->getName();
					log.log(name.str(),RWPE_LOCATION);
				}
			} else {
				const Frame* const bframe = tsensor->getFrame();
				const RWPEBody* const body = bc->getBody(bframe);
				constraints = bc->getConstraints(body,islandStateH);
				rwpebody = body;
				if (doLog) {
					std::stringstream name;
					name << "Body: " << rwpebody->get()->getName();
					log.log(name.str(),RWPE_LOCATION);
				}
			}
			BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
				const Wrench6D<> wrench0 = constraint->getWrench(islandState0);
				const Wrench6D<> wrenchH = constraint->getWrench(islandStateH);
				if(const RWPERWConstraint* const rwconstraint = dynamic_cast<const RWPERWConstraint*>(constraint) ) {
					if (dynamic_cast<BodyContactSensor*>(tsensor))
						continue;
					if (doLog) {
						std::stringstream name;
						name << "Adding RW constraint: " << rwconstraint->getConstraint()->getName();
						log.log(name.str(),RWPE_LOCATION) << "Wrench 0: " << wrench0 << "Wrench H: " << wrenchH;
					}
					if (constraint->getParent() == rwpebody) {
						const Vector3D<> pos = constraint->getPositionParentW(islandStateH);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),0.5*(wrench0.torque()+wrenchH.torque()),rwstate,rwpebody->get());
						tsensor->addForceW(pos,0.5*(wrench0.force()+wrenchH.force()),Vector3D<>::zero(),rwstate,rwpebody->get());
					} else {
						const Vector3D<> pos = constraint->getPositionChildW(islandStateH);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),-0.5*(wrench0.torque()+wrenchH.torque()),rwstate,rwpebody->get());
						tsensor->addForceW(pos,-0.5*(wrench0.force()+wrenchH.force()),Vector3D<>::zero(),rwstate,rwpebody->get());
					}
				} else if(const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint) ) {
					if (doLog) {
						log.log("Adding contact",RWPE_LOCATION) << "Wrench 0: " << wrench0 << "Wrench H: " << wrenchH;
					}
					const Vector3D<> n = contact->getNormalW(islandStateH);
					if (constraint->getParent() == rwpebody) {
						const Vector3D<> pos = constraint->getPositionParentW(islandStateH);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),0.5*(wrench0.torque()+wrenchH.torque()),rwstate,rwpebody->get());
						tsensor->addForceW(pos,0.5*(wrench0.force()+wrenchH.force()),n,rwstate,rwpebody->get());
					} else {
						const Vector3D<> pos = constraint->getPositionChildW(islandStateH);
						tsensor->addWrenchWToCOM(Vector3D<>::zero(),-0.5*(wrench0.torque()+wrenchH.torque()),rwstate,rwpebody->get());
						tsensor->addForceW(pos,-0.5*(wrench0.force()+wrenchH.force()),-n,rwstate,rwpebody->get());
					}
				} else {
					RW_THROW("RWPEUtil (updateSensors): unknown type of RWPEConstraint encountered!");
				}
			}
		}
		ssensor->update(info,rwstate);
		log.endSection(__LINE__);
	}
}

ContactStrategyTracking::UserData::Ptr RWPEUtil::getNewMark() {
	static const ContactStrategyTracking::UserData::Ptr MARK = ownedPtr(new DummyMark());
	return MARK;
}
