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

#include <rw/kinematics/FramePairMap.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::tntphysics;

const void* TNTUtil::MARK_NEW = getNewMark();

const void* TNTUtil::MARK_RAW = NULL;

std::vector<std::size_t> TNTUtil::getMarkedContacts(const std::vector<Contact>& input, const ContactDetectorTracking& tracking, const void* mark) {
	std::vector<std::size_t> res;
	for (std::size_t i = 0; i < input.size(); i++) {
		if (tracking.getUserData(i) == mark)
			res.push_back(i);
	}
	return res;
}
/*
std::vector<std::size_t> TNTUtil::getKnownContacts(const std::vector<Contact> &input, const ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate) {
	std::vector<std::size_t> res;
	const TNTBodyConstraintManager::ConstraintList constraints = bc->getTemporaryConstraints(&tntstate);
	for (std::size_t i = 0; i < input.size(); i++) {
		const void* const data = tracking.getUserData(i);
		BOOST_FOREACH(TNTConstraint* constraint, constraints) {
			if (constraint == data) {
				res.push_back(i);
				break;
			}
		}
	}
	return res;
}*/

std::vector<Contact> TNTUtil::getContacts(const std::vector<rwsim::contacts::Contact>& contacts, const std::vector<std::size_t>& ids) {
	std::vector<Contact> res(ids.size());
	for (std::size_t k = 0; k < ids.size(); k++) {
		res[k] = contacts[ids[k]];
	}
	return res;
}

void TNTUtil::removeNonPenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const void* mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) != mark) {
			it++;
			continue;
		}
		if ((*it).getDepth() < 0) {
			it = contacts.erase(it);
			it--;
			tracking.remove(i);
		} else {
			i++;
		}
	}
}

void TNTUtil::removeKnown(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate) {
	RW_ASSERT(contacts.size() == tracking.getInfo().size());
	const TNTBodyConstraintManager::ConstraintList constraints = bc->getTemporaryConstraints(&tntstate);
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		const void* const data = tracking.getUserData(i);
		if (data == MARK_RAW || data == MARK_NEW) {
			i++;
			continue;
		}
		bool found = false;
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			if (constraint == data) {
				it = contacts.erase(it);
				it--;
				tracking.remove(i);
				found = true;
				break;
			}
		}
		if (!found)
			i++;
	}
}

void TNTUtil::remove(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const void* mark) {
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

void TNTUtil::mark(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const void* oldMark, const void* newMark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) == oldMark) {
			tracking.setUserData(i,newMark);
		}
		i++;
	}
}
/*
void TNTUtil::keepOnlyMostPenetrating(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const void* mark) {
	std::vector<bool> remove(contacts.size(),false);
	{
		FramePairMap<std::vector<std::size_t> > frameToContacts;
		std::vector<Contact>::const_iterator it;
		std::size_t i = 0;
		for (it = contacts.begin(); it != contacts.end(); it++) {
			const Contact& c = *it;
			if (tracking.getUserData(i) == mark)
				frameToContacts(c.getFrameA(),c.getFrameB()).push_back(i);
			i++;
		}
		for (it = contacts.begin(); it != contacts.end(); it++) {
			const Contact& c = *it;
			if (frameToContacts.has(c.getFrameA(),c.getFrameB())) {
				const std::vector<std::size_t>& contactIDs = frameToContacts(c.getFrameA(),c.getFrameB());
				std::size_t bestID = contactIDs[0];
				double depth = contacts[contactIDs[0]].getDepth();
				for (std::size_t i = 1; i < contactIDs.size(); i++) {
					const double curDepth = contacts[contactIDs[i]].getDepth();
					if (curDepth > depth) {
						bestID = contactIDs[i];
						depth = curDepth;
					}
				}
				for (std::size_t i = 0; i < contactIDs.size(); i++) {
					if (contactIDs[i] != bestID) {
						remove[contactIDs[i]] = true;
					}
				}
				frameToContacts.erase(c.getFrameA(),c.getFrameB());
			}
		}
	}
	{
		std::size_t i = 0;
		std::vector<Contact>::iterator it;
		std::size_t removed = 0;
		for (it = contacts.begin(); it != contacts.end(); it++) {
			if (remove[i]) {
				tracking.remove(i-removed);
				removed++;
				it = contacts.erase(it);
				it--;
			}
			i++;
		}
	}
}
*/
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

/*std::vector<Contact> TNTUtil::contactsWithinThreshold(const std::vector<Contact>& contacts, ContactDetectorTracking& tracking, double threshold) {
	std::vector<Contact> res;
	BOOST_FOREACH(const Contact&c, contacts) {
		if (fabs(c.getDepth()) < threshold)
			res.push_back(c);
	}
	return res;
}*/

void TNTUtil::removeContactsOutsideThreshold(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, double threshold, const void* mark) {
	std::vector<Contact>::iterator it;
	std::size_t i = 0;
	for (it = contacts.begin(); it != contacts.end(); it++) {
		if (tracking.getUserData(i) != mark) {
			i++;
			continue;
		}
		const Contact& c = *it;
		if (fabs(c.getDepth()) > threshold) {
			it = contacts.erase(it);
			it--;
			tracking.remove(i);
		} else {
			i++;
		}
	}
}

/*std::set<std::pair<const Frame*, const Frame*> > TNTUtil::getFramePairs(const std::vector<Contact>& contacts) {
	std::set<std::pair<const Frame*, const Frame*> > pairs;
	BOOST_FOREACH(const Contact &c, contacts) {
		if (c.getFrameA() < c.getFrameB())
			pairs.insert(std::make_pair<const Frame*, const Frame*>(c.getFrameA(),c.getFrameB()));
		else
			pairs.insert(std::make_pair<const Frame*, const Frame*>(c.getFrameB(),c.getFrameA()));
	}
	return pairs;
}*/

void TNTUtil::updateTemporaryContacts(const std::vector<Contact>& contacts, const ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, TNTIslandState& tntstate, const State &rwstate) {
	std::size_t i = 0;
	TNTBodyConstraintManager::ConstraintList constraints = bc->getTemporaryConstraints(&tntstate);
	BOOST_FOREACH(const Contact &c, contacts) {
		const void* const data = tracking.getUserData(i);
		const TNTContact* const tntcontact = (const TNTContact*) data;
		TNTContact* contact = NULL;
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
/*
std::vector<std::size_t> TNTUtil::getLeavingContacts(
		const std::vector<Contact>& contacts, const ContactDetectorTracking& tracking,
		const TNTBodyConstraintManager* bc,
		const TNTIslandState& tntstate, const State &rwstate,
		const void* mark) {
	std::vector<std::size_t> res;
	std::size_t i = 0;
	BOOST_FOREACH(const Contact &c, contacts) {
		if (tracking.getUserData(i) != mark)
			continue;
		const TNTBody* const bodyA = bc->getBody(c.getFrameA());
		const TNTBody* const bodyB = bc->getBody(c.getFrameB());
		if (bodyA == NULL)
			RW_THROW("TNTUtil (getLeavingContacts): Could not find a TNTBody for frame \"" << c.getFrameA()->getName() << "\".");
		if (bodyB == NULL)
			RW_THROW("TNTUtil (getLeavingContacts): Could not find a TNTBody for frame \"" << c.getFrameB()->getName() << "\".");
		const TNTContact* const tntcontact = new TNTContact(bodyA,bodyB,c,rwstate);
		const VelocityScrew6D<> velParent = tntcontact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<> velChild = tntcontact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> relVel = velParent.linear()-velChild.linear();
		if (dot(relVel,c.getNormal()) < 0)
			res.push_back(i);
		delete tntcontact;
		i++;
	}
	return res;
}
*/
/*
void TNTUtil::keepOnlyNonLeavingContacts(std::vector<Contact>& contacts, ContactDetectorTracking& tracking, const TNTBodyConstraintManager* bc, const TNTIslandState& tntstate, const State &rwstate) {
	std::vector<Contact> res;
	BOOST_FOREACH(const Contact &c, contacts) {
		const TNTBody* const bodyA = bc->getBody(c.getFrameA());
		const TNTBody* const bodyB = bc->getBody(c.getFrameB());
		if (bodyA == NULL)
			RW_THROW("TNTUtil (getNonLeavingContacts): Could not find a TNTBody for frame \"" << c.getFrameA()->getName() << "\".");
		if (bodyB == NULL)
			RW_THROW("TNTUtil (getNonLeavingContacts): Could not find a TNTBody for frame \"" << c.getFrameB()->getName() << "\".");
		const TNTContact* const tntcontact = new TNTContact(bodyA,bodyB,c,rwstate);
		const VelocityScrew6D<> velParent = tntcontact->getVelocityParentW(tntstate,rwstate);
		const VelocityScrew6D<> velChild = tntcontact->getVelocityChildW(tntstate,rwstate);
		const Vector3D<> relVel = velParent.linear()-velChild.linear();
		if (dot(relVel,c.getNormal()) >= 0)
			res.push_back(c);
		delete tntcontact;
	}
	return res;
}
*/

const void* TNTUtil::getNewMark() {
	static const DummyMark MARKOBJECT;
	static const void* MARK = &MARKOBJECT;
	return MARK;
}
