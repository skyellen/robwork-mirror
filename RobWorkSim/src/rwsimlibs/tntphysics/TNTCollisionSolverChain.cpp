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

#include "TNTCollisionSolverChain.hpp"
#include "TNTCollisionSolverSingle.hpp"
#include "TNTRigidBody.hpp"
#include "TNTSettings.hpp"
#include "TNTContact.hpp"
#include "TNTBodyConstraintManager.hpp"

#ifdef TNT_DEBUG_ENABLE_BOUNCING
#include <rwsim/dynamics/Body.hpp>
#endif

#include <stack>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

TNTCollisionSolverChain::TNTCollisionSolverChain():
	_solver(new TNTCollisionSolverSingle())
{
	_solver->setCheckNoChain(false);
}

TNTCollisionSolverChain::~TNTCollisionSolverChain() {
	delete _solver;
}

void TNTCollisionSolverChain::applyImpulses(
		const std::vector<const TNTContact*>& contacts,
		const TNTBodyConstraintManager& bc,
		const TNTMaterialMap* const map,
		TNTIslandState& tntstate,
		const State& rwstate) const
{
	// First connected rigid body components are found, with a mapping of the contacts to components.
	std::vector<std::vector<std::size_t> > componentToContacts;
	std::vector<std::size_t> contactToComponent = std::vector<std::size_t>(contacts.size(),0);
	std::vector<std::set<const TNTRigidBody*> > componentToBodies;

	for (std::size_t contactID = 0; contactID < contacts.size(); contactID++) {
		const TNTContact* const contact = contacts[contactID];
		std::set<const TNTRigidBody*> potentialChain;

		std::set<const TNTRigidBody*> bodiesToCheck;
		{
			const TNTBody* const bodyA = contact->getParent();
			const TNTBody* const bodyB = contact->getChild();
			const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
			const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
			if (rBodyA)
				bodiesToCheck.insert(rBodyA);
			if (rBodyB)
				bodiesToCheck.insert(rBodyB);
		}
		while (!bodiesToCheck.empty()) {
			const TNTRigidBody* const body = *bodiesToCheck.begin();
			bodiesToCheck.erase(bodiesToCheck.begin());
			{
				// Skip if body is in chain already
				bool found = false;
				BOOST_FOREACH(const TNTRigidBody* const rbody, potentialChain) {
					if (rbody == body)
						found = true;
				}
				if (found)
					continue;
			}
			potentialChain.insert(body);

			// Check if body is in existing component
			{
				bool found = false;
				std::size_t compId;
				for (compId = 0; !found && compId < componentToBodies.size(); compId++) {
					BOOST_FOREACH(const TNTRigidBody* const b, componentToBodies[compId]) {
						if (b == body) {
							found = true;
							break;
						}
					}
				}
				if (found) {
					compId--;
					contactToComponent[contactID] = compId;
					componentToContacts[compId].push_back(contactID);
					bodiesToCheck.clear();
					potentialChain.clear();
					break;
				}
			}

			// If there are more bodies in component these are added now
			const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(body,tntstate);
			BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
				if (constraint->getDimVelocity() == 0)
					continue;
				const TNTBody* const bodyA = constraint->getParent();
				const TNTBody* const bodyB = constraint->getChild();
				const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
				const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
				if (rBodyA && body != bodyA)
					bodiesToCheck.insert(rBodyA);
				if (rBodyB && body != bodyB)
					bodiesToCheck.insert(rBodyB);
			}
		}

		// If complete component has been checked and no matches were found with existing components,
		// a new component is created.
		if (potentialChain.size() > 0) {
			componentToBodies.push_back(potentialChain);
			contactToComponent[contactID] = componentToContacts.size();
			componentToContacts.push_back(std::vector<std::size_t>(1,contactID));
		}
	}
	if (componentToBodies.size() > 0) {
		TNT_DEBUG_BOUNCING("Found " << componentToBodies.size() << " component(s) for the " << contacts.size() << " bouncing contact(s).");
	} else {
		TNT_DEBUG_BOUNCING("Found no component for the " << contacts.size() << " bouncing contact(s).");
		return;
	}

	// Chains are now constructed from the components (if they are not chains an exception is thrown)
	std::list<std::list<const TNTBody*> > chains;
	BOOST_FOREACH(const std::set<const TNTRigidBody*>& bodies, componentToBodies) {
		std::list<const TNTBody*> chain;
		std::set<const TNTRigidBody*> handled;

		// Now one body is inserted in the chain at a time
		std::stack<const TNTRigidBody*> queue;
		queue.push(*bodies.begin());
		while (!queue.empty()) {
			const TNTRigidBody* const current = queue.top();
			queue.pop();

			// If the body has already been handled skip it
			if (handled.find(current) != handled.end())
				continue;
			handled.insert(current);

			// The set of connected bodies are found
			const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(current,tntstate);
			std::set<const TNTBody*> other;
			BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
				if (constraint->getDimVelocity() == 0)
					continue;
				const TNTBody* const bodyA = constraint->getParent();
				const TNTBody* const bodyB = constraint->getChild();
				RW_ASSERT(bodyA != NULL);
				RW_ASSERT(bodyB != NULL);
				if (current == bodyA)
					other.insert(bodyB);
				else
					other.insert(bodyA);
			}

			// We extract the two connected bodies or throw an error if there are more
			const TNTBody* bodyA = NULL;
			const TNTBody* bodyB = NULL;
			const TNTRigidBody* rBodyA = NULL;
			const TNTRigidBody* rBodyB = NULL;
			{
				std::set<const TNTBody*>::iterator it = other.begin();
				if (other.size() >= 1) {
					bodyA = *it;
					rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
				}
				if (other.size() >= 2) {
					it++;
					bodyB = *it;
					rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
				}
				if (other.size() > 2) {
					RW_THROW("TNTCollisionSolverChain (applyImpulses): Can not solve for bodies not connected in chain.");
				}
			}

			// If bodies are rigid, add them to the queue to be checked for neighbours later
			if (rBodyA)
				queue.push(rBodyA);
			if (rBodyB)
				queue.push(rBodyB);
			RW_ASSERT(bodyB == NULL || (bodyB != NULL && bodyA != NULL));

			// If the this is the first body, insert this body directly.
			if (chain.size() == 0) {
				chain.push_front(current);
			}

			// Search for this body and the neighbours in the chain
			std::list<const TNTBody*>::iterator itThis;
			std::list<const TNTBody*>::reverse_iterator itThisRev;
			for (itThis = chain.begin(); itThis != chain.end(); itThis++) {
				if (*itThis == current)
					break;
			}
			for (itThisRev = chain.rbegin(); itThisRev != chain.rend(); itThisRev++) {
				if (*itThisRev == current)
					break;
			}
			RW_ASSERT(itThis != chain.end());
			RW_ASSERT(itThisRev != chain.rend());
			std::list<const TNTBody*>::reverse_iterator itPrev = itThisRev;
			std::list<const TNTBody*>::iterator itNext = itThis;
			itPrev++;
			itNext++;

			// Insert into chain at correct places (only at ends!) and do extra sanity check that neighbours are valid
			if (bodyA != NULL) {
				if (itPrev != chain.rend()) {
					RW_ASSERT(*itPrev == bodyA || *itPrev == bodyB);
				} else {
					chain.push_front(bodyA);
				}
			}
			if (bodyB != NULL) {
				if (itNext != chain.end()) {
					RW_ASSERT(*itNext == bodyA || *itNext == bodyB);
				} else {
					chain.push_back(bodyB);
				}
			}
		}
		chains.push_back(chain);
		RW_ASSERT(handled.size() == bodies.size());
	}
	RW_ASSERT(componentToBodies.size() == chains.size());

	// Each chain is now constructed as a vector to allow using indices afterwards
	std::list<std::vector<const TNTBody*> > chainsVec;
	BOOST_FOREACH(const std::list<const TNTBody*>& chain, chains) {
		std::vector<const TNTBody*> vec = std::vector<const TNTBody*>(chain.size(),NULL);
		std::size_t i = 0;
		BOOST_FOREACH(const TNTBody* const body, chain) {
			vec[i] = body;
			i++;
		}
		chainsVec.push_back(vec);
	}
	RW_ASSERT(chainsVec.size() == chains.size());

	// Now handle each chain
	std::size_t chainID = 0;
	BOOST_FOREACH(const std::vector<const TNTBody*>& chain, chainsVec) {
		RW_ASSERT(chain.size() >= 2);
#ifdef TNT_DEBUG_ENABLE_BOUNCING
		std::stringstream sstr;
		sstr << "Handling chain with bodies:";
		BOOST_FOREACH(const TNTBody* const body, chain) {
			sstr << " " << body->get()->getName();
		}
		TNT_DEBUG_BOUNCING(sstr.str());
#endif
		// First check that at least one of the originating contacts in chain has relative velocity higher than threshold
		bool velExceedingThreshold = false;
		BOOST_FOREACH(const std::size_t contactID, componentToContacts[chainID]) {
			const TNTContact* const contact = contacts[contactID];
			const Vector3D<> velP = contact->getVelocityParentW(tntstate,rwstate).linear();
			const Vector3D<> velC = contact->getVelocityChildW(tntstate,rwstate).linear();
			const Vector3D<> n = contact->getNormalW(tntstate);
			const double relVel = dot(velC-velP,n);
			if (fabs(relVel) > 1e-16) {
				velExceedingThreshold = true;
				break;
			}
		}
		if (!velExceedingThreshold) {
			TNT_DEBUG_BOUNCING("Relative velocity of all initiating contacts in chain was below threshold - skipping impulse solver.");
			continue;
		}

		// Construct initial list of indices for object pairs where impulse(s) origins
		bool equal = true;
		std::set<std::size_t> indices;
		std::vector<std::vector<const TNTContact*> > indexToContacts(chain.size()-1);
		BOOST_FOREACH(const std::size_t contactID, componentToContacts[chainID]) {
			const TNTContact* const contact = contacts[contactID];
			const TNTBody* const bodyA = contact->getParent();
			const TNTBody* const bodyB = contact->getChild();
			// Search
			std::size_t i;
			for (i = 0; i < chain.size()-1; i++) {
				const TNTBody* const body = chain[i];
				const TNTBody* const bodyNext = chain[i+1];
				if (body == bodyA && bodyNext == bodyB)
					break;
				if (body == bodyB && bodyNext == bodyA)
					break;
			}
			bool eq = i%2 == 0;
			if (indices.size() == 0) {
				equal = eq;
				indices.insert(i);
				RW_ASSERT(i < indexToContacts.size());
				indexToContacts[i].push_back(contact);
			} else {
				if (equal != eq)
					RW_THROW("TNTCollisionSolverChain (applyImpulses): only every second object pair in chain can have an impulse.");
				else {
					indices.insert(i);
					RW_ASSERT(i < indexToContacts.size());
					indexToContacts[i].push_back(contact);
				}
			}
		}

		// Now solve the initial impulses
		BOOST_FOREACH(const std::size_t index, indices) {
			const std::vector<const TNTContact*>& contactsOrigin = indexToContacts[index];
			RW_ASSERT(contactsOrigin.size() > 0);
			TNT_DEBUG_BOUNCING("Solving chain with " << contactsOrigin.size() << " initial penetrating contacts.");
			_solver->applyImpulses(contactsOrigin,bc,map,tntstate,rwstate);
		}

		// Now solve even and uneven pairs in shift until no object pairs have penetrating contacts
		const std::size_t chainSize = chain.size();
		std::size_t iterations = 0;
		if (chainSize > 2) {
			bool penetrations = true;
			bool shift = true;
			while (penetrations && iterations <= TNT_CONTACTRESOLVER_MAX_ITERATIONS) {
				iterations++;
				penetrations = false;
				std::size_t start;
				if ((equal && shift) || (!equal && !shift))
					start = 2;
				else
					start = 1;
				for (std::size_t i = start; i < chainSize; i += 2) {
					const TNTBody* const bodyA = chain[i-1];
					const TNTBody* const bodyB = chain[i];
					const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyA,bodyB,tntstate);
					bool solvePair = false;
					BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
						if (!solvePair) {
							const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
							const VelocityScrew6D<>& velI = constraint->getVelocityParentW(tntstate,rwstate);
							const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(tntstate,rwstate);
							const Vector3D<> linRelVel = velI.linear()-velJ.linear();
							const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
							if (contact) {
								const Vector3D<> nij = contact->getNormalW(tntstate);
								solvePair = dot(-linRelVel,nij) < 0;
							} else {
								const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
								Vector3D<> linRelVelConstraint = Vector3D<>::zero();
								Vector3D<> angRelVelConstraint = Vector3D<>::zero();
								for (std::size_t modeI = 0; modeI < 6; modeI++) {
									if (modes[modeI] == TNTConstraint::Velocity) {
										if (modeI < 3) {
											const Vector3D<> dir = constraint->getLinearRotationParent().getCol(modeI);
											linRelVelConstraint += dot(dir,linRelVel)*dir;
										} else {
											const Vector3D<> dir = constraint->getAngularRotationParent().getCol(modeI);
											angRelVelConstraint += dot(dir,angRelVel)*dir;
										}
									}
								}
								if (linRelVelConstraint.normInf() > 1e-8)
									solvePair = true;
								else if (angRelVelConstraint.normInf() > 1e-8)
									solvePair = true;
							}
						}
						if (solvePair)
							break;
					}
					if (solvePair) {
						_solver->applyImpulses(bodyA,bodyB,bc,map,tntstate,rwstate);
						penetrations = true;
					}
				}
				shift = !shift;
				if (iterations == TNT_CONTACTRESOLVER_MAX_ITERATIONS)
					RW_THROW("TNTCollisionSolverChain (applyImpulses): maximum number of iterations reached: " << TNT_CONTACTRESOLVER_MAX_ITERATIONS);
			}
		}
		chainID++;
	}
}
