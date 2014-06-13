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

#include "TNTContactResolverNonPenetration.hpp"
#include "TNTSolver.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraint.hpp"
#include "TNTContact.hpp"
#include "TNTIslandState.hpp"
#include "TNTSettings.hpp"
#include "TNTRigidBody.hpp"
#include "TNTIntegrator.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

TNTContactResolverNonPenetration::TNTContactResolverNonPenetration():
	_solver(NULL)
{
}

TNTContactResolverNonPenetration::TNTContactResolverNonPenetration(const TNTSolver* const solver):
	_solver(solver)
{
}

TNTContactResolverNonPenetration::~TNTContactResolverNonPenetration() {
}

const TNTContactResolver* TNTContactResolverNonPenetration::createResolver(const TNTSolver* solver) const {
	return new TNTContactResolverNonPenetration(solver);
}

void TNTContactResolverNonPenetration::solve(const std::vector<TNTContact*>& persistentContacts, double h, const State &rwstate, TNTIslandState &tntstate) const {
	if (_solver == NULL)
		RW_THROW("TNTContactResolverNonPenetration (solve): There is no TNTSolver set for this resolver - please construct a new resolver for TNTSolver to use.");
	TNTIslandState resState;
	const TNTBodyConstraintManager::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&tntstate);
	std::vector<TNTContact*> contacts;
	BOOST_FOREACH(TNTConstraint* const constraint, constraints) {
		if (TNTContact* const contact = dynamic_cast<TNTContact*>(constraint)) {
			contact->setTypeLeaving();
			contacts.push_back(contact);
		}
	}
	TNTIslandState tmpState;
	bool repeat = true;
	unsigned int iterations = 0;
	std::list<std::list<bool> > testedCombinations;
	std::list<std::list<bool> > allCombinations;
	bool testedAll = false;
	while (repeat) {
		if (iterations == TNT_CONTACTRESOLVER_MAX_ITERATIONS)
			RW_THROW("TNTContactResolverNonPenetration (solve): maximum number of iterations (" << TNT_CONTACTRESOLVER_MAX_ITERATIONS << ") reached in contact resolution.");
		// Check if we have already tested this combination before.
		bool testedCombination = false;
		BOOST_FOREACH(const std::list<bool>& comb, testedCombinations) {
			bool match = true;
			std::size_t i = 0;
			BOOST_FOREACH(const bool val, comb) {
				const TNTContact* const contact = contacts[i];
				if (!val) {
					if (!contact->isLeaving()) {
						match = false;
						break;
					}
				} else {
					if (contact->isLeaving() || contact->getTypeLinear() != TNTContact::Sliding || contact->getTypeAngular() != TNTContact::Sliding) {
						match = false;
						break;
					}
				}
				i++;
			}
			if (match) {
				testedCombination = true;
				if (allCombinations.size() > 0) {
					allCombinations.erase(allCombinations.begin());
					if (allCombinations.size() == 0)
						testedAll = true;
				}
				break;
			}
		}
		if (testedAll)
			RW_THROW("TNTContactResolverNonPenetration (solve): all combinations tested - none was valid.");
		// If loop was found, we break it by suggesting a new combination
		if (testedCombination) {
			// Construct list of all possible combinations if not already done
			if (allCombinations.size() == 0) {
				const std::size_t nrOfContacts = contacts.size();
				RW_ASSERT(nrOfContacts > 0);
				std::size_t nrOfComb = 2;
				for (std::size_t i = 1; i < nrOfContacts; i++)
					nrOfComb *= 2;
				allCombinations.resize(nrOfComb);
				std::size_t i = 0;
				BOOST_FOREACH(std::list<bool>& comb, allCombinations) {
					std::size_t val = i;
					for (std::size_t k = 0; k < nrOfContacts; k++) {
						comb.push_front(val%2);
						val = val >> 1;
					}
					i++;
				}
			}
			// Now try the first combination in list
			std::size_t i = 0;
			BOOST_FOREACH(const bool val, allCombinations.front()) {
				TNTContact* const contact = contacts[i];
				if (!val)
					contact->setTypeLeaving();
				else
					contact->setType(TNTContact::Sliding,TNTContact::Sliding);
				i++;
			}
			continue;
		}
		// Add the current combinations to the list of tested combinations
		testedCombinations.resize(testedCombinations.size()+1);
		BOOST_FOREACH(const TNTContact* const contact, contacts) {
			if (contact->isLeaving()) {
				testedCombinations.back().push_back(false);
			} else {
				const TNTContact::Type linType = contact->getTypeLinear();
				const TNTContact::Type angType = contact->getTypeAngular();
				if (linType == TNTContact::Sticking) {
					RW_THROW("TNTContactResolverNonPenetration (solve): contact should not be sticking in this resolver.");
				} else if (linType == TNTContact::Sliding) {
					if (angType == TNTContact::Sticking)
						RW_THROW("TNTContactResolverNonPenetration (solve): contact should not be sticking in this resolver.");
					else if (angType == TNTContact::Sliding)
						testedCombinations.back().push_back(true);
					else
						RW_THROW("TNTContactResolverNonPenetration (solve): encountered unknown angular contact type.");
				} else {
					RW_THROW("TNTContactResolverNonPenetration (solve): encountered unknown linear contact type.");
				}
			}
		}

		// Now try to solve and check if the solution is valid
		iterations++;
		repeat = false;
		const Eigen::VectorXd solution = _solver->solve(h, rwstate, tntstate);
		tmpState = tntstate;
		_solver->saveSolution(solution,tmpState);
		resState = tmpState;
		{
			// Update velocities (but keep the same positions)
			const TNTBodyConstraintManager::DynamicBodyList rbodies = _solver->getManager()->getDynamicBodies();
			BOOST_FOREACH(const TNTRigidBody* rbody, rbodies) {
				TNTRigidBody::RigidConfiguration* config = dynamic_cast<TNTRigidBody::RigidConfiguration*>(tmpState.getConfiguration(rbody));
				const Transform3D<> wTcom = config->getWorldTcom();
				rbody->getIntegrator()->integrate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config,tmpState,rwstate);
				config->setWorldTcom(wTcom);
			}
		}
		BOOST_FOREACH(TNTContact* contact, contacts) {
			const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
			const Vector3D<> linRelVel = linVelI-linVelJ;
			const Vector3D<> nij = contact->getNormalW(tmpState);
			if (contact->isLeaving()) {
				const bool penetrating = dot(-linRelVel,nij) < -1e-8;
				if (penetrating) {
					repeat = true;
					contact->setType(TNTContact::Sliding,TNTContact::Sliding);
				}
			} else {
				const Wrench6D<> wrench = contact->getWrenchConstraint(tmpState);
				if (dot(wrench.force(),nij) > 0) {
					repeat = true;
					contact->setTypeLeaving();
				}
			}
		}
	}
	BOOST_FOREACH(TNTContact* contact, contacts) {
		if (contact->isLeaving()) {
			bool found = false;
			BOOST_FOREACH(TNTContact* persistent, persistentContacts) {
				if (contact == persistent) {
					found = true;
					break;
				}
			}
			if (!found) {
				_solver->getManager()->removeTemporaryConstraint(contact,resState);
			}
		}
	}
	tntstate = resState;
}
