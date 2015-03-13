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
#include "TNTRigidBody.hpp"
#include "TNTIntegrator.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_PROPMAXITERATIONS "TNTContactResolverMaxIterations"
#define PROPERTY_PROPMAXVEL "TNTContactResolverMaxPenetrationVelocity"
#define PROPERTY_PROPMAXFORCE "TNTContactResolverMaxAttractionForce"

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

void TNTContactResolverNonPenetration::solve(const std::vector<TNTContact*>& persistentContacts,
		double h, bool discontinuity, const TNTMaterialMap& map, const State &rwstate,
		const TNTIslandState &tntstate0, TNTIslandState &tntstateH,
		const PropertyMap& pmap) const
{
	if (_solver == NULL)
		RW_THROW("TNTContactResolverNonPenetration (solve): There is no TNTSolver set for this resolver - please construct a new resolver for TNTSolver to use.");
	int maxIterations = pmap.get<int>(PROPERTY_PROPMAXITERATIONS,-1);
	double maxPenetrationVelocity = pmap.get<double>(PROPERTY_PROPMAXVEL,-1);
	double maxAttractionForce = pmap.get<double>(PROPERTY_PROPMAXFORCE,-1);

	if (maxIterations < 0 || maxPenetrationVelocity < 0 || maxAttractionForce < 0) {
		PropertyMap tmpMap;
		addDefaultProperties(tmpMap);
		if (maxIterations < 0) {
			maxIterations = tmpMap.get<int>(PROPERTY_PROPMAXITERATIONS,-1);
			RW_ASSERT(maxIterations > 0);
		}
		if (maxPenetrationVelocity < 0) {
			maxPenetrationVelocity = tmpMap.get<double>(PROPERTY_PROPMAXVEL,-1);
			RW_ASSERT(maxPenetrationVelocity >= 0);
		}
		if (maxAttractionForce < 0) {
			maxAttractionForce = tmpMap.get<double>(PROPERTY_PROPMAXFORCE,-1);
			RW_ASSERT(maxAttractionForce >= 0);
		}
	}

	TNTIslandState resState;
	const TNTBodyConstraintManager::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&tntstateH);
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
		if ((int)iterations == maxIterations && maxIterations != 0)
			RW_THROW("TNTContactResolverNonPenetration (solve): maximum number of iterations (" << maxIterations << ") reached in contact resolution - " << allCombinations.size() << " combinations in total.");
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
					if (contact->isLeaving()) {
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
					contact->setType(TNTContact::None,TNTContact::None);
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
				if (linType != TNTContact::None) {
					RW_THROW("TNTContactResolverNonPenetration (solve): only allowed linear contact type in this resolver is None.");
				} else {
					if (angType != TNTContact::None)
						RW_THROW("TNTContactResolverNonPenetration (solve): only allowed linear contact type in this resolver is None.");
					else
						testedCombinations.back().push_back(true);
				}
			}
		}

		// Now try to solve and check if the solution is valid
		iterations++;
		repeat = false;
		const Eigen::VectorXd solution = _solver->solve(h, discontinuity, rwstate, tntstate0, tntstateH, pmap);
		tmpState = tntstateH;
		_solver->saveSolution(solution,tmpState);
		resState = tmpState;
		{
			// Update velocities (but keep the same positions)
			const TNTBodyConstraintManager::DynamicBodyList rbodies = _solver->getManager()->getDynamicBodies();
			BOOST_FOREACH(const TNTRigidBody* rbody, rbodies) {
				const TNTRigidBody::RigidConfiguration* const config0 = dynamic_cast<TNTRigidBody::RigidConfiguration*>(tntstate0.getConfiguration(rbody));
				TNTRigidBody::RigidConfiguration* const config = dynamic_cast<TNTRigidBody::RigidConfiguration*>(tmpState.getConfiguration(rbody));
				//const Transform3D<> wTcom = config->getWorldTcom();
				//rbody->getIntegrator()->integrate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config,tmpState,rwstate);
				//config->setWorldTcom(wTcom);
				rbody->getIntegrator()->velocityUpdate(_solver->getManager()->getConstraints(rbody, tmpState),_solver->getGravity(),h,*config0,*config,tntstate0,tmpState,rwstate);
			}
		}
		BOOST_FOREACH(TNTContact* contact, contacts) {
			const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,rwstate).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,rwstate).linear();
			const Vector3D<> linRelVel = linVelI-linVelJ;
			const Vector3D<> nij = contact->getNormalW(tmpState);
			if (contact->isLeaving()) {
				const bool penetrating = dot(-linRelVel,nij) < -maxPenetrationVelocity;
				if (penetrating) {
					repeat = true;
					contact->setType(TNTContact::None,TNTContact::None);
				}
			} else {
				const Wrench6D<> wrench = contact->getWrenchConstraint(tmpState);
				if (dot(wrench.force(),nij) > -maxAttractionForce) {
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
	tntstateH = resState;
}

void TNTContactResolverNonPenetration::addDefaultProperties(PropertyMap& map) const {
	map.add<int>(PROPERTY_PROPMAXITERATIONS,"Stop if resolver exceeds this number of iterations (use 0 to test all combinations).",1000);
	map.add<double>(PROPERTY_PROPMAXVEL,"Continue resolving as long as there are contacts with penetrating relative velocities greater than this (m/s).",1e-5);
	map.add<double>(PROPERTY_PROPMAXFORCE,"Continue resolving as long as there are contacts with attracting forces greater than this (N).",0);
}
