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
#include "TNTUtil.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraint.hpp"
#include "TNTContact.hpp"
#include "TNTIslandState.hpp"
#include "TNTSettings.hpp"

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
	State tmpRWState;
	bool repeat = true;
	unsigned int iterations = 0;
	while (repeat) {
		if (iterations == TNT_CONTACTRESOLVER_MAX_ITERATIONS)
			RW_THROW("TNTContactResolverNonPenetration (solve): maximum number of iterations (" << TNT_CONTACTRESOLVER_MAX_ITERATIONS << ") reached in contact resolution.");
		iterations++;
		repeat = false;
		const Eigen::VectorXd solution = _solver->solve(h, rwstate, tntstate);
		tmpState = tntstate;
		tmpRWState = rwstate;
		_solver->saveSolution(solution,tmpState);
		resState = tmpState;
		TNTUtil::step(h,_solver->getGravity(),_solver->getManager(),tmpState,tmpRWState);
		BOOST_FOREACH(TNTContact* contact, contacts) {
			const Vector3D<> linVelI = contact->getVelocityParentW(tmpState,tmpRWState).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(tmpState,tmpRWState).linear();
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
