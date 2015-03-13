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

#include "TNTContactResolverFull.hpp"
#include "TNTSolver.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraint.hpp"
#include "TNTContact.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

TNTContactResolverFull::TNTContactResolverFull():
	_solver(NULL)
{
}

TNTContactResolverFull::TNTContactResolverFull(const TNTSolver* const solver):
	_solver(solver)
{
}

TNTContactResolverFull::~TNTContactResolverFull() {
}

const TNTContactResolver* TNTContactResolverFull::createResolver(const TNTSolver* solver) const {
	return new TNTContactResolverFull(solver);
}

void TNTContactResolverFull::solve(const std::vector<TNTContact*>& persistentContacts,
	double h, bool discontinuity, const TNTMaterialMap& map, const State &rwstate,
	const TNTIslandState &tntstate0, TNTIslandState &tntstateH,
	const PropertyMap& pmap) const
{
	if (_solver == NULL)
		RW_THROW("TNTContactResolverFull (solve): There is no TNTSolver set for this resolver - please construct a new resolver for TNTSolver to use.");

	const TNTBodyConstraintManager::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&tntstateH);
	std::vector<TNTContact*> contacts;
	BOOST_FOREACH(TNTConstraint* const constraint, constraints) {
		if (TNTContact* const contact = dynamic_cast<TNTContact*>(constraint)) {
			contact->setType(TNTContact::None,TNTContact::None);
			contacts.push_back(contact);
		}
	}
	const Eigen::VectorXd solution = _solver->solve(h, discontinuity, rwstate, tntstate0, tntstateH, pmap);
	_solver->saveSolution(solution,tntstateH);
	// remove the contacts that became leaving (we need an active set from the solver?):
	// _solver->getManager()->removeTemporaryConstraint(contact,resState);
}

void TNTContactResolverFull::addDefaultProperties(PropertyMap& map) const {
}
