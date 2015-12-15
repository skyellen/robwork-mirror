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

#include "RWPEContactResolverFull.hpp"
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEConstraint.hpp"
#include "RWPEConstraintSolver.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

RWPEContactResolverFull::RWPEContactResolverFull():
	_solver(NULL)
{
}

RWPEContactResolverFull::RWPEContactResolverFull(const RWPEConstraintSolver* const solver):
	_solver(solver)
{
}

RWPEContactResolverFull::~RWPEContactResolverFull() {
}

const RWPEContactResolver* RWPEContactResolverFull::createResolver(const RWPEConstraintSolver* solver) const {
	return new RWPEContactResolverFull(solver);
}

void RWPEContactResolverFull::solve(const std::vector<RWPEContact*>& persistentContacts,
	double h, bool discontinuity, const RWPEMaterialMap& map, const State &rwstate,
	const RWPEIslandState &islandState0, RWPEIslandState &islandStateH,
	const PropertyMap& pmap,
	class RWPELogUtil* log) const
{
	if (_solver == NULL)
		RW_THROW("RWPEContactResolverFull (solve): There is no RWPEConstraintSolver set for this resolver - please construct a new resolver for RWPEConstraintSolver to use.");

	const RWPEBodyConstraintGraph::ConstraintList constraints = _solver->getManager()->getTemporaryConstraints(&islandStateH);
	std::vector<RWPEContact*> contacts;
	BOOST_FOREACH(RWPEConstraint* const constraint, constraints) {
		if (RWPEContact* const contact = dynamic_cast<RWPEContact*>(constraint)) {
			contact->setType(RWPEContact::Sliding,RWPEContact::None);
			contacts.push_back(contact);
		}
	}
	const Eigen::VectorXd solution = _solver->solve(h, discontinuity, rwstate, islandState0, islandStateH, pmap, log);
	_solver->saveSolution(solution,islandStateH, log);
	// remove the contacts that became leaving (we need an active set from the solver?):
	// _solver->getManager()->removeTemporaryConstraint(contact,resState);
}

void RWPEContactResolverFull::addDefaultProperties(PropertyMap& map) const {
}
