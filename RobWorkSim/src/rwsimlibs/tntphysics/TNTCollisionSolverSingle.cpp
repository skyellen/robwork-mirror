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

#include "TNTCollisionSolverSingle.hpp"
#include "TNTCollisionSolverSimultaneous.hpp"
#include "TNTSettings.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTContact.hpp"
#include "TNTFixedBody.hpp"
#include "TNTRigidBody.hpp"

#include <rw/common/ThreadTask.hpp>

#if TNT_DEBUG_ENABLE_BOUNCING
#include <rwsim/dynamics/Body.hpp>
using namespace rwsim::dynamics;
#endif

using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_RESOLVER_TOLERANCE "TNTCollisionSolverResolverTolerance"
#define PROPERTY_SVD_PRECISION "TNTCollisionSolverSingularValuePrecision"

class TNTCollisionSolverSingle::SolvePairTask: public ThreadTask {
public:
	SolvePairTask(ThreadTask::Ptr parent,
			const TNTRigidBody* const bodyA,
			const std::vector<const TNTBody*>& bodyB,
			const TNTBodyConstraintManager& bc,
			const TNTMaterialMap* const map,
			TNTIslandState& tntstate,
			const State& rwstate,
			const PropertyMap& pmap):
				ThreadTask(parent),
				_bodyA(bodyA),_bodyB(bodyB),_bc(bc),_map(map),_tntstate(tntstate),_rwstate(rwstate),_pmap(pmap)
{
}

	void run() {
		std::list<const TNTRigidBody*> component;
		std::list<const TNTConstraint*> constraints;
		if (_bodyA == NULL) {
			try {
				RW_THROW("TNTCollisionSolverSingle::SolvePairTask run(): body A was NULL!");
			} catch(const Exception& e) {
				registerFailure(e);
			}
			return;
		}
		component.push_back(_bodyA);
		BOOST_FOREACH(const TNTBody* const body,_bodyB) {
			if (component.size() < 2) {
				if (const TNTRigidBody* const rbody = dynamic_cast<const TNTRigidBody*>(body))
					component.push_back(rbody);
			}
			const std::list<const TNTConstraint*> list = _bc.getConstraints(_bodyA,body,_tntstate);
			constraints.insert(constraints.end(),list.begin(),list.end());
		}
		RW_ASSERT(component.size() > 0);
		try {
			TNTCollisionSolverSimultaneous::resolveContacts(component,constraints,_map,_tntstate,_rwstate,_pmap);
		} catch(const Exception& e) {
			registerFailure(e);
		}
	}

private:
	const TNTRigidBody* const _bodyA;
	std::vector<const TNTBody*> _bodyB;
	const TNTBodyConstraintManager& _bc;
	const TNTMaterialMap* const _map;
	TNTIslandState& _tntstate;
	const State& _rwstate;
	const PropertyMap& _pmap;
};

TNTCollisionSolverSingle::TNTCollisionSolverSingle()
{
}

TNTCollisionSolverSingle::~TNTCollisionSolverSingle() {
}

void TNTCollisionSolverSingle::doCollisions(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& bc,
	const TNTMaterialMap* map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap,
	ThreadTask::Ptr task) const
{
	// Arrange contacts in bins (if there are multiple collisions between independent pairs of objects)
	std::vector<std::vector<const TNTContact*> > bins;
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		bool found = false;
		const TNTBody* bodyA = contact->getParent();
		const TNTBody* bodyB = contact->getChild();
		if (bodyA > bodyB) {
			bodyA = contact->getChild();
			bodyB = contact->getParent();
		}
		// Search for existing bin for the object pair.
		BOOST_FOREACH(std::vector<const TNTContact*>& bin, bins) {
			const TNTBody* const parent = bin.front()->getParent();
			const TNTBody* const child = bin.front()->getChild();
			if ((bodyA == parent && bodyB == child) || (bodyA == child && bodyB == parent)) {
				bin.push_back(contact);
				found = true;
				break;
			}
		}
		if (!found) {
			// Create new bin for the contact
			const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
			const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
			if (!rBodyA && !rBodyB) {
				TNT_DEBUG_BOUNCING("The collision solver can not solve impulses between the two kinematic/static bodies \"" << bodyA->get()->getName() << "\" and \"" << bodyB->get()->getName() << "\".");
				RW_THROW("TNTCollisionSolverSingle (doCollisions): cannot solve impulses for contact between a pair of kinematic/static bodies.");
			}
			// Check if it is chain
			bool chain = false;
			const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyA, tntstate);
			if (rBodyA) {
				const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyA, tntstate);
				BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
					const TNTBody* cBodyA = constraint->getParent();
					const TNTBody* cBodyB = constraint->getChild();
					if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
						chain = true;
					}
				}
			}
			if (!chain && bodyB)
				const TNTBodyConstraintManager::ConstraintListConst constraints = bc.getConstraints(bodyB, tntstate);
			BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
				const TNTBody* cBodyA = constraint->getParent();
				const TNTBody* cBodyB = constraint->getChild();
				if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
					chain = true;
				}
			}
			if (chain) {
				TNT_DEBUG_BOUNCING("Chain encountered in the collision solver for single collisions.");
				RW_THROW("TNTCollisionSolverSingle (doCollisions): this collision solver can not handle impulse chains!");
			}
			bins.push_back(std::vector<const TNTContact*>(1,contact));
		}
	}
	// Now handle each independent bin
	BOOST_FOREACH(const std::vector<const TNTContact*>& bin, bins) {
		RW_ASSERT(bin.size() > 0);
		const TNTBody* const bodyA = bin[0]->getParent();
		const TNTBody* const bodyB = bin[0]->getChild();
		const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
		const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
		const TNTRigidBody* rigidBody;
		std::vector<const TNTBody*> other(1,NULL);
		if (bodyA != NULL) {
			rigidBody = rBodyA;
			other[0] = bodyB;
		} else if (rBodyB != NULL) {
			rigidBody = rBodyB;
			other[0] = bodyA;
		} else {
			RW_THROW("TNTCollisionSolverSingle (doCollisions): neither of the given bodies are rigid as required!");
		}
		const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,other,bc,map,tntstate,rwstate,pmap));
		if (task == NULL) {
			mainTask->execute();
			const std::list<Exception> exceptions = mainTask->getExceptions();
			if (exceptions.size() > 0) {
				throw exceptions.front();
			}
		} else {
			task->addSubTask(mainTask);
		}
	}
}

void TNTCollisionSolverSingle::doCollisions(
	const TNTBody* bodyA,
	const TNTBody* bodyB,
	const TNTBodyConstraintManager& bc,
	const TNTMaterialMap* map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap,
	ThreadTask::Ptr task) const
{
	const TNTRigidBody* const rBodyA = dynamic_cast<const TNTRigidBody*>(bodyA);
	const TNTRigidBody* const rBodyB = dynamic_cast<const TNTRigidBody*>(bodyB);
	const TNTRigidBody* rigidBody;
	std::vector<const TNTBody*> other(1,NULL);
	if (rBodyA != NULL) {
		rigidBody = rBodyA;
		other[0] = bodyB;
	} else if (rBodyB != NULL) {
		rigidBody = rBodyB;
		other[0] = bodyA;
	} else {
		RW_THROW("TNTCollisionSolverSingle (doCollisions): neither of the given bodies are rigid as required!");
	}
	const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,other,bc,map,tntstate,rwstate,pmap));
	if (task == NULL) {
		mainTask->execute();
		const std::list<Exception> exceptions = mainTask->getExceptions();
		if (exceptions.size() > 0) {
			throw exceptions.front();
		}
	} else {
		task->addSubTask(mainTask);
	}
}


void TNTCollisionSolverSingle::doCollisions(
	const TNTRigidBody* rigidBody,
	const std::vector<const TNTFixedBody*> fixedBodies,
	const TNTBodyConstraintManager& bc,
	const TNTMaterialMap* map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap,
	ThreadTask::Ptr task) const
{
	const std::vector<const TNTBody*> fBodies(fixedBodies.begin(),fixedBodies.end());
	const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,fBodies,bc,map,tntstate,rwstate,pmap));
	if (task == NULL) {
		mainTask->execute();
		const std::list<Exception> exceptions = mainTask->getExceptions();
		if (exceptions.size() > 0) {
			throw exceptions.front();
		}
	} else {
		task->addSubTask(mainTask);
	}
}

void TNTCollisionSolverSingle::addDefaultProperties(PropertyMap& map) const {
	TNTCollisionSolverSimultaneous solver;
	solver.addDefaultProperties(map);
}
