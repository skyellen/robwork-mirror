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

#include <rw/common/ThreadTask.hpp>
#include <rwsim/dynamics/Body.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPECollisionSolverSimultaneous.hpp"
#include "RWPECollisionSolverSingle.hpp"

#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyFixed.hpp"
#include "RWPEContact.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rwsimlibs::rwpe;

#define PROPERTY_RESOLVER_TOLERANCE "RWPECollisionSolverResolverTolerance"
#define PROPERTY_SVD_PRECISION "RWPECollisionSolverSingularValuePrecision"

class RWPECollisionSolverSingle::SolvePairTask: public ThreadTask {
public:
	SolvePairTask(ThreadTask::Ptr parent,
			const RWPEBodyDynamic* const bodyA,
			const std::vector<const RWPEBody*>& bodyB,
			const RWPEBodyConstraintGraph& bc,
			const RWPEMaterialMap* const map,
			RWPEIslandState& islandState,
			const State& rwstate,
			const PropertyMap& pmap,
			RWPELogUtil* log = NULL):
				ThreadTask(parent),
				_bodyA(bodyA),_bodyB(bodyB),_bc(bc),_map(map),_islandState(islandState),_rwstate(rwstate),_pmap(pmap),_log(log)
{
}

	void run() {
		const bool doLog = (_log == NULL)? false : _log->doLog();
		std::list<const RWPEBodyDynamic*> component;
		std::list<const RWPEConstraint*> constraints;
		if (_bodyA == NULL) {
			try {
				if (doLog)
					_log->log("Failed",RWPE_LOCATION) << "RWPECollisionSolverSingle::SolvePairTask run(): body A was NULL!";
				RW_THROW("RWPECollisionSolverSingle::SolvePairTask run(): body A was NULL!");
			} catch(const Exception& e) {
				registerFailure(e);
			}
			return;
		}
		component.push_back(_bodyA);
		BOOST_FOREACH(const RWPEBody* const body,_bodyB) {
			if (component.size() < 2) {
				if (const RWPEBodyDynamic* const rbody = dynamic_cast<const RWPEBodyDynamic*>(body))
					component.push_back(rbody);
			}
			const std::list<const RWPEConstraint*> list = _bc.getConstraints(_bodyA,body,_islandState);
			constraints.insert(constraints.end(),list.begin(),list.end());
		}
		RW_ASSERT(component.size() > 0);
		try {
			if (doLog)
				_log->log("Simultaneous Solver",RWPE_LOCATION);
			RWPECollisionSolverSimultaneous::resolveContacts(component,constraints,_map,_islandState,_rwstate,_pmap,_log);
		} catch(const Exception& e) {
			registerFailure(e);
		}
	}

private:
	const RWPEBodyDynamic* const _bodyA;
	std::vector<const RWPEBody*> _bodyB;
	const RWPEBodyConstraintGraph& _bc;
	const RWPEMaterialMap* const _map;
	RWPEIslandState& _islandState;
	const State& _rwstate;
	const PropertyMap& _pmap;
	RWPELogUtil* _log;
};

RWPECollisionSolverSingle::RWPECollisionSolverSingle()
{
}

RWPECollisionSolverSingle::~RWPECollisionSolverSingle() {
}

void RWPECollisionSolverSingle::doCollisions(
	const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& bc,
	const RWPEMaterialMap* map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log,
	ThreadTask::Ptr task) const
{
	const bool doLog = (log == NULL) ? false : log->doLog();

	// Arrange contacts in bins (if there are multiple collisions between independent pairs of objects)
	std::vector<std::vector<const RWPEContact*> > bins;
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		bool found = false;
		const RWPEBody* bodyA = contact->getParent();
		const RWPEBody* bodyB = contact->getChild();
		if (bodyA > bodyB) {
			bodyA = contact->getChild();
			bodyB = contact->getParent();
		}
		// Search for existing bin for the object pair.
		BOOST_FOREACH(std::vector<const RWPEContact*>& bin, bins) {
			const RWPEBody* const parent = bin.front()->getParent();
			const RWPEBody* const child = bin.front()->getChild();
			if ((bodyA == parent && bodyB == child) || (bodyA == child && bodyB == parent)) {
				bin.push_back(contact);
				found = true;
				break;
			}
		}
		if (!found) {
			// Create new bin for the contact
			const RWPEBodyDynamic* const rBodyA = dynamic_cast<const RWPEBodyDynamic*>(bodyA);
			const RWPEBodyDynamic* const rBodyB = dynamic_cast<const RWPEBodyDynamic*>(bodyB);
			if (!rBodyA && !rBodyB) {
				if (doLog)
					log->log("FATAL ERROR",RWPE_LOCATION) << "The collision solver can not solve impulses between the two kinematic/static bodies \"" << bodyA->get()->getName() << "\" and \"" << bodyB->get()->getName() << "\".";
				RW_THROW("RWPECollisionSolverSingle (doCollisions): cannot solve impulses for contact between a pair of kinematic/static bodies.");
			}
			// Check if it is chain
			bool chain = false;
			if (rBodyA) {
				const RWPEBodyConstraintGraph::ConstraintListConst constraints = bc.getConstraints(bodyA, islandState);
				BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
					const RWPEBody* cBodyA = constraint->getParent();
					const RWPEBody* cBodyB = constraint->getChild();
					if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
						chain = true;
					}
				}
			}
			if (!chain && rBodyB) {
				const RWPEBodyConstraintGraph::ConstraintListConst constraints = bc.getConstraints(bodyB, islandState);
				BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
					const RWPEBody* cBodyA = constraint->getParent();
					const RWPEBody* cBodyB = constraint->getChild();
					if ((bodyA != cBodyA || bodyB != cBodyB) && (bodyA != cBodyB || bodyB != cBodyA)) {
						chain = true;
					}
				}
			}
			if (chain) {
				if (doLog)
					log->log("FATAL ERROR",RWPE_LOCATION) << "Chain encountered in the collision solver for single collisions.";
				RW_THROW("RWPECollisionSolverSingle (doCollisions): this collision solver can not handle impulse chains!");
			}
			bins.push_back(std::vector<const RWPEContact*>(1,contact));
		}
	}
	// Now handle each independent bin
	BOOST_FOREACH(const std::vector<const RWPEContact*>& bin, bins) {
		RW_ASSERT(bin.size() > 0);
		const RWPEBody* const bodyA = bin[0]->getParent();
		const RWPEBody* const bodyB = bin[0]->getChild();
		const RWPEBodyDynamic* const rBodyA = dynamic_cast<const RWPEBodyDynamic*>(bodyA);
		const RWPEBodyDynamic* const rBodyB = dynamic_cast<const RWPEBodyDynamic*>(bodyB);
		const RWPEBodyDynamic* rigidBody;
		std::vector<const RWPEBody*> other(1,NULL);
		if (rBodyA != NULL) {
			rigidBody = rBodyA;
			other[0] = bodyB;
		} else if (rBodyB != NULL) {
			rigidBody = rBodyB;
			other[0] = bodyA;
		} else {
			RW_THROW("RWPECollisionSolverSingle (doCollisions): neither of the given bodies are rigid as required!");
		}
		const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,other,bc,map,islandState,rwstate,pmap));
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

void RWPECollisionSolverSingle::doCollisions(
	const RWPEBody* bodyA,
	const RWPEBody* bodyB,
	const RWPEBodyConstraintGraph& bc,
	const RWPEMaterialMap* map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log,
	ThreadTask::Ptr task) const
{
	const RWPEBodyDynamic* const rBodyA = dynamic_cast<const RWPEBodyDynamic*>(bodyA);
	const RWPEBodyDynamic* const rBodyB = dynamic_cast<const RWPEBodyDynamic*>(bodyB);
	const RWPEBodyDynamic* rigidBody;
	std::vector<const RWPEBody*> other(1,NULL);
	if (rBodyA != NULL) {
		rigidBody = rBodyA;
		other[0] = bodyB;
	} else if (rBodyB != NULL) {
		rigidBody = rBodyB;
		other[0] = bodyA;
	} else {
		RW_THROW("RWPECollisionSolverSingle (doCollisions): neither of the given bodies are rigid as required!");
	}
	RWPELogUtil* const sublog = (log == NULL)? NULL : log->parallel("Pair " + rigidBody->get()->getName() + "-" + other[0]->get()->getName(),RWPE_LOCATION);
	const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,other,bc,map,islandState,rwstate,pmap,sublog));
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


void RWPECollisionSolverSingle::doCollisions(
	const RWPEBodyDynamic* rigidBody,
	const std::vector<const RWPEBodyFixed*> fixedBodies,
	const RWPEBodyConstraintGraph& bc,
	const RWPEMaterialMap* map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	class RWPELogUtil* log,
	ThreadTask::Ptr task) const
{
	const std::vector<const RWPEBody*> fBodies(fixedBodies.begin(),fixedBodies.end());
	RWPELogUtil* const sublog = (log == NULL)? NULL : log->parallel(rigidBody->get()->getName() + " to fixed bodies.",RWPE_LOCATION);
	const ThreadTask::Ptr mainTask = ownedPtr(new SolvePairTask(task,rigidBody,fBodies,bc,map,islandState,rwstate,pmap,sublog));
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

void RWPECollisionSolverSingle::addDefaultProperties(PropertyMap& map) const {
	RWPECollisionSolverSimultaneous solver;
	solver.addDefaultProperties(map);
}
