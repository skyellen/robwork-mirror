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
#include "TNTFixedBody.hpp"
#include "TNTRigidBody.hpp"
#include "TNTSettings.hpp"
#include "TNTContact.hpp"
#include "TNTBodyConstraintManager.hpp"

#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadTask.hpp>

#ifdef TNT_DEBUG_ENABLE_BOUNCING
#include <rwsim/dynamics/Body.hpp>
#endif

#include <boost/thread/mutex.hpp>

#include <stack>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

#define PROPERTY_PROPTHRES_CONTACT "TNTCollisionSolverPropagateThresholdContact"
#define PROPERTY_PROPTHRES_CONSTRAINTLIN "TNTCollisionSolverPropagateThresholdConstraintLinear"
#define PROPERTY_PROPTHRES_CONSTRAINTANG "TNTCollisionSolverPropagateThresholdConstraintAngular"
#define PROPERTY_MAX_ITERATIONS "TNTCollisionSolverMaxIterations"

struct TNTCollisionSolverChain::SharedInfo {
	SharedInfo(const TNTCollisionSolverChain* solver, const PropertyMap& pmap, const TNTMaterialMap* const map, const State& rwstate):
		solver(solver), pmap(pmap), map(map), rwstate(rwstate),
		thresholdContact(0), thresholdConstraintLin(0), thresholdConstraintAng(0), maxIterations(0)
	{
	}

	void extractProperties() {
		// Extract the properties to use
		thresholdContact = pmap.get<int>(PROPERTY_PROPTHRES_CONTACT,-1);
		thresholdConstraintLin = pmap.get<int>(PROPERTY_PROPTHRES_CONSTRAINTLIN,-1);
		thresholdConstraintAng = pmap.get<int>(PROPERTY_PROPTHRES_CONSTRAINTANG,-1);
		maxIterations = pmap.get<int>(PROPERTY_MAX_ITERATIONS,-1);
		if (maxIterations < 0 || thresholdContact < 0 || thresholdConstraintLin < 0 || thresholdConstraintAng < 0) {
			PropertyMap tmpMap;
			solver->addDefaultProperties(tmpMap);
			if (maxIterations < 0) {
				maxIterations = tmpMap.get<int>(PROPERTY_MAX_ITERATIONS,-1);
				RW_ASSERT(maxIterations > 0);
			}
			if (thresholdContact < 0) {
				thresholdContact = tmpMap.get<double>(PROPERTY_PROPTHRES_CONTACT,-1);
				RW_ASSERT(thresholdContact > 0);
			}
			if (thresholdConstraintLin < 0) {
				thresholdConstraintLin = tmpMap.get<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,-1);
				RW_ASSERT(thresholdConstraintLin > 0);
			}
			if (thresholdConstraintAng < 0) {
				thresholdConstraintAng = tmpMap.get<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,-1);
				RW_ASSERT(thresholdConstraintAng > 0);
			}
		}
	}

	const TNTCollisionSolverChain* const solver;
	const PropertyMap& pmap;
	const TNTMaterialMap* const map;
	const State& rwstate;

	double thresholdContact;
	double thresholdConstraintLin;
	double thresholdConstraintAng;
	int maxIterations;
};

class TNTCollisionSolverChain::ChainTask: public ThreadTask {
public:
	ChainTask(ThreadTask::Ptr parent,
			const TNTBodyConstraintManager& component,
			const Chain chain,
			const std::set<std::size_t>& indices,
			TNTIslandState& tntstate,
			const SharedInfo& info):
				ThreadTask(parent),
				_component(component),_chain(chain),_indices(indices),_tntstate(tntstate),_info(info),
				_iterations(0),_runningTasks(0)
{
		setKeepAlive(true);
}

	void solveChain(const std::set<std::size_t>& indices) {
		BOOST_FOREACH(const std::size_t index, indices) {
			if (index == 0 || index == _chain.bodies.size()) {
				const TNTRigidBody* rigidBody;
				std::vector<const TNTFixedBody*> fixedBodies;
				if (index == 0) {
					rigidBody = dynamic_cast<const TNTRigidBody*>(_chain.bodies.front());
					RW_ASSERT(rigidBody != NULL);
					fixedBodies.insert(fixedBodies.begin(),_chain.fixedBodiesBegin.begin(),_chain.fixedBodiesBegin.end());
				} else {
					rigidBody = dynamic_cast<const TNTRigidBody*>(_chain.bodies.back());
					RW_ASSERT(rigidBody != NULL);
					fixedBodies.insert(fixedBodies.begin(),_chain.fixedBodiesEnd.begin(),_chain.fixedBodiesEnd.end());
				}
#ifdef TNT_DEBUG_ENABLE_BOUNCING
					std::stringstream sstr;
					sstr << "Solving for impulses between:";
					if (index == 0) {
						sstr << " {";
						BOOST_FOREACH(const TNTBody* const body, _chain.fixedBodiesBegin) {
							sstr << " " << body->get()->getName();
						}
						sstr << " } ";
						BOOST_FOREACH(const TNTBody* const body, _chain.bodies) {
							sstr << " " << body->get()->getName();
						}
					} else {
						BOOST_FOREACH(const TNTBody* const body, _chain.bodies) {
							sstr << " " << body->get()->getName();
						}
						sstr << " {";
						BOOST_FOREACH(const TNTBody* const body, _chain.fixedBodiesEnd) {
							sstr << " " << body->get()->getName();
						}
						sstr << " } ";
					}
					TNT_DEBUG_BOUNCING(sstr.str());
#endif
				TNT_DEBUG_BOUNCING(" - velocity before solving: " << rigidBody->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << rigidBody->getVelocityW(_info.rwstate,_tntstate).angular() << ".");
				_info.solver->_solver->doCollisions(rigidBody,fixedBodies,_component,_info.map,_tntstate,_info.rwstate,_info.pmap,this);
			} else {
				const TNTBody* const bodyA = _chain.bodies[index-1];
				const TNTBody* const bodyB = _chain.bodies[index];
				TNT_DEBUG_BOUNCING("Solving for impulses between " << bodyA->get()->getName() << " and " << bodyB->get()->getName() << ".");
				TNT_DEBUG_BOUNCING(" - velocities before solving: " << bodyA->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << bodyA->getVelocityW(_info.rwstate,_tntstate).angular() << " and " << bodyB->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << bodyB->getVelocityW(_info.rwstate,_tntstate).angular() << ".");
				_info.solver->_solver->doCollisions(bodyA,bodyB,_component,_info.map,_tntstate,_info.rwstate,_info.pmap,this);
			}
		}
	}

	void run() {
		if (_chain.bodies.size() == 0)
			return;
		// Solve the initial impulses
		{
			boost::mutex::scoped_lock lock(_mutex);
			_runningTasks += _indices.size();
		}
		solveChain(_indices);
	}

	void subTaskDone(ThreadTask* task) {
		{
			boost::mutex::scoped_lock lock(_mutex);
			_runningTasks--;
		}
		BOOST_FOREACH(const Exception& e, task->getExceptions()) {
			registerFailure(e);
		}
	}

	bool shouldSolve(const TNTConstraint* const constraint) {
		const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint);
		const VelocityScrew6D<>& velI = constraint->getVelocityParentW(_tntstate,_info.rwstate);
		const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(_tntstate,_info.rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
		if (contact) {
			const Vector3D<> nij = contact->getNormalW(_tntstate);
			return dot(-linRelVel,nij) < -_info.thresholdContact;
		} else {
			if (constraint->getDimVelocity() == 0)
				return false;
			const std::vector<TNTConstraint::Mode> modes = constraint->getConstraintModes();
			Vector3D<> linRelVelConstraint = Vector3D<>::zero();
			Vector3D<> angRelVelConstraint = Vector3D<>::zero();
			for (std::size_t modeI = 0; modeI < 6; modeI++) {
				if (modes[modeI] == TNTConstraint::Velocity) {
					if (modeI < 3) {
						const Vector3D<> dir = constraint->getLinearRotationParent().getCol(modeI);
						linRelVelConstraint += dot(dir,linRelVel)*dir;
					} else {
						const Vector3D<> dir = constraint->getAngularRotationParent().getCol(modeI-3);
						angRelVelConstraint += dot(dir,angRelVel)*dir;
					}
				}
			}
			if (linRelVelConstraint.normInf() > _info.thresholdConstraintLin)
				return true;
			else if (angRelVelConstraint.normInf() > _info.thresholdConstraintAng)
				return true;
		}
		return false;
	}

	void idle() {
		{
			boost::mutex::scoped_lock lock(_mutex);
			if (_runningTasks > 0)
				return;
		}
		if (getExceptions().size() > 0) {
			setKeepAlive(false);
			return;
		}

#if TNT_DEBUG_ENABLE_BOUNCING
		TNT_DEBUG_BOUNCING("Velocities after solving:");
		BOOST_FOREACH(const std::size_t index, _indices) {
			RW_ASSERT(index <= _chain.bodies.size());
			if (index == 0 || index == _chain.bodies.size()) {
				const TNTBody* body;
				if (index == 0)
					body = _chain.bodies.front();
				else
					body = _chain.bodies.back();
				TNT_DEBUG_BOUNCING(" - " << body->get()->getName() << ": " << body->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << body->getVelocityW(_info.rwstate,_tntstate).angular() << ".");
			} else {
				const TNTBody* const bodyA = _chain.bodies[index-1];
				const TNTBody* const bodyB = _chain.bodies[index];
				TNT_DEBUG_BOUNCING(" - " << bodyA->get()->getName() << " and " << bodyB->get()->getName() << ": " << bodyA->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << bodyA->getVelocityW(_info.rwstate,_tntstate).angular() << " and " << bodyB->getVelocityW(_info.rwstate,_tntstate).linear() << " - " << bodyB->getVelocityW(_info.rwstate,_tntstate).angular() << ".");
			}
		}
#endif

		_iterations++;
		if ((int)_iterations == _info.maxIterations) {
			try {
				RW_THROW("TNTCollisionSolverChain::ChainTask (idle): maximum number of iterations reached: " << _info.maxIterations);
			} catch(const Exception& e) {
				registerFailure(e);
				setKeepAlive(false);
				return;
			}
		}

		const bool equal = (*_indices.begin())%2 == 0;
		std::set<std::size_t> newIndices;
		const std::size_t start = equal ? 1 : 2;
		if (!equal) {
			const TNTRigidBody* const rigidBody = dynamic_cast<const TNTRigidBody*>(_chain.bodies.front());
			RW_ASSERT(_chain.fixedBodiesBegin.size() == 0 || rigidBody != NULL);
			BOOST_FOREACH(const TNTFixedBody* const fBody, _chain.fixedBodiesBegin) {
				const TNTBodyConstraintManager::ConstraintListConst constraints = _component.getConstraints(rigidBody,fBody,_tntstate);
				BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
					if (shouldSolve(constraint)) {
						newIndices.insert(0);
						break;
					}
				}
			}
		}
		for (std::size_t i = start; i < _chain.bodies.size(); i += 2) {
			const TNTBody* const bodyA = _chain.bodies[i-1];
			const TNTBody* const bodyB = _chain.bodies[i];
			const TNTBodyConstraintManager::ConstraintListConst constraints = _component.getConstraints(bodyA,bodyB,_tntstate);
			BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
				if (shouldSolve(constraint)) {
					newIndices.insert(i);
					break;
				}
			}
		}
		if (_chain.bodies.size()%2 == start%2) {
			const TNTRigidBody* const rigidBody = dynamic_cast<const TNTRigidBody*>(_chain.bodies.back());
			RW_ASSERT(_chain.fixedBodiesBegin.size() == 0 || rigidBody != NULL);
			BOOST_FOREACH(const TNTFixedBody* const fBody, _chain.fixedBodiesEnd) {
				const TNTBodyConstraintManager::ConstraintListConst constraints = _component.getConstraints(rigidBody,fBody,_tntstate);
				BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
					if (shouldSolve(constraint)) {
						newIndices.insert(_chain.bodies.size());
						break;
					}
				}
			}
		}
		_indices = newIndices;

		{
			boost::mutex::scoped_lock lock(_mutex);
			_runningTasks += newIndices.size();
		}
		// Solve impulse pairs in parallel
		solveChain(newIndices);

		if (newIndices.size() == 0)
			setKeepAlive(false);
	}

private:
	const TNTBodyConstraintManager& _component;
	const Chain _chain;
	std::set<std::size_t> _indices;
	TNTIslandState& _tntstate;
	const SharedInfo& _info;
	std::size_t _iterations;
	std::size_t _runningTasks;
	boost::mutex _mutex;
};

class TNTCollisionSolverChain::DecomposeTask: public ThreadTask {
public:
	typedef rw::common::Ptr<DecomposeTask> Ptr;

	DecomposeTask(ThreadTask::Ptr parent,
			const std::vector<const TNTContact*>& contacts,
			const TNTBodyConstraintManager* bc,
			TNTIslandState& tntstate,
			SharedInfo& info,
			bool decomposed = false):
				ThreadTask(parent),
				_contacts(contacts),_bc(bc),_tntstate(tntstate),_info(info),
				_decomposed(decomposed)
{
}

	void run() {
		if (!_decomposed) {
			// First load the properties
			_info.extractProperties();

			// Connected dynamic body components are found.
			const std::set<TNTBodyConstraintManager*> components = _bc->getDynamicComponents(_tntstate);
			if (components.size() > 0) {
				TNT_DEBUG_BOUNCING("Found " << components.size() << " component(s) for the " << _contacts.size() << " bouncing contact(s).");
			} else {
				TNT_DEBUG_BOUNCING("Found no component for the " << _contacts.size() << " bouncing contact(s).");
			}

			// Handle each component in parallel
			BOOST_FOREACH(const TNTBodyConstraintManager* component, components) {
					// If there are any initiating contacts in the component, a new subtask is forked for handling this
					const ThreadTask::Ptr subtask = ownedPtr(new DecomposeTask(this,_contacts,component,_tntstate,_info,true));
					addSubTask(subtask);
			}
		} else {
			// First find the initiating contacts for this component
			std::vector<const TNTContact*> componentContacts;
			BOOST_FOREACH(const TNTContact* const contact, _contacts) {
				BOOST_FOREACH(const TNTConstraint* const constraint, _bc->getPersistentConstraints()) {
					if (constraint == contact) {
						componentContacts.push_back(contact);
						break;
					}
				}
			}
			if (componentContacts.size() > 0) {
				try {
					// Construct chain and find the indicies for the initial collisions in the chain
					const Chain chain = TNTCollisionSolverChain::constructChain(componentContacts,*_bc,_tntstate);
#ifdef TNT_DEBUG_ENABLE_BOUNCING
					std::stringstream sstr;
					sstr << "Handling chain with bodies: {";
					BOOST_FOREACH(const TNTBody* const body, chain.fixedBodiesBegin) {
						sstr << " " << body->get()->getName();
					}
					sstr << " } ";
					BOOST_FOREACH(const TNTBody* const body, chain.bodies) {
						sstr << " " << body->get()->getName();
					}
					sstr << " { ";
					BOOST_FOREACH(const TNTBody* const body, chain.fixedBodiesEnd) {
						sstr << " " << body->get()->getName();
					}
					sstr << " } ";
					TNT_DEBUG_BOUNCING(sstr.str());
#endif
					const std::set<std::size_t> indices = TNTCollisionSolverChain::getIndices(componentContacts,chain);
					const ThreadTask::Ptr chainTask = ownedPtr(new ChainTask(this,*_bc,chain,indices,_tntstate,_info));
					addSubTask(chainTask);
				} catch(const Exception& e) {
					registerFailure(e);
				}
			}
		}
	}

	void done() {
		if (_decomposed)
			delete _bc;
	}

private:
	const std::vector<const TNTContact*> _contacts;
	const TNTBodyConstraintManager* const _bc;
	TNTIslandState& _tntstate;
	struct SharedInfo _info;
	const bool _decomposed;
};

TNTCollisionSolverChain::TNTCollisionSolverChain():
	_solver(new TNTCollisionSolverSingle())
{
}

TNTCollisionSolverChain::~TNTCollisionSolverChain() {
	delete _solver;
}

void TNTCollisionSolverChain::doCollisions(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& bc,
	const TNTMaterialMap* const map,
	TNTIslandState& tntstate,
	const State& rwstate,
	const PropertyMap& pmap,
	ThreadTask::Ptr task) const
{
	SharedInfo info(this,pmap,map,rwstate);
	const DecomposeTask::Ptr mainTask = ownedPtr(new DecomposeTask(task,contacts,&bc,tntstate,info));
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

TNTCollisionSolverChain::Chain TNTCollisionSolverChain::constructChain(
	const std::vector<const TNTContact*>& contacts,
	const TNTBodyConstraintManager& component,
	const TNTIslandState& tntstate)
{
	Chain res;
	std::list<const TNTBody*> chain;

	// Now one body is inserted in the chain at a time
	std::set<const TNTRigidBody*> handled;
	std::stack<const TNTRigidBody*> queue;
	queue.push(component.getDynamicBodies().front());
	while (!queue.empty()) {
		const TNTRigidBody* const current = queue.top();
		queue.pop();

		// If the body has already been handled skip it
		if (handled.find(current) != handled.end())
			continue;
		handled.insert(current);

		// The set of connected bodies are found
		const TNTBodyConstraintManager::ConstraintListConst constraints = component.getConstraints(current,tntstate);
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
			else if (current == bodyB)
				other.insert(bodyA);
			else
				RW_THROW("TNTCollisionSolverChain (constructChain): fatal error - constraint is not attached to body as expected.");
		}

		// We extract the two connected bodies or throw an error if there are more (except if they are all not rigid bodies)
		const TNTBody* bodyA = NULL;
		const TNTBody* bodyB = NULL;
		const TNTRigidBody* rBodyA = NULL;
		const TNTRigidBody* rBodyB = NULL;
		std::set<const TNTFixedBody*> fixed;
		{
			std::size_t rI = 0;
			for (std::set<const TNTBody*>::iterator it = other.begin(); it != other.end(); it++) {
				const TNTFixedBody* const fBody = dynamic_cast<const TNTFixedBody*>(*it);
				if (fBody) {
					fixed.insert(fBody);
				} else {
					const TNTRigidBody* const rBody = dynamic_cast<const TNTRigidBody*>(*it);
					if (rI == 0) {
						rBodyA = rBody;
						bodyA = *it;
					} else if (rI == 1) {
						rBodyB = rBody;
						bodyB = *it;
					} else {
						RW_THROW("TNTCollisionSolverChain (constructChain): Can not solve for bodies not connected in chain.");
					}
					rI++;
				}
			}
		}
		RW_ASSERT(bodyB == NULL || (bodyB != NULL && bodyA != NULL));

		// If bodies are rigid, add them to the queue to be checked for neighbours later
		if (rBodyA)
			queue.push(rBodyA);
		if (rBodyB)
			queue.push(rBodyB);

		// If the this is the first body, insert this body directly.
		if (chain.size() == 0) {
			chain.push_front(current);
		}

		// Search for this body in the chain
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

		// Search for the neighbours in the chain
		std::list<const TNTBody*>::reverse_iterator itPrev = itThisRev;
		std::list<const TNTBody*>::iterator itNext = itThis;
		itPrev++;
		itNext++;

		if (fixed.size() > 0) {
			if (bodyB != NULL)
				RW_THROW("TNTCollisionSolverChain (constructChain): Can not solve for bodies not connected in chain.");

			if (itPrev == chain.rend() && itNext == chain.end()) {
				// Order does not matter (there is only one body in the chain)
				RW_ASSERT(res.fixedBodiesEnd.size() == 0);
				res.fixedBodiesEnd = fixed;
				if (bodyA != NULL)
					chain.push_front(bodyA);
			} else if (itPrev != chain.rend()) {
				RW_ASSERT(res.fixedBodiesEnd.size() == 0);
				res.fixedBodiesEnd = fixed;
				if (bodyA != NULL)
					RW_ASSERT(*itPrev == bodyA);
			} else if (itNext != chain.end()) {
				RW_ASSERT(res.fixedBodiesBegin.size() == 0);
				res.fixedBodiesBegin = fixed;
				if (bodyA != NULL)
					RW_ASSERT(*itNext == bodyA);
			}
		} else {
			// Find which body is in the front and which is in the back (or both)
			const TNTBody* body1 = NULL;
			const TNTBody* body2 = NULL;
			if (itPrev == chain.rend() && itNext == chain.end()) {
				// Order does not matter (there is only one body in the chain)
				RW_ASSERT(res.fixedBodiesBegin.size() == 0);
				RW_ASSERT(res.fixedBodiesEnd.size() == 0);
				body1 = bodyA;
				body2 = bodyB;
			} else if (itPrev != chain.rend()) {
				if (*itPrev == bodyA)
					body2 = bodyB;
				else
					body2 = bodyA;
			} else if (itNext != chain.end()) {
				if (*itNext == bodyA)
					body1 = bodyB;
				else
					body1 = bodyA;
			}

			// Insert into chain at correct places (only at ends!) and do extra sanity check that neighbours are valid
			if (body1 != NULL) {
				if (itPrev != chain.rend()) {
					RW_ASSERT(*itPrev == bodyA || *itPrev == bodyB);
				} else {
					RW_ASSERT(res.fixedBodiesBegin.size() == 0);
					chain.push_front(body1);
				}
			}
			if (body2 != NULL) {
				if (itNext != chain.end()) {
					RW_ASSERT(*itNext == bodyA || *itNext == bodyB);
				} else {
					RW_ASSERT(res.fixedBodiesEnd.size() == 0);
					chain.push_back(body2);
				}
			}
		}
	}
	RW_ASSERT(handled.size() == component.getDynamicBodies().size());

	res.bodies = std::vector<const TNTBody*>(chain.size(),NULL);
	std::size_t i = 0;
	BOOST_FOREACH(const TNTBody* const body, chain) {
		res.bodies[i] = body;
		i++;
	}

	RW_ASSERT(chain.size() >= 2 || (chain.size() == 1 && res.fixedBodiesBegin.size() == 0 && res.fixedBodiesEnd.size() > 0));
	return res;
}

std::set<std::size_t> TNTCollisionSolverChain::getIndices(
	const std::vector<const TNTContact*>& contacts,
	const Chain& chain)
{
	bool equal = true;
	std::set<std::size_t> indices;
	BOOST_FOREACH(const TNTContact* const contact, contacts) {
		const TNTBody* const bodyA = contact->getParent();
		const TNTBody* const bodyB = contact->getChild();
		// Search
		std::size_t i = 0;
		bool found = false;
		BOOST_FOREACH(const TNTFixedBody* const body, chain.fixedBodiesBegin) {
			const TNTBody* const bodyNext = chain.bodies.front();
			if (body == bodyA && bodyNext == bodyB) {
				found = true;
				break;
			}
			if (body == bodyB && bodyNext == bodyA) {
				found = true;
				break;
			}
		}
		if (!found) {
			for (i = 0; i < chain.bodies.size()-1; i++) {
				const TNTBody* const body = chain.bodies[i];
				const TNTBody* const bodyNext = chain.bodies[i+1];
				if (body == bodyA && bodyNext == bodyB) {
					found = true;
					break;
				}
				if (body == bodyB && bodyNext == bodyA) {
					found = true;
					break;
				}
			}
		}
		if (!found) {
			BOOST_FOREACH(const TNTFixedBody* const body, chain.fixedBodiesEnd) {
				const TNTBody* const bodyNext = chain.bodies.back();
				if (body == bodyA && bodyNext == bodyB) {
					found = true;
					break;
				}
				if (body == bodyB && bodyNext == bodyA) {
					found = true;
					break;
				}
			}
			i++;
		}
		if (!found)
			RW_THROW("TNTCollisionSolverChain (getIndices): could not find the contact in the chain.");
		bool eq = i%2 == 0;
		if (indices.size() == 0) {
			equal = eq;
			indices.insert(i);
		} else {
			if (equal != eq)
				RW_THROW("TNTCollisionSolverChain (getIndices): only every second object pair in chain can have an impulse.");
			else {
				indices.insert(i);
			}
		}
	}
	return indices;
}

void TNTCollisionSolverChain::addDefaultProperties(PropertyMap& map) const {
	_solver->addDefaultProperties(map);
	map.add<double>(PROPERTY_PROPTHRES_CONTACT,"Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).",1e-4);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,"Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).",1e-8);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,"Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).",1e-8);
	map.add<int>(PROPERTY_MAX_ITERATIONS,"If impulses are still propagating after this number of iterations, an exception is thrown.",1000);
}
