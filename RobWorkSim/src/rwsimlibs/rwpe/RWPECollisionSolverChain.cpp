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

#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadTask.hpp>

#include <rwsim/dynamics/Body.hpp>

#include <boost/thread/mutex.hpp>

#include <stack>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPECollisionSolverChain.hpp"

#include "RWPEBodyDynamic.hpp"
#include "RWPEBodyFixed.hpp"
#include "RWPECollisionSolverSingle.hpp"
#include "RWPEContact.hpp"
#include "RWPELogUtil.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::rwpe;

#define PROPERTY_PROPTHRES_CONTACT "RWPECollisionSolverPropagateThresholdContact"
#define PROPERTY_PROPTHRES_CONSTRAINTLIN "RWPECollisionSolverPropagateThresholdConstraintLinear"
#define PROPERTY_PROPTHRES_CONSTRAINTANG "RWPECollisionSolverPropagateThresholdConstraintAngular"
#define PROPERTY_MAX_ITERATIONS "RWPECollisionSolverMaxIterations"

struct RWPECollisionSolverChain::SharedInfo {
	SharedInfo(const RWPECollisionSolverChain* solver, const PropertyMap& pmap, const RWPEMaterialMap* const map, const State& rwstate):
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

	const RWPECollisionSolverChain* const solver;
	const PropertyMap& pmap;
	const RWPEMaterialMap* const map;
	const State& rwstate;

	double thresholdContact;
	double thresholdConstraintLin;
	double thresholdConstraintAng;
	int maxIterations;
};

class RWPECollisionSolverChain::ChainTask: public ThreadTask {
public:
	ChainTask(ThreadTask::Ptr parent,
			const RWPEBodyConstraintGraph& component,
			const Chain chain,
			const std::set<std::size_t>& indices,
			RWPEIslandState& islandState,
			const SharedInfo& info,
			RWPELogUtil* log):
				ThreadTask(parent),
				_component(component),_chain(chain),_indices(indices),_islandState(islandState),_info(info),
				_iterations(0),_runningTasks(0),
				_log(log),_doLog((log == NULL)?false:log->doLog())
{
		setKeepAlive(true);
}

	void solveChain(const std::set<std::size_t>& indices) {
		const bool doLog = (_log == NULL)? false : _log->doLog();
		BOOST_FOREACH(const std::size_t index, indices) {
			RW_ASSERT(index <= _chain.bodies.size());
			RWPELogUtil* const sublog = doLog? _log->parallel("Single Collision",RWPE_LOCATION) : NULL;
			if (index == 0 || index == _chain.bodies.size()) {
				const RWPEBodyDynamic* rigidBody;
				std::vector<const RWPEBodyFixed*> fixedBodies;
				if (index == 0) {
					rigidBody = dynamic_cast<const RWPEBodyDynamic*>(_chain.bodies.front());
					RW_ASSERT(rigidBody != NULL);
					fixedBodies.insert(fixedBodies.begin(),_chain.fixedBodiesBegin.begin(),_chain.fixedBodiesBegin.end());
				} else {
					rigidBody = dynamic_cast<const RWPEBodyDynamic*>(_chain.bodies.back());
					RW_ASSERT(rigidBody != NULL);
					fixedBodies.insert(fixedBodies.begin(),_chain.fixedBodiesEnd.begin(),_chain.fixedBodiesEnd.end());
				}
				if (doLog) {
					std::ostream& lstr = _log->log(RWPE_LOCATION);
					lstr << "Solving for impulses between:";
					if (index == 0) {
						lstr << " {";
						BOOST_FOREACH(const RWPEBody* const body, _chain.fixedBodiesBegin) {
							lstr << " " << body->get()->getName();
						}
						lstr << " } ";
						BOOST_FOREACH(const RWPEBody* const body, _chain.bodies) {
							lstr << " " << body->get()->getName();
						}
					} else {
						BOOST_FOREACH(const RWPEBody* const body, _chain.bodies) {
							lstr << " " << body->get()->getName();
						}
						lstr << " {";
						BOOST_FOREACH(const RWPEBody* const body, _chain.fixedBodiesEnd) {
							lstr << " " << body->get()->getName();
						}
						lstr << " } ";
					}

					std::map<std::string,VelocityScrew6D<> > velMap;
					velMap[rigidBody->get()->getName()] = rigidBody->getVelocityW(_info.rwstate,_islandState);
					_log->addVelocities("Velocity before solving",velMap,RWPE_LOCATION);
				}

				_info.solver->_solver->doCollisions(rigidBody,fixedBodies,_component,_info.map,_islandState,_info.rwstate,_info.pmap,sublog,this);
			} else {
				const RWPEBody* const bodyA = _chain.bodies[index-1];
				const RWPEBody* const bodyB = _chain.bodies[index];

				if (_doLog) {
					_log->log(RWPE_LOCATION) << "Solving for impulses between " << bodyA->get()->getName() << " and " << bodyB->get()->getName() << ".";
					std::map<std::string,VelocityScrew6D<> > velMap;
					velMap[bodyA->get()->getName()] = bodyA->getVelocityW(_info.rwstate,_islandState);
					velMap[bodyB->get()->getName()] = bodyB->getVelocityW(_info.rwstate,_islandState);
					_log->addVelocities("Velocities before solving",velMap,RWPE_LOCATION);
				}
				_info.solver->_solver->doCollisions(bodyA,bodyB,_component,_info.map,_islandState,_info.rwstate,_info.pmap,sublog,this);
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

	bool shouldSolve(const RWPEConstraint* const constraint) {
		const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint);
		const VelocityScrew6D<>& velI = constraint->getVelocityParentW(_islandState,_info.rwstate);
		const VelocityScrew6D<>& velJ = constraint->getVelocityChildW(_islandState,_info.rwstate);
		const Vector3D<> linRelVel = velI.linear()-velJ.linear();
		const Vector3D<> angRelVel = velI.angular().axis()*velI.angular().angle()-velJ.angular().axis()*velJ.angular().angle();
		if (contact) {
			const Vector3D<> nij = contact->getNormalW(_islandState);
			return dot(-linRelVel,nij) < -_info.thresholdContact;
		} else {
			if (constraint->getDimVelocity() == 0)
				return false;
			const std::vector<RWPEConstraint::Mode> modes = constraint->getConstraintModes();
			Vector3D<> linRelVelConstraint = Vector3D<>::zero();
			Vector3D<> angRelVelConstraint = Vector3D<>::zero();
			for (std::size_t modeI = 0; modeI < 6; modeI++) {
				if (modes[modeI] == RWPEConstraint::Velocity) {
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

		if (_doLog) {
			std::map<std::string,VelocityScrew6D<> > velMap;
			BOOST_FOREACH(const std::size_t index, _indices) {
				RW_ASSERT(index <= _chain.bodies.size());
				if (index == 0 || index == _chain.bodies.size()) {
					const RWPEBody* body;
					if (index == 0)
						body = _chain.bodies.front();
					else
						body = _chain.bodies.back();
					velMap[body->get()->getName()] = body->getVelocityW(_info.rwstate,_islandState);
				} else {
					const RWPEBody* const bodyA = _chain.bodies[index-1];
					const RWPEBody* const bodyB = _chain.bodies[index];
					velMap[bodyA->get()->getName()] = bodyA->getVelocityW(_info.rwstate,_islandState);
					velMap[bodyB->get()->getName()] = bodyB->getVelocityW(_info.rwstate,_islandState);
				}
			}
			_log->addVelocities("Velocities after solving",velMap,RWPE_LOCATION);
		}

		_iterations++;
		if ((int)_iterations == _info.maxIterations) {
			try {
				_log->log(RWPE_LOCATION) << "RWPECollisionSolverChain::ChainTask (idle): maximum number of iterations reached: " << _info.maxIterations;
				RW_THROW("RWPECollisionSolverChain::ChainTask (idle): maximum number of iterations reached: " << _info.maxIterations);
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
			const RWPEBodyDynamic* const rigidBody = dynamic_cast<const RWPEBodyDynamic*>(_chain.bodies.front());
			RW_ASSERT(_chain.fixedBodiesBegin.size() == 0 || rigidBody != NULL);
			BOOST_FOREACH(const RWPEBodyFixed* const fBody, _chain.fixedBodiesBegin) {
				const RWPEBodyConstraintGraph::ConstraintListConst constraints = _component.getConstraints(rigidBody,fBody,_islandState);
				BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
					if (shouldSolve(constraint)) {
						newIndices.insert(0);
						break;
					}
				}
			}
		}
		for (std::size_t i = start; i < _chain.bodies.size(); i += 2) {
			const RWPEBody* const bodyA = _chain.bodies[i-1];
			const RWPEBody* const bodyB = _chain.bodies[i];
			const RWPEBodyConstraintGraph::ConstraintListConst constraints = _component.getConstraints(bodyA,bodyB,_islandState);
			BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
				if (shouldSolve(constraint)) {
					newIndices.insert(i);
					break;
				}
			}
		}
		if (_chain.bodies.size()%2 == start%2) {
			const RWPEBodyDynamic* const rigidBody = dynamic_cast<const RWPEBodyDynamic*>(_chain.bodies.back());
			//RW_ASSERT(_chain.fixedBodiesBegin.size() == 0 || rigidBody != NULL); // squeeze between non-dynamic bodies
			BOOST_FOREACH(const RWPEBodyFixed* const fBody, _chain.fixedBodiesEnd) {
				const RWPEBodyConstraintGraph::ConstraintListConst constraints = _component.getConstraints(rigidBody,fBody,_islandState);
				BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
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
	const RWPEBodyConstraintGraph& _component;
	const Chain _chain;
	std::set<std::size_t> _indices;
	RWPEIslandState& _islandState;
	const SharedInfo& _info;
	std::size_t _iterations;
	std::size_t _runningTasks;
	boost::mutex _mutex;
	RWPELogUtil* const _log;
	const bool _doLog;
};

class RWPECollisionSolverChain::DecomposeTask: public ThreadTask {
public:
	typedef rw::common::Ptr<DecomposeTask> Ptr;

	DecomposeTask(ThreadTask::Ptr parent,
			const std::vector<const RWPEContact*>& contacts,
			const RWPEBodyConstraintGraph* bc,
			RWPEIslandState& islandState,
			const SharedInfo& info,
			RWPELogUtil* log,
			bool decomposed = false):
				ThreadTask(parent),
				_contacts(contacts),_bc(bc),_islandState(islandState),_info(info),
				_decomposed(decomposed), _log(log)
{
}

	void run() {
		const bool doLog = (_log == NULL)? false : _log->doLog();
		if (!_decomposed) {
			// First load the properties
			_info.extractProperties();

			// Connected dynamic body components are found.
			const std::set<RWPEBodyConstraintGraph*> components = _bc->getDynamicComponents(_islandState);
			if (doLog) {
				_log->beginSection("Collision Decomposition",RWPE_LOCATION);
				std::vector<std::string> labels;
				std::vector<double> values;
				labels.push_back("Components");
				labels.push_back("Bouncing Contacts");
				values.push_back(components.size());
				values.push_back(_contacts.size());
				_log->addValues("Found components",values,labels,RWPE_LOCATION);
				BOOST_FOREACH(const RWPEBodyConstraintGraph* const bc, components) {
					_log->addPositions("Component",bc,_info.rwstate,RWPE_LOCATION);
				}
				_log->endSection(__LINE__);
			}

			// Handle each component in parallel
			BOOST_FOREACH(const RWPEBodyConstraintGraph* component, components) {
				// If there are any initiating contacts in the component, a new subtask is forked for handling this
				RWPELogUtil* const sublog = doLog? _log->parallel("Construct Chain",RWPE_LOCATION) : NULL;
				const ThreadTask::Ptr subtask = ownedPtr(new DecomposeTask(this,_contacts,component,_islandState,_info,sublog,true));
				addSubTask(subtask);
			}
		} else {
			// First find the initiating contacts for this component
			std::vector<const RWPEContact*> componentContacts;
			BOOST_FOREACH(const RWPEContact* const contact, _contacts) {
				BOOST_FOREACH(const RWPEConstraint* const constraint, _bc->getPersistentConstraints()) {
					if (constraint == contact) {
						componentContacts.push_back(contact);
						break;
					}
				}
			}
			if (componentContacts.size() > 0) {
				try {
					// Construct chain and find the indicies for the initial collisions in the chain
					const Chain chain = RWPECollisionSolverChain::constructChain(componentContacts,*_bc,_islandState);
					if (doLog) {
						std::ostream& lstr = _log->log(RWPE_LOCATION);
						lstr << "Handling chain with bodies: {";
						BOOST_FOREACH(const RWPEBody* const body, chain.fixedBodiesBegin) {
							lstr << " " << body->get()->getName();
						}
						lstr << " } ";
						BOOST_FOREACH(const RWPEBody* const body, chain.bodies) {
							lstr << " " << body->get()->getName();
						}
						lstr << " { ";
						BOOST_FOREACH(const RWPEBody* const body, chain.fixedBodiesEnd) {
							lstr << " " << body->get()->getName();
						}
						lstr << " } ";
					}
					const std::set<std::size_t> indices = RWPECollisionSolverChain::getIndices(componentContacts,chain);
					const ThreadTask::Ptr chainTask = ownedPtr(new ChainTask(this,*_bc,chain,indices,_islandState,_info,_log));
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
	const std::vector<const RWPEContact*> _contacts;
	const RWPEBodyConstraintGraph* const _bc;
	RWPEIslandState& _islandState;
	struct SharedInfo _info;
	const bool _decomposed;
	RWPELogUtil* const _log;
};

RWPECollisionSolverChain::RWPECollisionSolverChain():
	_solver(new RWPECollisionSolverSingle())
{
}

RWPECollisionSolverChain::~RWPECollisionSolverChain() {
	delete _solver;
}

void RWPECollisionSolverChain::doCollisions(
	const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& bc,
	const RWPEMaterialMap* const map,
	RWPEIslandState& islandState,
	const State& rwstate,
	const PropertyMap& pmap,
	RWPELogUtil* log,
	ThreadTask::Ptr task) const
{
	const SharedInfo info(this,pmap,map,rwstate);
	RWPELogUtil* taskLog = NULL;
	if (log != NULL) {
		taskLog = log->parallel("Find Components",RWPE_LOCATION);
	}
	const DecomposeTask::Ptr mainTask = ownedPtr(new DecomposeTask(task,contacts,&bc,islandState,info,taskLog));
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

RWPECollisionSolverChain::Chain RWPECollisionSolverChain::constructChain(
	const std::vector<const RWPEContact*>& contacts,
	const RWPEBodyConstraintGraph& component,
	const RWPEIslandState& islandState)
{
	Chain res;
	std::list<const RWPEBody*> chain;

	// Now one body is inserted in the chain at a time
	std::set<const RWPEBodyDynamic*> handled;
	std::stack<const RWPEBodyDynamic*> queue;
	queue.push(component.getDynamicBodies().front());
	while (!queue.empty()) {
		const RWPEBodyDynamic* const current = queue.top();
		queue.pop();

		// If the body has already been handled skip it
		if (handled.find(current) != handled.end())
			continue;
		handled.insert(current);

		// The set of connected bodies are found
		const RWPEBodyConstraintGraph::ConstraintListConst constraints = component.getConstraints(current,islandState);
		std::set<const RWPEBody*> other;
		BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
			if (constraint->getDimVelocity() == 0)
				continue;
			const RWPEBody* const bodyA = constraint->getParent();
			const RWPEBody* const bodyB = constraint->getChild();
			RW_ASSERT(bodyA != NULL);
			RW_ASSERT(bodyB != NULL);
			if (current == bodyA)
				other.insert(bodyB);
			else if (current == bodyB)
				other.insert(bodyA);
			else
				RW_THROW("RWPECollisionSolverChain (constructChain): fatal error - constraint is not attached to body as expected.");
		}

		// We extract the two connected bodies or throw an error if there are more (except if they are all not rigid bodies)
		const RWPEBody* bodyA = NULL;
		const RWPEBody* bodyB = NULL;
		const RWPEBodyDynamic* rBodyA = NULL;
		const RWPEBodyDynamic* rBodyB = NULL;
		std::set<const RWPEBodyFixed*> fixed;
		{
			std::size_t rI = 0;
			for (std::set<const RWPEBody*>::iterator it = other.begin(); it != other.end(); it++) {
				const RWPEBodyFixed* const fBody = dynamic_cast<const RWPEBodyFixed*>(*it);
				if (fBody) {
					fixed.insert(fBody);
				} else {
					const RWPEBodyDynamic* const rBody = dynamic_cast<const RWPEBodyDynamic*>(*it);
					if (rI == 0) {
						rBodyA = rBody;
						bodyA = *it;
					} else if (rI == 1) {
						rBodyB = rBody;
						bodyB = *it;
					} else {
						RW_THROW("RWPECollisionSolverChain (constructChain): Can not solve for bodies not connected in chain.");
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
		std::list<const RWPEBody*>::iterator itThis;
		std::list<const RWPEBody*>::reverse_iterator itThisRev;
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
		std::list<const RWPEBody*>::reverse_iterator itPrev = itThisRev;
		std::list<const RWPEBody*>::iterator itNext = itThis;
		itPrev++;
		itNext++;

		if (fixed.size() > 0) {
			if (bodyB != NULL)
				RW_THROW("RWPECollisionSolverChain (constructChain): Can not solve for bodies not connected in chain.");

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
			const RWPEBody* body1 = NULL;
			const RWPEBody* body2 = NULL;
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

	res.bodies = std::vector<const RWPEBody*>(chain.size(),NULL);
	std::size_t i = 0;
	BOOST_FOREACH(const RWPEBody* const body, chain) {
		res.bodies[i] = body;
		i++;
	}

	RW_ASSERT(chain.size() >= 2 || (chain.size() == 1 && res.fixedBodiesBegin.size() == 0 && res.fixedBodiesEnd.size() > 0));
	return res;
}

std::set<std::size_t> RWPECollisionSolverChain::getIndices(
	const std::vector<const RWPEContact*>& contacts,
	const Chain& chain)
{
	bool equal = true;
	std::set<std::size_t> indices;
	BOOST_FOREACH(const RWPEContact* const contact, contacts) {
		const RWPEBody* const bodyA = contact->getParent();
		const RWPEBody* const bodyB = contact->getChild();
		// Search
		std::size_t i = 0;
		bool found = false;
		BOOST_FOREACH(const RWPEBodyFixed* const body, chain.fixedBodiesBegin) {
			const RWPEBody* const bodyNext = chain.bodies.front();
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
			for (i = 1; i < chain.bodies.size(); i++) {
				const RWPEBody* const body = chain.bodies[i-1];
				const RWPEBody* const bodyNext = chain.bodies[i];
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
			BOOST_FOREACH(const RWPEBodyFixed* const body, chain.fixedBodiesEnd) {
				const RWPEBody* const bodyNext = chain.bodies.back();
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
		if (!found)
			RW_THROW("RWPECollisionSolverChain (getIndices): could not find the contact in the chain.");
		bool eq = i%2 == 0;
		if (indices.size() == 0) {
			equal = eq;
			indices.insert(i);
		} else {
			if (equal != eq)
				RW_THROW("RWPECollisionSolverChain (getIndices): only every second object pair in chain can have an impulse.");
			else {
				indices.insert(i);
			}
		}
	}
	return indices;
}

void RWPECollisionSolverChain::addDefaultProperties(PropertyMap& map) const {
	_solver->addDefaultProperties(map);
	map.add<double>(PROPERTY_PROPTHRES_CONTACT,"Continue propagating impulses as long as there are contacts in chain with relative normal velocities greater than this threshold (m/s).",1e-4);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTLIN,"Continue propagating impulses as long as there are constraints in chain with relative linear velocities greater than this threshold (m/s).",1e-8);
	map.add<double>(PROPERTY_PROPTHRES_CONSTRAINTANG,"Continue propagating impulses as long as there are constraints in chain with relative angular velocities greater than this threshold (rad/s).",1e-8);
	map.add<int>(PROPERTY_MAX_ITERATIONS,"If impulses are still propagating after this number of iterations, an exception is thrown.",1000);
}
