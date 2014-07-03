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

#include "TNTIsland.hpp"
#include "TNTBroadPhase.hpp"
#include "TNTBodyConstraintManager.hpp"
#include "TNTConstraintCorrection.hpp"
#include "TNTIslandState.hpp"
#include "TNTMaterialMap.hpp"

#include "TNTCollisionSolver.hpp"
#include "TNTContact.hpp"
#include "TNTRollbackMethod.hpp"
#include "TNTSettings.hpp"
#include "TNTSolver.hpp"
#include "TNTContactResolver.hpp"
#include "TNTUtil.hpp"
#include "TNTRigidBody.hpp"

#include <rw/common/ThreadTask.hpp>
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::drawable;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsimlibs::tntphysics;

class TNTIsland::StepTask: public ThreadTask {
public:
	StepTask(TNTIsland* engine, double dt, const State &state, ThreadTask::Ptr parent):
		ThreadTask(parent),
		_engine(engine),
		_dt(dt),
		_state(state)
	{
	}

	void run() {
		while(_engine->_state->getRepetitions() > 0)
			_engine->doStep(_dt,_state);
	}

private:
	TNTIsland* const _engine;
	const double _dt;
	State _state;
};

struct TNTIsland::IntegrateSample {
	IntegrateSample():
		time(0)
	{
	}

	IntegrateSample(double time, const TNTIslandState& tntstate, const rw::kinematics::State& rwstate):
		time(time),
		tntstate(tntstate),
		rwstate(rwstate),
		forwardTrack(tntstate.getContactsTracking()),
		forwardContacts(tntstate.getContacts())
	{
	}

	double time;
	TNTIslandState tntstate;
	State rwstate;
	ContactDetectorTracking forwardTrack;
	ContactDetectorTracking backwardTrack;
	std::vector<Contact> forwardContacts;
	std::vector<Contact> backwardContacts;
};

bool TNTIsland::IntegrateSampleCompare::operator()(const IntegrateSample& s1, const IntegrateSample& s2) const {
	if (s1.time != s2.time)
		return s1.time < s2.time;
	return false;
}

PropertyMap TNTIsland::getDefaultPropertyMap() {
	PropertyMap map;
	map.add<std::string>("TNTCollisionSolver","Default collision solver.","Chain");
	map.add<std::string>("TNTSolver","Default constraint solver.","SVD");
	map.add<std::string>("TNTRollbackMethod","Default constraint solver.","Ridder");
	map.add<std::string>("TNTContactResolver","Default contact resolver.","Heuristic");
	return map;
}

TNTIsland::TNTIsland():
	_correction(new TNTConstraintCorrection()),
	_dwc(NULL),
	_materialMap(NULL),
	_bp(NULL),
	_detector(NULL),
	_bc(NULL),
	_state(NULL),
	_defaultMap(getDefaultPropertyMap())
{
}

TNTIsland::TNTIsland(rw::common::Ptr<ContactDetector> detector):
	_correction(new TNTConstraintCorrection()),
	_dwc(NULL),
	_materialMap(NULL),
	_bp(NULL),
	_detector(detector),
	_bc(NULL),
	_state(NULL),
	_defaultMap(getDefaultPropertyMap())
{
}

TNTIsland::TNTIsland(rw::common::Ptr<DynamicWorkCell> dwc, rw::common::Ptr<ContactDetector> detector):
	_correction(new TNTConstraintCorrection()),
	_dwc(dwc),
	_materialMap(new TNTMaterialMap(dwc->getContactData(),dwc->getMaterialData())),
	_bp(NULL),
	_detector(detector == NULL ? ContactDetector::makeDefault(dwc->getWorkcell()) : detector),
	_bc(NULL),
	_state(NULL),
	_gravity(dwc->getGravity()),
	_map(dwc->getEngineSettings()),
	_defaultMap(getDefaultPropertyMap())
{
}

TNTIsland::~TNTIsland() {
	delete _correction;
	delete _materialMap;
	if (_bc != NULL)
		exitPhysics();
}

const Vector3D<>& TNTIsland::getGravity() const {
	return _gravity;
}

void TNTIsland::setGravity(const Vector3D<> &gravity) {
	_gravity = gravity;
}

void TNTIsland::step(double dt, State& state, rw::common::Ptr<ThreadTask> task) {
	if (task == NULL) {
		while(_state->getRepetitions() > 0) {
			TNT_TIMING("Complete step",	doStep(dt,state));
		}
	} else {
		_task = ownedPtr(new StepTask(this,dt,state,task));
		task->addSubTask(_task);
	}
}

TNTIslandState TNTIsland::getState() const {
	return *_state;
}

void TNTIsland::resetScene(const TNTIslandState &state) {
	*_state = state;
}

bool TNTIsland::setContactDetector(rw::common::Ptr<ContactDetector> detector) {
	_detector = detector;
	if (_bp != NULL)
		_detector->setProximityFilterStrategy(_bp->getProximityFilterStrategy());
	TNT_DEBUG_CONTACTS("Contact detector set:" << std::endl << _detector);
	return true;
}

void TNTIsland::load(rw::common::Ptr<DynamicWorkCell> dwc) {
	if (!(_dwc == NULL))
		RW_THROW("TNTIsland (load): dynamic workcell has already been loaded!");
	RW_ASSERT(dwc != NULL);
	_dwc = dwc;
	_materialMap = new TNTMaterialMap(dwc->getContactData(),dwc->getMaterialData());
	_detector = ContactDetector::makeDefault(dwc->getWorkcell());
	TNT_DEBUG_CONTACTS("Default contact detector constructed based on dynamic workcell:" << std::endl << _detector);
	_gravity = dwc->getGravity();
	_map = dwc->getEngineSettings();
}

void TNTIsland::step(double dt, State& state) {
	doStep(dt,state);
}

void TNTIsland::resetScene(State& state) {
	if (_state == NULL)
		RW_THROW("TNTIsland (resetScene): please call initPhysics first!");
	const TNTBodyConstraintManager::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const TNTBody* body, bodies) {
		body->reset(*_state,state);
	}
	const TNTBodyConstraintManager::ConstraintList constraints = _bc->getPersistentConstraints();
	BOOST_FOREACH(TNTConstraint* constraint, constraints) {
		constraint->reset(*_state,state);
	}
	BOOST_FOREACH(SimulatedController::Ptr controller, _controllers) {
		controller->reset(state);
	}
}

void TNTIsland::initPhysics(State& state) {
	if (_dwc == NULL)
		RW_THROW("TNTIsland (initPhysics): no dynamic workcell given - please use load first!");
	if (_bc != NULL)
		RW_THROW("TNTIsland (initPhysics): was called earlier!");
	_bc = new TNTBodyConstraintManager();
	if (_state != NULL)
		delete _state;
	_state = new TNTIslandState();
	if (!(_dwc == NULL)) {
		_bc->initFromDWC(_dwc);
	}
	if (_bp != NULL)
		delete _bp;
	_bp = new TNTBroadPhase(_dwc);
	const TNTBodyConstraintManager::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const TNTBody* body, bodies) {
		_bp->addObject(body->get()->getObject());
	}
	_detector->setProximityFilterStrategy(_bp->getProximityFilterStrategy());

	// Add controllers
	const DynamicWorkCell::ControllerList& controllers = _dwc->getControllers();
	BOOST_FOREACH(SimulatedController::Ptr controller, controllers){
		addController(controller);
	}

	resetScene(state);
}

void TNTIsland::exitPhysics() {
	if (_bc == NULL)
		RW_THROW("TNTIsland (exitPhysics): initPhysics has not been called!");
	delete _bc;
	_bc = NULL;
	if (_state != NULL)
		delete _state;
	_state = NULL;
	if (_bp != NULL)
		delete _bp;
	_bp = NULL;
}

double TNTIsland::getTime() {
	return _state->getTime();
}

void TNTIsland::setEnabled(Body::Ptr body, bool enabled) {
	// Ignore for now
}

void TNTIsland::setDynamicsEnabled(Body::Ptr body, bool enabled) {
	// Ignore for now
}

SimulatorDebugRender::Ptr TNTIsland::createDebugRender() {
	RW_WARN("TNTIsland (createDebugRender): no debug render implemented yet!");
	return NULL;
}

PropertyMap& TNTIsland::getPropertyMap() {
	return _map;
}

const PropertyMap& TNTIsland::getPropertyMap() const {
	return _map;
}

const PropertyMap& TNTIsland::getPropertyMapDefault() const {
	return _defaultMap;
}

void TNTIsland::emitPropertyChanged() {
}

void TNTIsland::addController(SimulatedController::Ptr controller) {
	_controllers.push_back(controller);
}

void TNTIsland::removeController(SimulatedController::Ptr controller) {
	std::vector<rwlibs::simulation::SimulatedController::Ptr>::iterator it;
	for (it = _controllers.begin(); it != _controllers.end(); it++) {
		if ((*it) == controller.get())
			it = _controllers.erase(it);
	}
}

void TNTIsland::addBody(Body::Ptr body, State &state) {
	RW_WARN("TNTIsland (addBody): body can not be added yet (" << body->getName() << ")");
}

void TNTIsland::addDevice(DynamicDevice::Ptr dev, State &state) {
	RW_WARN("TNTIsland (addDevice): device can not be added yet (" << dev->getName() << ")");
}

void TNTIsland::addSensor(SimulatedSensor::Ptr sensor, State &state) {
	const SimulatedSensor* const ssensor = sensor.get();
	if(const SimulatedTactileSensor* const tsensor = dynamic_cast<const SimulatedTactileSensor*>(ssensor) ){
        const Frame* const bframe = tsensor->getFrame();
        RW_ASSERT(bframe!=NULL);
        if(_bc->getBody(bframe) == NULL)
            RW_THROW("TNTIsland (addSensor): The frame (" << bframe->getName() << ") that the sensor is being attached to is not a body in the simulator! Did you remember to run initphysics!");
		_sensors.push_back(sensor);
    } else {
    	RW_WARN("TNTIsland (addSensor): this type of sensor can not be added yet (" << sensor->getName() << ")");
    }
}

void TNTIsland::removeSensor(SimulatedSensor::Ptr sensor) {
	// Do nothing
}

void TNTIsland::attach(Body::Ptr b1, Body::Ptr b2) {
	RW_WARN("TNTIsland (attach): attach not possible yet (" << b1->getName() << "," << b2->getName() << ")");
}

void TNTIsland::detach(Body::Ptr b1, Body::Ptr b2) {
	RW_WARN("TNTIsland (detach): detach not possible yet (" << b1->getName() << "," << b2->getName() << ")");
}

std::vector<SimulatedSensor::Ptr> TNTIsland::getSensors() {
	std::vector<SimulatedSensor::Ptr> sensors;
	return sensors;
}

void TNTIsland::doStep(double dt, State& state) {
	/*static int count = 0;
	if (_state->getTime() > 0.991468) {
		if (count > 3)
			RW_THROW("STOP");
	} else
		count = 0;
	count++;*/

	TNT_DEBUG_DELIMITER()
	TNT_DEBUG_GENERAL("Time: " << _state->getTime())

	if (_state->getContacts().size() > 0) {
		const std::size_t rawNo = TNTUtil::getMarkedContacts(_state->getContacts(),_state->getContactsTracking(),TNTUtil::MARK_RAW).size();
		const std::size_t newNo = TNTUtil::getMarkedContacts(_state->getContacts(),_state->getContactsTracking(),TNTUtil::MARK_NEW).size();
		const std::size_t remaining = _state->getContacts().size()-rawNo-newNo;
		const std::size_t tntTempConstraints = _bc->getTemporaryConstraints(_state).size();
#if TNT_DEBUG_ENABLE_CONTACTS
		TNT_DEBUG_CONTACTS("Raw: " << rawNo << " New: " << newNo << " Remaining " << remaining << " TNT: " << tntTempConstraints);
		const TNTBodyConstraintManager::ConstraintList constraints = _bc->getTemporaryConstraints(_state);
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			if (const TNTContact* const contact = dynamic_cast<const TNTContact*>(constraint)) {
				const TNTContact c = *contact;
				TNT_DEBUG_CONTACTS(" - Known: " << contact->getContact());
			}
		}
#endif
		if (remaining != tntTempConstraints)
			RW_THROW("TNTIsland (doStep): the number of temporary constraints should be the same as the number of contacts that are not either raw or new.");
	}

	if (_state->getRepetitions() == 0) {
		// Update controllers
		Simulator::UpdateInfo info;
		info.dt = dt;
		info.dt_prev = _state->getLastTimeStep();
		info.time = _state->getTime();
		info.rollback = false;
		BOOST_FOREACH(SimulatedController::Ptr controller, _controllers) {
			controller->update(info,state);
		}
	} else {
		TNT_DEBUG_GENERAL("Repetitions: " << _state->getRepetitions())
	}

	IntegrateSample sampleInitial(0,*_state,state);

	// Update constraints
	{
		TNTBodyConstraintManager::ConstraintList constraints = _bc->getConstraints(*_state);
		BOOST_FOREACH(TNTConstraint* constraint, constraints) {
			constraint->clearWrench(*_state);
			constraint->update(*_state,state);
		}
	}

	// Do impulses & find the constraint forces
	TNT_TIMING("Impulse & Constraint solver", solveConstraints(dt,*_state,state))

	ContactDetectorData cdData = _state->getContactData();
	IntegrateSample sample0(0,*_state,state);

	TNT_DEBUG_CONTACTS("Known contacts: " << sample0.forwardContacts.size());

	// Do a broad phase integration (halfes the step until there are no PQP trimesh penetrations)
	IntegrateSample sampleH;
	TNT_TIMING("Broad-phase Rollback",
			{
					const double oldDt = dt;
					dt = integrateBroadPhase(dt,sample0,sampleH,cdData);
					if (dt < oldDt)
						TNT_DEBUG_ROLLBACK("Broad-phase check decreased timestep to " << dt << " from " << oldDt << ".");
			}
	)

	// If there are new contacts, check if rollback should be performed
	bool doRollback = false;
	const std::vector<std::size_t> newContactsH = TNTUtil::getMarkedContacts(sampleH.forwardContacts,sampleH.forwardTrack,TNTUtil::MARK_RAW);
	if (newContactsH.size() > 0) {
#if TNT_DEBUG_ENABLE_CONTACTS
		TNT_DEBUG_CONTACTS("Found " << newContactsH.size() << " new contacts after step of size " << dt << ".");
		BOOST_FOREACH(std::size_t i, newContactsH) {
			TNT_DEBUG_CONTACTS(" - " << sampleH.forwardContacts[i]);
		}
#endif
		const double minDist = TNTUtil::minDistance(TNTUtil::getContacts(sampleH.forwardContacts,newContactsH));
		if (minDist > 0) {
			RW_THROW("TNTIsland (doStep): there should not be non-penetrating raw contacts at step H!");
		}
		TNTUtil::mark(sampleH.forwardContacts,sampleH.forwardTrack,TNTUtil::MARK_RAW,TNTUtil::MARK_NEW);
		if (minDist < -0.0001) {
			doRollback = true;
			// Construct backward tracking with the new contacts only
			sampleH.backwardTrack = sampleH.forwardTrack;
			sampleH.backwardContacts = sampleH.forwardContacts;
			TNTUtil::removeKnown(sampleH.backwardContacts,sampleH.backwardTrack,_bc,sampleH.tntstate);
			TNT_DEBUG_ROLLBACK("Rollback requested - " << sampleH.backwardContacts.size() << " contacts in candidate set with minimum distance " << minDist << "!");
#if TNT_DEBUG_ENABLE_CONTACTS && TNT_DEBUG_ENABLE_ROLLBACK
			BOOST_FOREACH(const Contact& c, sampleH.backwardContacts) {
				TNT_DEBUG_CONTACTS(" - " << c);
			}
#endif
		} else {
			TNT_DEBUG_ROLLBACK("Distance for new contact(s) at step size " << dt << " was low enough (" << minDist << ") that rollback is not required!");
		}
	}

	if (!doRollback) {
		// Store the results & update RobWork state
		storeResults(cdData,sampleH,state);
	} else {
		TNT_TIMING("Ordinary Rollback",
				{
						// Do a backwards tracking of the new contacts to time dt=0
						sample0.backwardTrack = sampleH.backwardTrack;
						sample0.backwardContacts = _detector->updateContacts(sample0.rwstate,cdData,sample0.backwardTrack);
						if (sample0.backwardContacts.size() == 0) {
							//RW_THROW("TNTIsland (doStep): The new contact could not be traced back to time dt = 0!");
							TNT_DEBUG_ROLLBACK("The new contact(s) could not be traced back to time dt = 0!");
							TNT_DEBUG_ROLLBACK(" - taking complete step instead.");
							storeResults(cdData,sampleH,state);
						} else {

#if TNT_DEBUG_ENABLE_ROLLBACK && TNT_DEBUG_ENABLE_CONTACTS
							TNT_DEBUG_ROLLBACK("Tracked " << sample0.backwardContacts.size() << " new contact(s) to dt = 0.");
							BOOST_FOREACH(const Contact& c, sample0.backwardContacts) {
								TNT_DEBUG_CONTACTS(" - " << c);
							}
#endif

							TNTUtil::removePenetrating(sample0.backwardContacts,sample0.backwardTrack,TNTUtil::MARK_NEW);
							TNT_DEBUG_ROLLBACK(sample0.backwardContacts.size() << " contacts left after removing penetrating contacts.");

							double minDist = 0;
							if (sample0.backwardContacts.size() > 0) {
								minDist = TNTUtil::minDistance(sample0.backwardContacts);
								if (minDist < 0.0001) {
									TNT_DEBUG_ROLLBACK("Distance at dt = 0 was low enough (" << minDist << ") that previous step is repeated with more new contacts!");
									if (_state->getRepetitions() < 5) {
										sampleInitial.forwardContacts = _detector->findContacts(sampleH.rwstate,cdData,sampleInitial.forwardTrack);
										TNTUtil::mark(sampleInitial.forwardContacts,sampleInitial.forwardTrack,TNTUtil::MARK_RAW,TNTUtil::MARK_NEW);
										sampleInitial.forwardContacts = _detector->findContacts(sampleInitial.rwstate,cdData,sampleInitial.forwardTrack);
										storeResults(cdData,sampleInitial,state);
									} else {
										TNT_DEBUG_ROLLBACK(" - as there have already been 5 repetitions there is probably an issue with tracking contacts - instead taking complete step.");
										storeResults(cdData,sampleH,state);
									}
								} else {
									// Ordinary Rollback
									TNT_DEBUG_ROLLBACK("Performing narrow-phase rollback on  " << sampleH.backwardContacts.size() << " new contact(s).");
									IntegrateSample result = integrateRollback(sample0,sampleH,cdData);
									TNT_DEBUG_ROLLBACK("Narrow-phase rollback performed with stepsize " << result.time);
									result.forwardTrack = sampleH.forwardTrack;
									result.forwardContacts = _detector->findContacts(result.rwstate,cdData,result.forwardTrack);
									storeResults(cdData,result,state);
								}
							} else {
								TNT_DEBUG_ROLLBACK("There are no contacts that are non-penetrating at dt = 0, hence rollback is impossible. Contact tracking must have failed.");
								TNT_DEBUG_ROLLBACK(" - one complete step is performed even though the penetration is too big.");
								// Tracking must have failed!
								// Store the results & update RobWork state
								storeResults(cdData,sampleH,state);
							}
						}
				}
		)
	}
	TNT_DEBUG_GENERAL("Step ended.")
	TNT_DEBUG_DELIMITER()
}

void TNTIsland::solveConstraints(double dt, TNTIslandState& tntstate, const State& rwstate) const {
	std::vector<TNTContact*> persistentContacts;

	// Bouncing contacts
	{
		// Construct list of new contacts
		std::vector<Contact> contacts = tntstate.getContacts();
		ContactDetectorTracking tracking = tntstate.getContactsTracking();
		const std::vector<std::size_t> contactIDs = TNTUtil::getMarkedContacts(contacts,tracking,TNTUtil::MARK_NEW);
		const std::vector<Contact> newContacts = TNTUtil::getContacts(contacts,contactIDs);
#if TNT_DEBUG_ENABLE_CONTACTS
		if (contactIDs.size() > 0) {
			TNT_DEBUG_CONTACTS("New contacts: " << contactIDs.size());
			BOOST_FOREACH(std::size_t i, contactIDs) {
				TNT_DEBUG_CONTACTS(" - " << contacts[i]);
			}
		}
#endif

		// Construct list of TNTContacts for doing bouncing
		std::vector<const TNTContact*> tntcontacts;
		std::vector<TNTContact*> tntcontactsLeaving;
		BOOST_FOREACH(std::size_t id, contactIDs) {
			const Contact &c = contacts[id];
			const TNTBody* const bodyA = _bc->getBody(c.getFrameA());
			const TNTBody* const bodyB = _bc->getBody(c.getFrameB());
			if (bodyA == NULL)
				RW_THROW("Could not find a TNTBody for frame \"" << c.getFrameA()->getName() << "\".");
			if (bodyB == NULL)
				RW_THROW("Could not find a TNTBody for frame \"" << c.getFrameB()->getName() << "\".");
			TNTContact* const tntcontact = new TNTContact(bodyA,bodyB,c,rwstate);
			tracking.setUserData(id,TNTUtil::TNTUserData::make(tntcontact));
			const Vector3D<> velI = tntcontact->getVelocityParentW(tntstate,rwstate).linear();
			const Vector3D<> velJ = tntcontact->getVelocityChildW(tntstate,rwstate).linear();
			const Vector3D<> nij = tntcontact->getNormalW(tntstate);
			bool leaving = -dot(velI-velJ,nij) >= 0;
			_bc->addTemporaryConstraint(tntcontact,tntstate);
			if (leaving) {
				tntcontactsLeaving.push_back(tntcontact);
			} else {
				tntcontacts.push_back(tntcontact);
			}
		}
		tntstate.setContacts(contacts,tracking);

		// Do bouncing on all known contacts
		if (tntcontacts.size() > 0) {
			TNT_DEBUG_BOUNCING("Contacts to treat as bouncing: " << tntcontacts.size());
			std::string solverID;
			if (_map.has("TNTCollisionSolver"))
				solverID = _map.get<std::string>("TNTCollisionSolver");
			else
				solverID = _defaultMap.get<std::string>("TNTCollisionSolver");
			const TNTCollisionSolver::Ptr bouncingSolver = TNTCollisionSolver::Factory::makeSolver(solverID);
			TNTMaterialMap map(_dwc->getContactData(),_dwc->getMaterialData());
			bouncingSolver->applyImpulses(tntcontacts,*_bc,&map,tntstate,rwstate);
		} else {
			TNT_DEBUG_BOUNCING("No contacts to treat as bouncing.");
		}
	}

	// Velocity correction step might be needed here to make sure that constraints and new contacts have zero velocity

	// Non-Bouncing
	{
		std::string solverID;
		if (_map.has("TNTSolver"))
			solverID = _map.get<std::string>("TNTSolver");
		else
			solverID = _defaultMap.get<std::string>("TNTSolver");
		const TNTSolver* const solver = TNTSolver::Factory::makeSolver(solverID,_bc,_gravity);
		std::string resolverID;
		if (_map.has("TNTContactResolver"))
			resolverID = _map.get<std::string>("TNTContactResolver");
		else
			resolverID = _defaultMap.get<std::string>("TNTContactResolver");
		const TNTContactResolver* const resolver = TNTContactResolver::Factory::makeResolver(resolverID,solver);
		resolver->solve(persistentContacts,dt,rwstate,tntstate);
		delete resolver;
		delete solver;
	}

	// Check size of forces
	{
		const TNTBodyConstraintManager::ConstraintList constraints = _bc->getConstraints(tntstate);
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			const double force = constraint->getWrench(tntstate).force().norm2();
			if (force > TNT_CONSTRAINT_MAX_FORCE) {
				RW_THROW("Constraint force on " << force << "N exceeded limit of " << TNT_CONSTRAINT_MAX_FORCE << "N.");
			}
		}
	}

#if TNT_DEBUG_ENABLE_SOLVER
	const TNTBodyConstraintManager::ConstraintList constraints = _bc->getTemporaryConstraints(&tntstate);
	if (constraints.size() > 0) {
		TNT_DEBUG_SOLVER("Contact Resolution");
		BOOST_FOREACH(const TNTConstraint* const constraint, constraints) {
			if (const TNTContact* const c = dynamic_cast<const TNTContact*>(constraint)) {
				const std::string linear = c->isLeaving() ? "Leaving" : c->getTypeLinear() == TNTContact::Sticking ? "Sticking" : "Sliding";
				const std::string angular = c->isLeaving() ? "Leaving" : c->getTypeAngular() == TNTContact::Sticking ? "Sticking" : "Sliding";
				const Wrench6D<> wrench = c->getWrench(tntstate);
				TNT_DEBUG_SOLVER(" - " << linear << " " << angular << " - force/torque: " << wrench.force() << " " << wrench.torque() << " - contact: " << c->getContact());
			}
		}
	} else {
		TNT_DEBUG_SOLVER("All contacts treated as leaving.");
	}
#endif
}

double TNTIsland::integrateBroadPhase(double dt, const IntegrateSample& first, IntegrateSample& res, ContactDetectorData& cdData) const {
#if TNT_DEBUG_ENABLE_INTEGRATOR
	{
		const TNTBodyConstraintManager::DynamicBodyList bodies = _bc->getDynamicBodies();
		if (bodies.size() > 0) {
			TNT_DEBUG_INTEGRATOR("Motion before integration");
			BOOST_FOREACH(const TNTRigidBody* const body, bodies) {
				const Transform3D<> pos = body->getWorldTcom(first.tntstate);
				const VelocityScrew6D<> vel = body->getVelocityW(first.rwstate,first.tntstate);
				TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " pos: " << pos.P() << " " << RPY<>(pos.R()));
				TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " vel: " << vel.linear() << " " << vel.angular());
			}
		}
	}
#endif

	bool fastCheck = true;
	for (unsigned int iteration = 1; iteration <= TNT_MAX_ITERATIONS; iteration++) {
		const double dtTry = dt/std::pow(2.,(int)(iteration-1));
		if (iteration > 1)
			TNT_DEBUG_ROLLBACK("Trying broad-phase time-step " << dtTry << " (original " << dt << ").");
		res = first;
		res.time = dtTry;

		TNT_TIMING("Integration Step", TNTUtil::step(dtTry,_gravity,_bc,res.tntstate,res.rwstate))
		if (fastCheck) {
			bool penetrating = false;
			TNT_TIMING("Penetration Check", penetrating = _bp->maxPenetrationExceeded(_detector,res.rwstate) )
			if (!penetrating)
				fastCheck = false;
		}
		if (!fastCheck) {
			TNT_TIMING("Update Contacts", res.forwardContacts = _detector->updateContacts(res.rwstate,cdData,res.forwardTrack) )
#if TNT_DEBUG_ENABLE_CONTACTS
			TNT_DEBUG_CONTACTS("Contacts after update: " << res.forwardContacts.size());
			BOOST_FOREACH(const Contact& c, res.forwardContacts) {
				TNT_DEBUG_CONTACTS(" - " << c);
			}
			if (res.forwardContacts.size() != first.forwardContacts.size()) {
				// More detailed check needed - matching algorithm
				TNT_DEBUG_CONTACTS(" - Number of contacts changed.");
				TNT_DEBUG_CONTACTS(" - Old contacts to track: " << first.forwardContacts.size());
				BOOST_FOREACH(const Contact& c, first.forwardContacts) {
					TNT_DEBUG_CONTACTS(" - " << c);
				}
			}
#endif
			TNTUtil::updateTemporaryContacts(res.forwardContacts,res.forwardTrack,_bc,res.tntstate,res.rwstate);
#if TNT_ENABLE_CONSTRAINT_CORRECTION
			TNT_TIMING("Constraint Correction", _correction->correct(_bc->getConstraints(res.tntstate),res.tntstate,res.rwstate) )
			// Update RobWork bodies
			const TNTBodyConstraintManager::BodyList bodies = _bc->getBodies();
			BOOST_FOREACH(const TNTBody* body, bodies) {
				body->updateRW(res.rwstate,res.tntstate);
			}
#endif
			if (!_bp->maxPenetrationExceeded(_detector,res.rwstate)) {
				TNT_TIMING("Find Contacts", res.forwardContacts = _detector->findContacts(res.rwstate,cdData,res.forwardTrack) )
				TNT_DEBUG_CONTACTS("Contacts after broad-phase rollback: " << res.forwardContacts.size	());
				BOOST_FOREACH(const Contact& c, res.forwardContacts) {
					TNT_DEBUG_CONTACTS(" - " << c);
				}
				dt = dtTry;
				break;
			}
		}
		if (iteration == TNT_MAX_ITERATIONS) {
			TNT_DEBUG_ROLLBACK("Maximum of " << TNT_MAX_ITERATIONS << " iterations exceeded in broad-phase rollback!");
			RW_THROW("TNTIsland (integrateBroadPhase): Maximum of " << TNT_MAX_ITERATIONS << " iterations exceeded!");
		}
	}

	const TNTBodyConstraintManager::DynamicBodyList bodies = _bc->getDynamicBodies();
	if (bodies.size() > 0) {
		TNT_DEBUG_INTEGRATOR("Motion after broad-phase integration");
		BOOST_FOREACH(const TNTRigidBody* const body, bodies) {
			const Transform3D<> pos = body->getWorldTcom(res.tntstate);
			const VelocityScrew6D<> vel = body->getVelocityW(res.rwstate,res.tntstate);
			if (pos.P().normInf() > TNT_WORKSPACE)
				RW_THROW("TNTIsland (integrateBroadPhase): position for body \"" << body->get()->getName() << "\" is " << pos.P() << " which is outisde the workspace of size " << TNT_WORKSPACE << "x" << TNT_WORKSPACE << "x" << TNT_WORKSPACE << " m - aborting.");
			if (vel.linear().norm2() > TNT_MAX_LINVEL)
				RW_THROW("TNTIsland (integrateBroadPhase): linear velocity for body \"" << body->get()->getName() << "\" is " << vel.linear().norm2() << " m/s which is bigger than the limit " << TNT_MAX_LINVEL << " m/s - aborting.");
			if (vel.angular().angle() > TNT_MAX_ANGVEL)
				RW_THROW("TNTIsland (integrateBroadPhase): angular velocity for body \"" << body->get()->getName() << "\" is " << vel.angular().angle() << " rad/s which is bigger than the limit " << TNT_MAX_ANGVEL << " rad/s - aborting.");
			TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " pos: " << pos.P() << " " << RPY<>(pos.R()));
			TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " vel: " << vel.linear() << " " << vel.angular());
		}
	}

	return dt;
}

TNTIsland::IntegrateSample TNTIsland::integrateRollback(const IntegrateSample& sample0, const IntegrateSample& sampleH, ContactDetectorData& cdData) const {
	IntegrateSample result;

	const TNTRollbackMethod::Sample rollbackSample0(sample0.time,sample0.backwardContacts);
	const TNTRollbackMethod::Sample rollbackSampleH(sampleH.time,sampleH.backwardContacts);

	std::string methodID;
	if (_map.has("TNTRollbackMethod"))
		methodID = _map.get<std::string>("TNTRollbackMethod");
	else
		methodID = _defaultMap.get<std::string>("TNTRollbackMethod");
	TNTRollbackMethod::Ptr rollback = TNTRollbackMethod::Factory::makeMethod(methodID);
	TNTRollbackMethod::RollbackData* rollbackData = rollback->createData();
	TNTRollbackMethod::SampleSet rollbackSamples;
	rollbackSamples.insert(rollbackSample0);
	rollbackSamples.insert(rollbackSampleH);

	bool doRollback = true;
	for (unsigned int iteration = 1; iteration <= TNT_MAX_ITERATIONS; iteration++) {
		RW_ASSERT(rollbackSamples.size() >= 2);
		const double dtTry = rollback->getTimestep(rollbackSamples,rollbackData);
		RW_ASSERT(dtTry > rollbackSamples.begin()->time);
		RW_ASSERT(dtTry < rollbackSamples.rbegin()->time);
		TNT_DEBUG_ROLLBACK("Iteration " << iteration << ": trying time " << dtTry);

		result = sample0;
		result.time = dtTry;
		{
			const TNTBodyConstraintManager::DynamicBodyList bodies = _bc->getDynamicBodies();
			if (bodies.size() > 0) {
				TNT_DEBUG_INTEGRATOR("Motion before step");
				BOOST_FOREACH(const TNTRigidBody* const body, bodies) {
					const Transform3D<> pos = body->getWorldTcom(result.tntstate);
					const VelocityScrew6D<> vel = body->getVelocityW(result.rwstate,result.tntstate);
					TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " pos: " << pos.P() << " " << RPY<>(pos.R()));
					TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " vel: " << vel.linear() << " " << vel.angular());
				}
			}
		}
		TNTUtil::step(dtTry,_gravity,_bc,result.tntstate,result.rwstate);
		{
			const TNTBodyConstraintManager::DynamicBodyList bodies = _bc->getDynamicBodies();
			if (bodies.size() > 0) {
				TNT_DEBUG_INTEGRATOR("Motion after step");
				BOOST_FOREACH(const TNTRigidBody* const body, bodies) {
					const Transform3D<> pos = body->getWorldTcom(result.tntstate);
					const VelocityScrew6D<> vel = body->getVelocityW(result.rwstate,result.tntstate);
					TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " pos: " << pos.P() << " " << RPY<>(pos.R()));
					TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " vel: " << vel.linear() << " " << vel.angular());
				}
			}
		}
		result.forwardContacts = _detector->updateContacts(result.rwstate,cdData,result.forwardTrack);
		TNTUtil::updateTemporaryContacts(result.forwardContacts,result.forwardTrack,_bc,result.tntstate,result.rwstate);
#if TNT_ENABLE_CONSTRAINT_CORRECTION && 0
		_correction->correct(_bc->getConstraints(result.tntstate),result.tntstate,result.rwstate);
		// Update RobWork bodies
		const TNTBodyConstraintManager::BodyList bodies = _bc->getBodies();
		BOOST_FOREACH(const TNTBody* body, bodies) {
			body->updateRW(result.rwstate,result.tntstate);
		}
#endif
		result.backwardTrack = sampleH.backwardTrack;
		result.backwardContacts = _detector->updateContacts(result.rwstate,cdData,result.backwardTrack);
		if (result.backwardContacts.size() > 0) {
			const double minDist = TNTUtil::minDistance(result.backwardContacts);
			TNT_DEBUG_ROLLBACK(" Minimum distance: " << minDist);
			if (fabs(minDist) < 0.0001) {
				TNT_DEBUG_ROLLBACK(" - distance is less than " << 0.0001 << " (" << fabs(minDist) << ") - rollback done.");
				doRollback = false;
			} else {
				if (minDist < 0)
					TNTUtil::removeNonPenetrating(result.backwardContacts,result.backwardTrack,TNTUtil::MARK_NEW);
#if TNT_DEBUG_ENABLE_ROLLBACK && TNT_DEBUG_ENABLE_CONTACTS
				TNT_DEBUG_ROLLBACK("Making rollback sample with " << result.backwardContacts.size() << " new contact(s).");
				BOOST_FOREACH(const Contact& c, result.backwardContacts) {
					TNT_DEBUG_CONTACTS(" - " << c);
				}
#endif
				const TNTRollbackMethod::Sample rollbackSample = TNTRollbackMethod::Sample(dtTry,result.backwardContacts);
				rollbackSamples.insert(rollbackSample);
			}
		} else {
			TNT_DEBUG_ROLLBACK(" - lost all contacts - rollback done.");
			doRollback = false;
		}

		if (!doRollback)
			break;

		if (iteration == TNT_MAX_ITERATIONS)
			RW_THROW("TNTIsland (doStep): Maximum of " << TNT_MAX_ITERATIONS << " iterations exceeded!");
	}
	delete rollbackData;

#if TNT_DEBUG_ENABLE_INTEGRATOR
	{
		const TNTBodyConstraintManager::DynamicBodyList bodies = _bc->getDynamicBodies();
		if (bodies.size() > 0) {
			TNT_DEBUG_INTEGRATOR("Motion after narrow-phase integration");
			BOOST_FOREACH(const TNTRigidBody* const body, bodies) {
				const Transform3D<> pos = body->getWorldTcom(result.tntstate);
				const VelocityScrew6D<> vel = body->getVelocityW(result.rwstate,result.tntstate);
				TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " pos: " << pos.P() << " " << RPY<>(pos.R()));
				TNT_DEBUG_INTEGRATOR(" - " << body->get()->getName() << " vel: " << vel.linear() << " " << vel.angular());
			}
		}
	}
#endif

	return result;
}

void TNTIsland::storeResults(ContactDetectorData& cdData, IntegrateSample& sample, State& rwstate) const {
	// Update tactile sensors
	if (sample.time > 0) {
		TNTUtil::updateSensors(_sensors,_state->getTime(),sample.time,_state->getLastTimeStep(),_bc,*_state,rwstate);
	}

	const TNTIslandState& tntstate = sample.tntstate;
	*_state = tntstate;
	_state->setLastTimeStep(sample.time);
	if (sample.time == 0)
		_state->setRepetitions(_state->getRepetitions()+1);
	else
		_state->setRepetitions(0);
	_state->setTime(_state->getTime()+sample.time);
	rwstate = sample.rwstate;

	// Update RobWork bodies
	const TNTBodyConstraintManager::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const TNTBody* body, bodies) {
		body->updateRW(rwstate,*_state);
	}

	// Keep only the new contacts for bouncing that are within a certain distance
	TNTUtil::remove(sample.forwardContacts,sample.forwardTrack,TNTUtil::MARK_RAW);
#if TNT_DEBUG_ENABLE_ROLLBACK && TNT_DEBUG_ENABLE_CONTACTS
	TNT_DEBUG_ROLLBACK("All contacts for next step: " << sample.forwardContacts.size());
	BOOST_FOREACH(const Contact& c, sample.forwardContacts) {
		TNT_DEBUG_ROLLBACK(" - " << c);
	}
#endif
	TNTUtil::removeContactsOutsideThreshold(sample.forwardContacts,sample.forwardTrack,0.0001,TNTUtil::MARK_NEW);
	TNT_DEBUG_ROLLBACK("After removing contacts outside threshold: " << sample.forwardContacts.size());

	// Update the constraints
	TNTUtil::updateTemporaryContacts(sample.forwardContacts,sample.forwardTrack,_bc,*_state,rwstate);

	// Store contact information in state
	_state->setContactData(cdData);
	_state->setContacts(sample.forwardContacts,sample.forwardTrack);
}