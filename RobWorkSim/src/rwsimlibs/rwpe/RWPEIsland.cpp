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
#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/sensor/SimulatedTactileSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>
#include "RWPEUtil.hpp"
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPECollisionSolver.hpp"
#include "RWPEConstraintCorrection.hpp"
#include "RWPEConstraintSolver.hpp"
#include "RWPEDebugRender.hpp"
#include "RWPEIsland.hpp"

#include "RWPEBroadPhase.hpp"
#include "RWPEContact.hpp"
#include "RWPEContactResolver.hpp"
#include "RWPEIslandState.hpp"
#include "RWPELogUtil.hpp"
#include "RWPEMaterialMap.hpp"
#include "RWPERollbackMethod.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::drawable;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsim::sensor;
using namespace rwsimlibs::rwpe;

class RWPEIsland::StepTask: public ThreadTask {
public:
	StepTask(RWPEIsland* engine, double dt, ThreadTask::Ptr parent):
		ThreadTask(parent),
		_engine(engine),
		_dt(dt)
	{
	}

	void run() {
		while(_engine->_state->getRepetitions() > 0)
			_engine->doStep(_dt);
	}

private:
	RWPEIsland* const _engine;
	const double _dt;
};

struct RWPEIsland::IntegrateSample {
	IntegrateSample():
		time(0)
	{
	}

	IntegrateSample(double time, const RWPEIslandState& islandState, const rw::kinematics::State& rwstate):
		time(time),
		islandState(islandState),
		rwstate(rwstate),
		forwardTrack(islandState.getContactsTracking()),
		forwardContacts(islandState.getContacts())
	{
	}

	double time;
	RWPEIslandState islandState;
	State rwstate;
	ContactDetectorTracking forwardTrack;
	ContactDetectorTracking backwardTrack;
	std::vector<Contact> forwardContacts;
	std::vector<Contact> backwardContacts;
};

bool RWPEIsland::IntegrateSampleCompare::operator()(const IntegrateSample& s1, const IntegrateSample& s2) const {
	if (s1.time != s2.time)
		return s1.time < s2.time;
	return false;
}

PropertyMap RWPEIsland::getDefaultPropertyMap() {
	PropertyMap map;
	map.add<std::string>("RWPECollisionSolver","Default collision solver.","Chain");
	RWPECollisionSolver::Factory::makeSolver("Chain")->addDefaultProperties(map);
	map.add<std::string>("RWPEConstraintSolver","Default constraint solver.","Iterative");
	const RWPEConstraintSolver* const solver = RWPEConstraintSolver::Factory::makeSolver("Iterative",NULL,Vector3D<>::zero());
	if (solver == NULL)
		RW_THROW("RWPEIsland (getDefaultPropertyMap): could not construct default RWPEConstraintSolver!");
	solver->addDefaultProperties(map);
	delete solver;
	map.add<std::string>("RWPERollbackMethod","Default constraint solver.","Ridder");
	map.add<std::string>("RWPEContactResolver","Default contact resolver.","Full");
	const RWPEContactResolver* const resolver = RWPEContactResolver::Factory::makeResolver("Full",NULL);
	if (resolver == NULL)
		RW_THROW("RWPEIsland (getDefaultPropertyMap): could not construct default RWPEContactResolver!");
	resolver->addDefaultProperties(map);
	delete resolver;
	map.add<int>("RWPECorrection","Enable or disable contact & constraint correction.",1);
	RWPEConstraintCorrection::addDefaultProperties(map);
	map.add<int>("RWPERollback","Enable or disable rollback.",1);
	map.add<double>("RWPERollbackThreshold","Precision of rollback.",1e-5);
	map.add<int>("RWPERollbackIterations","Stop with exception if rollback requires more than this number of iterations.",10);
	map.add<double>("RWPEWorkspace","The coordinates of all bodies must lie within this workspace, or simulation fails (meters).",50);
	map.add<double>("RWPEConstraintMaxForce","Stop with exception if theshold exceeds this value (Newtons).",1e6);
	return map;
}

RWPEIsland::RWPEIsland():
	_correction(new RWPEConstraintCorrection()),
	_dwc(NULL),
	_materialMap(NULL),
	_bp(NULL),
	_detector(NULL),
	_bc(NULL),
	_state(NULL),
	_render(NULL),
	_log(new RWPELogUtil()),
	_defaultMap(getDefaultPropertyMap())
{
}

RWPEIsland::RWPEIsland(rw::common::Ptr<ContactDetector> detector):
	_correction(new RWPEConstraintCorrection()),
	_dwc(NULL),
	_materialMap(NULL),
	_bp(NULL),
	_detector(detector),
	_bc(NULL),
	_state(NULL),
	_render(NULL),
	_log(new RWPELogUtil()),
	_defaultMap(getDefaultPropertyMap())
{
}

RWPEIsland::RWPEIsland(rw::common::Ptr<DynamicWorkCell> dwc, rw::common::Ptr<ContactDetector> detector):
	_correction(new RWPEConstraintCorrection()),
	_dwc(dwc),
	_materialMap(new RWPEMaterialMap(dwc->getContactData(),dwc->getMaterialData())),
	_bp(NULL),
	_detector(detector == NULL ? ContactDetector::makeDefault(dwc->getWorkcell(),dwc->getEngineSettings()) : detector),
	_bc(NULL),
	_state(NULL),
	_gravity(dwc->getGravity()),
	_render(NULL),
	_log(new RWPELogUtil()),
	_map(dwc->getEngineSettings()),
	_defaultMap(getDefaultPropertyMap())
{
}

RWPEIsland::~RWPEIsland() {
	delete _correction;
	delete _materialMap;
	if (_bc != NULL)
		exitPhysics();
	delete _log;
}

const Vector3D<>& RWPEIsland::getGravity() const {
	return _gravity;
}

void RWPEIsland::setGravity(const Vector3D<> &gravity) {
	_gravity = gravity;
}

void RWPEIsland::step(double dt, rw::common::Ptr<ThreadTask> task) {
	if (task == NULL) {
		while(_state->getRepetitions() > 0) {
			doStep(dt);
		}
	} else {
		_task = ownedPtr(new StepTask(this,dt,task));
		task->addSubTask(_task);
	}
}

RWPEIslandState RWPEIsland::getRWPEState() const {
	return *_state;
}

State RWPEIsland::getState() const {
	return _rwstate;
}

void RWPEIsland::resetScene(const RWPEIslandState &state) {
	*_state = state;
}

bool RWPEIsland::setContactDetector(rw::common::Ptr<ContactDetector> detector) {
	_detector = detector;
	if (_bp != NULL)
		_detector->setProximityFilterStrategy(_bp->getProximityFilterStrategy());
	std::cout << "Contact detector set:" << std::endl << _detector << std::endl;
	return true;
}

void RWPEIsland::load(rw::common::Ptr<DynamicWorkCell> dwc) {
	if (!(_dwc == NULL))
		RW_THROW("RWPEIsland (load): dynamic workcell has already been loaded!");
	RW_ASSERT(dwc != NULL);
	_dwc = dwc;
	_materialMap = new RWPEMaterialMap(dwc->getContactData(),dwc->getMaterialData());
	_detector = ContactDetector::makeDefault(dwc->getWorkcell(),dwc->getEngineSettings());
	std::cout << "Default contact detector constructed based on dynamic workcell:" << std::endl << _detector << std::endl;
	_gravity = dwc->getGravity();
	_map = dwc->getEngineSettings();
}

void RWPEIsland::step(double dt) {
	doStep(dt);
}

void RWPEIsland::step(double dt, State& state) {
	doStep(dt);
	state = _rwstate;
}

void RWPEIsland::resetScene(State& state) {
	if (_state == NULL)
		RW_THROW("RWPEIsland (resetScene): please call initPhysics first!");
	_rwstate = state;

	const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		body->reset(*_state,_rwstate);
	}
	const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getPersistentConstraints();
	BOOST_FOREACH(RWPEConstraint* constraint, constraints) {
		constraint->reset(*_state,_rwstate);
	}
	_bc->clearTemporaryConstraints(*_state);
	BOOST_FOREACH(SimulatedController::Ptr controller, _controllers) {
		controller->reset(_rwstate);
	}
	BOOST_FOREACH(SimulatedSensor::Ptr sensor, _sensors) {
		sensor->reset(_rwstate);
	}
	_state->getContactData().clear();
	_state->setContacts(std::vector<Contact>(),ContactDetectorTracking());
	_state->setLastTimeStep(0);
	_state->setRepetitions(0);
	_state->setTime(0);
}

void RWPEIsland::initPhysics(State& state) {
	if (_dwc == NULL)
		RW_THROW("RWPEIsland (initPhysics): no dynamic workcell given - please use load first!");
	if (_bc != NULL)
		RW_THROW("RWPEIsland (initPhysics): was called earlier!");
	_bc = new RWPEBodyConstraintGraph();
	if (_state != NULL)
		delete _state;
	_state = new RWPEIslandState();
	_bc->initFromDWC(_dwc);
	if (_bp != NULL)
		delete _bp;
	_bp = new RWPEBroadPhase(_dwc);
	const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		_bp->addObject(body->get()->getObject(), dynamic_cast<const RWPEBodyDynamic*>(body) != NULL);
	}
	_detector->setProximityFilterStrategy(_bp->getProximityFilterStrategy());

	// Add controllers
	const DynamicWorkCell::ControllerList& controllers = _dwc->getControllers();
	BOOST_FOREACH(SimulatedController::Ptr controller, controllers){
		addController(controller);
	}

	// Add sensors
    BOOST_FOREACH(SimulatedSensor::Ptr sensor, _dwc->getSensors()){
    	addSensor(sensor, state);
	}

	resetScene(state);
}

void RWPEIsland::exitPhysics() {
	if (_bc == NULL)
		RW_THROW("RWPEIsland (exitPhysics): initPhysics has not been called!");
	delete _bc;
	_bc = NULL;
	if (_state != NULL)
		delete _state;
	_state = NULL;
	_rwstate = State();
	if (_bp != NULL)
		delete _bp;
	_bp = NULL;
	_sensors.clear();
}

double RWPEIsland::getTime() {
	return _state->getTime();
}

void RWPEIsland::setEnabled(Body::Ptr body, bool enabled) {
	// Ignore for now
}

void RWPEIsland::setDynamicsEnabled(Body::Ptr body, bool enabled) {
	// Ignore for now
}

SimulatorDebugRender::Ptr RWPEIsland::createDebugRender() {
	if (_bc == NULL)
		return NULL;
	if (_render == NULL)
		_render = ownedPtr(new RWPEDebugRender());
	return _render;
}

PropertyMap& RWPEIsland::getPropertyMap() {
	return _map;
}

const PropertyMap& RWPEIsland::getPropertyMap() const {
	return _map;
}

const PropertyMap& RWPEIsland::getPropertyMapDefault() const {
	return _defaultMap;
}

void RWPEIsland::emitPropertyChanged() {
}

void RWPEIsland::addController(SimulatedController::Ptr controller) {
	_controllers.push_back(controller);
}

void RWPEIsland::removeController(SimulatedController::Ptr controller) {
	std::vector<rwlibs::simulation::SimulatedController::Ptr>::iterator it;
	for (it = _controllers.begin(); it != _controllers.end(); it++) {
		if ((*it) == controller.get())
			it = _controllers.erase(it);
	}
}

void RWPEIsland::addBody(Body::Ptr body, State &state) {
	RW_WARN("RWPEIsland (addBody): body can not be added yet (" << body->getName() << ")");
}

void RWPEIsland::addDevice(DynamicDevice::Ptr dev, State &state) {
	RW_WARN("RWPEIsland (addDevice): device can not be added yet (" << dev->getName() << ")");
}

void RWPEIsland::addSensor(SimulatedSensor::Ptr sensor, State &state) {
	const SimulatedSensor* const ssensor = sensor.get();
	if(const SimulatedTactileSensor* const tsensor = dynamic_cast<const SimulatedTactileSensor*>(ssensor) ){
		if(const SimulatedFTSensor* const ftsensor = dynamic_cast<const SimulatedFTSensor*>(tsensor) ){
			const Frame* const parentFrame = ftsensor->getBody1()->getBodyFrame();
			const Frame* const sensorFrame = ftsensor->getBody2()->getBodyFrame();
			RW_ASSERT(parentFrame != NULL);
			RW_ASSERT(sensorFrame != NULL);
			if(_bc->getBody(parentFrame) == NULL)
				RW_THROW("RWPEIsland (addSensor): The frame (" << parentFrame->getName() << ") that the sensor is being attached to is not a body in the simulator! Did you remember to run initphysics!");
			if(_bc->getBody(sensorFrame) == NULL)
				RW_THROW("RWPEIsland (addSensor): The frame (" << sensorFrame->getName() << ") that the sensor is being attached to is not a body in the simulator! Did you remember to run initphysics!");
		} else {
			const Frame* bframe = tsensor->getFrame();
			RW_ASSERT(bframe!=NULL);
			if(_bc->getBody(bframe) == NULL)
				RW_THROW("RWPEIsland (addSensor): The frame (" << bframe->getName() << ") that the sensor is being attached to is not a body in the simulator! Did you remember to run initphysics!");
		}
		_sensors.push_back(sensor);
    } else {
    	RW_WARN("RWPEIsland (addSensor): this type of sensor can not be added yet (" << sensor->getName() << ")");
    }
}

void RWPEIsland::removeSensor(SimulatedSensor::Ptr sensor) {
	// Do nothing
}

void RWPEIsland::attach(Body::Ptr b1, Body::Ptr b2) {
	RW_WARN("RWPEIsland (attach): attach not possible yet (" << b1->getName() << "," << b2->getName() << ")");
}

void RWPEIsland::detach(Body::Ptr b1, Body::Ptr b2) {
	RW_WARN("RWPEIsland (detach): detach not possible yet (" << b1->getName() << "," << b2->getName() << ")");
}

std::vector<SimulatedSensor::Ptr> RWPEIsland::getSensors() {
	return std::vector<SimulatedSensor::Ptr>(_sensors.begin(), _sensors.end());
}

void RWPEIsland::setSimulatorLog(SimulatorLogScope::Ptr log) {
	_log->setSimulatorLog(log);
}

void RWPEIsland::doStep(double dt) {
	const bool doLog = _log->doLog();
	if (doLog) {
		_log->beginStep(_state->getTime(),__FILE__,__LINE__);
		_log->addPositions("Positions",_bc,_rwstate,__FILE__,__LINE__);
		_log->addVelocities("Velocities",_bc,_rwstate,*_state,__FILE__,__LINE__);
	}

	if (_state->hasContacts()) {
		const std::size_t rawNo = RWPEUtil::getMarkedContacts(_state->getContacts(),_state->getContactsTracking(),RWPEUtil::MARK_RAW).size();
		const std::size_t newNo = RWPEUtil::getMarkedContacts(_state->getContacts(),_state->getContactsTracking(),RWPEUtil::MARK_NEW).size();
		const std::size_t remaining = _state->getContacts().size()-rawNo-newNo;
		const std::size_t rwpeTempConstraints = _bc->getTemporaryConstraints(_state).size();
		if (_log != NULL) {
			_log->beginSection("Contact Status",__FILE__,__LINE__);
			_log->addContacts("New Contacts", RWPEUtil::getContacts(_state->getContacts(),RWPEUtil::getMarkedContacts(_state->getContacts(),_state->getContactsTracking(),RWPEUtil::MARK_NEW)), __FILE__,__LINE__);
			std::vector<const RWPEContact*> rwpeContacts;
			const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getTemporaryConstraints(_state);
			BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
				if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint))
					rwpeContacts.push_back(contact);
			}
			_log->addContacts("Known Contacts", rwpeContacts, __FILE__,__LINE__);
			_log->endSection(__LINE__);
		}
		if (remaining != rwpeTempConstraints)
			RW_THROW("RWPEIsland (doStep): the number of temporary constraints should be the same as the number of contacts that are not either raw or new.");
		if (rawNo != 0)
			RW_THROW("RWPEIsland (doStep): there should not be raw contacts at this point.");
	}

	if (_controllers.size() > 0) {
		if (_state->getRepetitions() == 0) {
			// Update controllers
			Simulator::UpdateInfo info;
			info.dt = dt;
			info.dt_prev = _state->getLastTimeStep();
			info.time = _state->getTime();
			info.rollback = false;
			BOOST_FOREACH(SimulatedController::Ptr controller, _controllers) {
				controller->update(info,_rwstate);
			}
		} else {
			RWPE_ENGINE_LOG(_log,"Repetitions: " << _state->getRepetitions());
		}
	}

	IntegrateSample sampleInitial(0,*_state,_rwstate);

	bool discontinuity;
	_log->beginSection("Collision Solver",RWPE_LOCATION);
	discontinuity = collisionSolver(*_state,_rwstate);
	_log->endSection(__LINE__);
	if (_state->getTime() == 0)
		discontinuity = true;

	if (doLog) {
		_log->addVelocities("Velocities after Collision Solver",_bc,_rwstate,*_state,RWPE_LOCATION);
		std::vector<const RWPEContact*> rwpeContacts;
		const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getTemporaryConstraints(_state);
		BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
			if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint))
				rwpeContacts.push_back(contact);
		}
		_log->addContacts("Contact Positions", rwpeContacts, __FILE__,__LINE__);
		_log->addContactVelocities("Contact Velocities",rwpeContacts,_rwstate,*_state,RWPE_LOCATION);
		_log->addContactWrenches("Contact Wrenches",rwpeContacts,*_state,RWPELogUtil::TOTAL,RWPE_LOCATION);
	}

	// Recalculate the applied wrench (the current one is based on previous prediction)
	// The constraint force calculated in previous step is kept
	BOOST_FOREACH(RWPEConstraint* constraint, _bc->getConstraints(*_state)) {
		constraint->clearWrenchApplied(*_state);
		constraint->update(*_state,_rwstate);
	}

	if (doLog) {
		std::vector<const RWPEContact*> rwpeContacts;
		const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getTemporaryConstraints(_state);
		BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
			if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint))
				rwpeContacts.push_back(contact);
		}
		_log->addContactWrenches("Contact Wrenches (applied updated)",rwpeContacts,*_state,RWPELogUtil::TOTAL,RWPE_LOCATION);
	}

	if (discontinuity) {
		// If there is a discontinuity in velocities, the constraint forces are unknown
		// Hence these are set to zero.
		BOOST_FOREACH(RWPEConstraint* constraint, _bc->getConstraints(*_state)) {
			constraint->clearWrenchConstraint(*_state);
			RWPEContact* const contact = dynamic_cast<RWPEContact*>(constraint);
			if (contact != NULL) {
				contact->reset(*_state,_rwstate);
			}
		}
		_log->log("Discontinuity",RWPE_LOCATION) << "Previously known constraint forces cleared.";
	}

	IntegrateSample sample0(0,*_state,_rwstate);
	IntegrateSample sampleH;
	if (doLog)
		_log->beginSection("Broad-phase Rollback",__FILE__,__LINE__);
	{
		const double oldDt = dt;
		dt = rollbackBroadPhase(dt,discontinuity,sample0,sampleH); // sampleH is sample0 adjusted with new time and body positions
		RWPE_ENGINE_LOG(_log,"Broad-phase check set timestep to " << dt << " (from " << oldDt << ").")
	}
	if (doLog)
		_log->endSection(__LINE__);

	{
		// Sanity check
		double workspace;
		if (_map.has("RWPEWorkspace"))
			workspace = _map.get<double>("RWPEWorkspace");
		else
			workspace = _defaultMap.get<double>("RWPEWorkspace");

		const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();
		if (bodies.size() > 0) {
			BOOST_FOREACH(const RWPEBody* const body, bodies) {
				const Transform3D<> pos = body->getWorldTcom(sampleH.islandState);
				if (pos.P().normInf() > workspace) {
					if (doLog) {
						_log->log("FAILED",RWPE_LOCATION) << "position for body \"" << body->get()->getName() << "\" is " << pos.P() << " which is outisde the workspace of size " << workspace << "x" << workspace << "x" << workspace << " m.";
						_log->endStep(_state->getTime(),__LINE__);
					}
					RW_THROW("RWPEIsland (doStep): position for body \"" << body->get()->getName() << "\" is " << pos.P() << " which is outisde the workspace of size " << workspace << "x" << workspace << "x" << workspace << " m - aborting.");
				}
			}
		}
	}

	// The contact positions are updated
	ContactDetectorData cdData = _state->getContactData();
	//long long start = TimerUtil::currentTimeMs();
	sampleH.forwardContacts = _detector->findContacts(sampleH.rwstate,cdData,sampleH.forwardTrack,_log->makeScope("Find Contacts",RWPE_LOCATION));
	//long long end = TimerUtil::currentTimeMs();
	//std::cout << "RWPE contacts: " << sampleH.forwardContacts.size() << " - " << end-start << " ms" << std::endl;

	if (doLog) {
		_log->addPositions("Positions after Broad-phase Rollback",_bc,sampleH.islandState,RWPE_LOCATION);
		_log->addContacts("Contacts after Broad-phase Rollback",sampleH.forwardContacts,RWPE_LOCATION);

		_log->beginSection("Narrow-phase Rollback",RWPE_LOCATION);
	}

	{
		int enableRollback;
		if (_map.has("RWPERollback"))
			enableRollback = _map.get<int>("RWPERollback");
		else
			enableRollback = _defaultMap.get<int>("RWPERollback");

		if (enableRollback) {
			const double oldDt = dt;
			dt = rollback(discontinuity,sample0,sampleH,cdData); // sampleH is sample0 adjusted with new time and body positions
			if (doLog) {
				if (dt == 0) {
					_log->log("Repeating Timestep",RWPE_LOCATION);
				} else if (dt < oldDt) {
					_log->log("Decreased Timestep",RWPE_LOCATION) << "Narrow-phase check decreased timestep to " << dt << " from " << oldDt << ".";
				} else {
					_log->log("No change",RWPE_LOCATION) << "Time-step was not decreased.";
				}
			}
		} else if (doLog) {
			_log->log("Disabled",RWPE_LOCATION) << "Rollback was disabled! (RWPERollback=0)";
		}
	}
	if (doLog)
		_log->endSection(__LINE__);
	if (dt == 0) {
		_log->log("Repeating Timestep",RWPE_LOCATION);
		_log->endStep(_state->getTime(),__LINE__);
		return;
	}

	// Velocity correction of the info from tracking
	for (std::size_t i = 0; i < sampleH.forwardContacts.size(); i++) {
		const ContactStrategyTracking::UserData::Ptr data = sampleH.forwardTrack.getUserData(i);
		if (data == RWPEUtil::MARK_RAW) {
			// Construct a candidate RWPEContact and if velocity is small enough we just treat it as a known contact (avoiding rollback and bouncing)
			const Contact &c = sampleH.forwardContacts[i];
			const RWPEBody* const bodyA = _bc->getBody(c.getFrameA());
			const RWPEBody* const bodyB = _bc->getBody(c.getFrameB());
			if (bodyA == NULL)
				RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameA()->getName() << "\".");
			if (bodyB == NULL)
				RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameB()->getName() << "\".");
			RWPEContact* const rwpecontact = new RWPEContact(bodyA,bodyB,_materialMap->getFrictionModel(*bodyA,*bodyB),c,sampleH.rwstate);
			const Vector3D<> velI = rwpecontact->getVelocityParentW(sampleH.islandState,sampleH.rwstate).linear();
			const Vector3D<> velJ = rwpecontact->getVelocityChildW(sampleH.islandState,sampleH.rwstate).linear();
			const Vector3D<> nij = rwpecontact->getNormalW(sampleH.islandState);
			const double penetratingVel = dot(velI-velJ,nij);
			if (penetratingVel < 1e-2) { // default in collision solver
				_bc->addTemporaryConstraint(rwpecontact,sampleH.islandState);
				sampleH.forwardTrack.setUserData(i,RWPEUtil::RWPEUserData::make(rwpecontact));
				//sampleH.islandState.setContacts(sampleH.forwardContacts,sampleH.forwardTrack);
				std::cout << "RWPE: new contact with no velocity!!!" << std::endl;
				//RW_THROW("Found new contact with no or too small penetrating velocity.");
			} else {
				delete rwpecontact;
			}
		} else {
			// Update the RWPEContact and if velocity is too large, the RWPEContact is removed and the contact is treated as a new contact.
			// This will cause rollback, so it might be wrong place to do this here?
			const RWPEUtil::RWPEUserData* const rwpeData = dynamic_cast<const RWPEUtil::RWPEUserData*>(data.get());
			if (!rwpeData)
				RW_THROW("RWPEIsland (doStep): got unexpected type of contact.");
			// Search for the contact in a way such that we can also delete it!
			BOOST_FOREACH(RWPEConstraint* const rwpeconstraint, sampleH.islandState.getTemporaryConstraints()) {
				RWPEContact* const rwpecontact = dynamic_cast<RWPEContact*>(rwpeconstraint);
				if (!rwpecontact)
					continue;
				if (rwpecontact == rwpeData->contact) {
					rwpecontact->setContact(sampleH.forwardContacts[i],sampleH.rwstate);
					const Vector3D<> velI = rwpecontact->getVelocityParentW(sampleH.islandState,sampleH.rwstate).linear();
					const Vector3D<> velJ = rwpecontact->getVelocityChildW(sampleH.islandState,sampleH.rwstate).linear();
					const Vector3D<> nij = rwpecontact->getNormalW(sampleH.islandState);
					const double penetratingVel = dot(velI-velJ,nij);
					if (penetratingVel > 1) {
						if (doLog) {
							_log->log("FAILED",RWPE_LOCATION) << "Got updated contact with too large penetrating velocity. Depth: " << sampleH.forwardContacts[i].getDepth();
							_log->endStep(_state->getTime(),__LINE__);
						}
						RW_THROW("Got updated contact with too large penetrating velocity. Depth: " << sampleH.forwardContacts[i].getDepth());
					}
					break;
				}
			}
		}
	}

	{
		const std::vector<std::size_t> newContactsH = RWPEUtil::getMarkedContacts(sampleH.forwardContacts,sampleH.forwardTrack,RWPEUtil::MARK_RAW);
		if (newContactsH.size() > 0) {
			if (doLog)
				_log->addContacts("New Contacts found",RWPEUtil::getContacts(sampleH.forwardContacts,newContactsH),RWPE_LOCATION);
			/*const double minDist = RWPEUtil::minDistance(RWPEUtil::getContacts(sampleH.forwardContacts,newContactsH));
			if (minDist > 0) {
				const std::string error = "Failure in RWPEIsland (doStep): there should not be non-penetrating raw contacts at step H!";
				_log->log("Fatal Failure",RWPE_LOCATION) << error;
				RW_THROW(error);
			}*/
			RWPEUtil::mark(sampleH.forwardContacts,sampleH.forwardTrack,RWPEUtil::MARK_RAW,RWPEUtil::MARK_NEW);
		}
	}

	sampleH.islandState.setContacts(sampleH.forwardContacts, sampleH.forwardTrack);
	if (doLog) {
		_log->addContactTracking("Tracking of Contacts",sample0.islandState,sampleH.islandState,RWPE_LOCATION);
	}

	velocityAndForce(dt, discontinuity, sample0, sampleH);

	// Remove contacts with too large outgoing velocity (contacts that breaks away)
	/*
	BOOST_FOREACH(const RWPEConstraint* const constraint, sampleH.islandState.getTemporaryConstraints()) {
		if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint)) {
			const Vector3D<> linVelI = contact->getVelocityParentW(sampleH.islandState,sampleH.rwstate).linear();
			const Vector3D<> linVelJ = contact->getVelocityChildW(sampleH.islandState,sampleH.rwstate).linear();
			const Vector3D<> linRelVel = linVelJ-linVelI;
			const Vector3D<> nij = contact->getNormalW(sampleH.islandState);
			const bool outgoing = dot(linRelVel,nij) > 1e-3;
			if (outgoing) {
				sampleH.islandState.removeTemporaryConstraint(contact);
			}
		}
	}*/

	// Remove contacts that is too far from being in contact (contacts that are breaking away)
	RWPEUtil::updateTemporaryContacts(sampleH.islandState.getContacts(),sampleH.islandState.getContactsTracking(),_bc,sampleH.islandState,sampleH.rwstate);
	BOOST_FOREACH(const RWPEConstraint* const constraint, sampleH.islandState.getTemporaryConstraints()) {
		if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint)) {
			const double distance = -contact->getContact().getDepth();
			if (distance > dt*1) {
				sampleH.islandState.removeTemporaryConstraint(contact);
			}
		}
	}

	// Update the constraints
	sampleH.forwardContacts = sampleH.islandState.getContacts();
	sampleH.forwardTrack = sampleH.islandState.getContactsTracking();

	position(sampleH, cdData);

	IntegrateSample& sample = sampleH;

	const RWPEIslandState& islandState = sample.islandState;

	*_state = islandState;
	_state->setLastTimeStep(sample.time);
	if (sample.time == 0)
		_state->setRepetitions(_state->getRepetitions()+1);
	else
		_state->setRepetitions(0);
	_state->setTime(_state->getTime()+sample.time);

	_rwstate = sample.rwstate;

	// Update RobWork bodies
	const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		body->updateRW(_rwstate,*_state);
	}

	// Keep only the new contacts for bouncing that are within a certain distance
	RWPEUtil::remove(sample.forwardContacts,sample.forwardTrack,RWPEUtil::MARK_RAW);
	if (doLog)
		_log->addContacts("All contacts for next step",sample.forwardContacts,RWPE_LOCATION);
	RWPEUtil::removeContactsOutsideThreshold(sample.forwardContacts,sample.forwardTrack,0.0001,RWPEUtil::MARK_NEW);
	if (doLog)
		_log->addContacts("All contacts after removing contacts outside threshold",sample.forwardContacts,RWPE_LOCATION);

	// Update the constraints
	RWPEUtil::updateTemporaryContacts(sample.forwardContacts,sample.forwardTrack,_bc,*_state,_rwstate);

	// Store contact information in state
	_state->setContactData(cdData);
	_state->setContacts(sample.forwardContacts,sample.forwardTrack);

	// Update tactile sensors
	if (sample.time > 0 && _sensors.size() > 0) {
		_log->beginSection("Sensor Update",RWPE_LOCATION);
		RWPEUtil::updateSensors(_sensors,_state->getTime(),sample.time,_state->getLastTimeStep(),_bc,sample0.islandState,*_state,_rwstate,*_log);
		_log->endSection(__LINE__);
	}

	if (_render != NULL) {
		_render.scast<RWPEDebugRender>()->update(_bc,_state,_gravity);
	}

	if (doLog) {
		_log->endStep(_state->getTime(),__LINE__);
	}
}

void RWPEIsland::position(IntegrateSample& sample, ContactDetectorData& cdData) const {
	const bool doLog = _log->doLog();

	int enableCorrection;
	if (_map.has("RWPECorrection"))
		enableCorrection = _map.get<int>("RWPECorrection");
	else
		enableCorrection = _defaultMap.get<int>("RWPECorrection");

	if (enableCorrection) {
		bool repeat = true;
		if (doLog) {
			_log->beginSection("Correction",RWPE_LOCATION);
			_log->addPositions("Positions before update",_bc,sample.rwstate,RWPE_LOCATION);
			_log->addContacts("Contacts before update",sample.forwardContacts,RWPE_LOCATION);
		}
		for (std::size_t iteration = 0; iteration < 5 && repeat; iteration++) {
			repeat = false;
			//RWPEUtil::updateTemporaryContacts(sample.forwardContacts,sample.forwardTrack,_bc,sample.islandState,sample.rwstate);
			std::list<RWPEConstraint*> constraints = _bc->getConstraints(sample.islandState);
			const std::vector<std::size_t> contactIDs = RWPEUtil::getMarkedContacts(sample.forwardContacts,sample.forwardTrack,RWPEUtil::MARK_NEW);
			std::vector<RWPEContact*> rwpecontacts;
			BOOST_FOREACH(std::size_t id, contactIDs) {
				const Contact &c = sample.forwardContacts[id];
				const RWPEBody* const bodyA = _bc->getBody(c.getFrameA());
				const RWPEBody* const bodyB = _bc->getBody(c.getFrameB());
				if (bodyA == NULL)
					RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameA()->getName() << "\".");
				if (bodyB == NULL)
					RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameB()->getName() << "\".");
				RWPEContact* const rwpecontact = new RWPEContact(bodyA,bodyB,_materialMap->getFrictionModel(*bodyA,*bodyB),c,sample.rwstate);
				rwpecontacts.push_back(rwpecontact);
				constraints.push_back(rwpecontact);
			}
			_correction->correct(constraints,sample.islandState,_map,_log);
			BOOST_FOREACH(RWPEContact* contact, rwpecontacts) {
				delete contact;
			}
			rwpecontacts.clear();
			// Update RobWork bodies
			BOOST_FOREACH(const RWPEBody* body, _bc->getBodies()) {
				body->updateRW(sample.rwstate,sample.islandState);
			}
			// Old:
			//sample.forwardContacts = _detector->updateContacts(sample.rwstate,cdData,sample.forwardTrack);
			sample.forwardContacts = _detector->findContacts(sample.rwstate,cdData,sample.forwardTrack,_log->makeScope("Find Contacts",RWPE_LOCATION));
			if (RWPEUtil::getMarkedContacts(sample.forwardContacts,sample.forwardTrack,RWPEUtil::MARK_RAW).size() > 0) {
				RWPEUtil::mark(sample.forwardContacts,sample.forwardTrack,RWPEUtil::MARK_RAW,RWPEUtil::MARK_NEW);
				repeat = true;
			}
			if (doLog) {
				_log->addPositions("Positions after update",_bc,sample.rwstate,RWPE_LOCATION);
				_log->addContacts("Contacts after udpate",sample.forwardContacts,RWPE_LOCATION);
			}
			RWPEUtil::updateTemporaryContacts(sample.forwardContacts,sample.forwardTrack,_bc,sample.islandState,sample.rwstate);
		}
		if (doLog) {
			_log->addPositions("Positions after update",_bc,sample.rwstate,RWPE_LOCATION);
			_log->addContacts("Contacts after udpate",sample.forwardContacts,RWPE_LOCATION);
			_log->endSection(__LINE__);
		}
	}
}

void RWPEIsland::velocityAndForce(double dt, bool discontinuity, const IntegrateSample& sample0, IntegrateSample& sampleH) const {
	const bool doLog = _log->doLog();
	if (_bc->hasContactsOrConstraints(sampleH.islandState)) {
		// Compute Applied Forces (update constraints for new state)
		{
			_log->beginSection("Compute Applied Forces",RWPE_LOCATION);
			RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getConstraints(sampleH.islandState);
			BOOST_FOREACH(RWPEConstraint* constraint, constraints) {
				constraint->clearWrench(sampleH.islandState);
				constraint->update(sampleH.islandState,sampleH.rwstate);
				constraint->step(sampleH.islandState,sampleH.rwstate,dt);
			}
			_log->endSection(__LINE__);
		}
		// Contact & Constraint Force Resolution
		_log->beginSection("Contact & Constraint Resolver",RWPE_LOCATION);
		constraintSolver(dt,discontinuity,sample0.islandState,sampleH.islandState,sampleH.rwstate);
		if (doLog) {
			_log->endSection(__LINE__);
			std::vector<const RWPEContact*> rwpeContacts;
			const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getTemporaryConstraints(&sampleH.islandState);
			BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
				if (const RWPEContact* const contact = dynamic_cast<const RWPEContact*>(constraint))
					rwpeContacts.push_back(contact);
			}
			_log->addContacts("Contacts",rwpeContacts,RWPE_LOCATION);
			_log->addContactWrenches("Contact Wrenches Found",rwpeContacts,sampleH.islandState,RWPELogUtil::TOTAL,RWPE_LOCATION);
		}
	}
	// Velocity Update
	if (doLog) {
		_log->beginSection("Velocity Update",RWPE_LOCATION);
		if (discontinuity)
			_log->log("Discontinuous Integrator",RWPE_LOCATION);
		else
			_log->log("Continuous Integrator",RWPE_LOCATION);
		_log->addVelocities("Velocity at 0",_bc,sample0.rwstate,sample0.islandState,RWPE_LOCATION);
	}
	RWPEUtil::velocityUpdate(dt,_gravity,discontinuity,_bc,sample0.islandState,sampleH.islandState,sample0.rwstate,_log);
	if (doLog) {
		_log->addVelocities("Velocity at h",_bc,sample0.rwstate,sampleH.islandState,RWPE_LOCATION);
		_log->endSection(__LINE__);
		_log->addVelocities("New Velocity",_bc,sample0.rwstate,sampleH.islandState,RWPE_LOCATION);
	}
}

bool RWPEIsland::collisionSolver(RWPEIslandState& islandState, const State& rwstate) const {
	// Construct list of new contacts
	const std::vector<Contact> contacts = islandState.getContacts();
	ContactDetectorTracking tracking = islandState.getContactsTracking();
	const std::vector<std::size_t> contactIDs = RWPEUtil::getMarkedContacts(contacts,tracking,RWPEUtil::MARK_NEW);
	const std::vector<Contact> newContacts = RWPEUtil::getContacts(contacts,contactIDs);
	_log->addContacts("New contacts",contacts,__FILE__,__LINE__);

	// Construct list of RWPEContacts for doing bouncing
	std::vector<const RWPEContact*> rwpecontacts;
	//std::vector<RWPEContact*> rwpecontactsLeaving;
	BOOST_FOREACH(std::size_t id, contactIDs) {
		const Contact &c = contacts[id];
		const RWPEBody* const bodyA = _bc->getBody(c.getFrameA());
		const RWPEBody* const bodyB = _bc->getBody(c.getFrameB());
		if (bodyA == NULL)
			RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameA()->getName() << "\".");
		if (bodyB == NULL)
			RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameB()->getName() << "\".");
		RWPEContact* const rwpecontact = new RWPEContact(bodyA,bodyB,_materialMap->getFrictionModel(*bodyA,*bodyB),c,rwstate);
		tracking.setUserData(id,RWPEUtil::RWPEUserData::make(rwpecontact));
		const Vector3D<> velI = rwpecontact->getVelocityParentW(islandState,rwstate).linear();
		const Vector3D<> velJ = rwpecontact->getVelocityChildW(islandState,rwstate).linear();
		const Vector3D<> nij = rwpecontact->getNormalW(islandState);
		bool leaving = -dot(velI-velJ,nij) >= 0;
		_bc->addTemporaryConstraint(rwpecontact,islandState);
		if (!leaving) {
		//if (leaving) {
			//rwpecontactsLeaving.push_back(rwpecontact);
		//} else {
			rwpecontacts.push_back(rwpecontact);
		}
	}
	islandState.setContacts(contacts,tracking);

	// Do bouncing on all known contacts
	if (rwpecontacts.size() > 0) {
		_log->addContacts("Contacts to treat as bouncing",rwpecontacts,__FILE__,__LINE__);
		std::string solverID;
		if (_map.has("RWPECollisionSolver"))
			solverID = _map.get<std::string>("RWPECollisionSolver");
		else
			solverID = _defaultMap.get<std::string>("RWPECollisionSolver");
		_log->log(RWPE_LOCATION) << "Making solver \"" << solverID << "\"";
		const RWPECollisionSolver::Ptr bouncingSolver = RWPECollisionSolver::Factory::makeSolver(solverID);
		bouncingSolver->doCollisions(rwpecontacts,*_bc,_materialMap,islandState,rwstate,_map,_log);
		return true;
	} else {
		_log->log(RWPE_LOCATION) << "No contacts to treat as bouncing.";
		return false;
	}
}

double RWPEIsland::rollbackBroadPhase(double dt, bool discontinuity, const IntegrateSample& first, IntegrateSample& res) const {
	const bool doLog = _log->doLog();

	int iterations;
	if (_map.has("RWPERollbackIterations"))
		iterations = _map.get<int>("RWPERollbackIterations");
	else
		iterations = _defaultMap.get<int>("RWPERollbackIterations");
	RW_ASSERT(iterations >= 0);

	if (doLog) {
		_log->addPositions("Positions",_bc,first.rwstate,__FILE__,__LINE__);
		_log->addVelocities("Velocities",_bc,first.rwstate,first.islandState,__FILE__,__LINE__);
	}

	for (unsigned int iteration = 1; iteration <= (unsigned int)(iterations+1); iteration++) {
		const double dtTry = dt/std::pow(2.,(int)(iteration-1));
		RWPE_ENGINE_LOG(_log,"Trying timestep " << dtTry << ".")
		res = first;
		res.time = dtTry;
		//RWPEUtil::integratePositionEuler(dtTry,_bc,res.islandState,res.rwstate);
		RWPEUtil::positionUpdate(dtTry,_gravity,discontinuity,_bc,res.islandState,res.rwstate);
		if (doLog) {
			_log->addPositions("Positions",_bc,res.rwstate,RWPE_LOCATION);
		}
		const bool penetrating = _bp->maxPenetrationExceeded(_detector,res.rwstate,_log);
		if (!penetrating) {
			dt = dtTry;
			break;
		}
		if (iteration == (unsigned int)iterations) {
			RWPE_ENGINE_LOG(_log,"Maximum of " << iterations << " iterations exceeded in broad-phase rollback!")
			RW_THROW("RWPEIsland (rollbackBroadPhase): Maximum of " << iterations << " iterations exceeded!");
		}
	}

	if (doLog) {
		_log->addPositions("Positions after Broad-Phase rollback",_bc,res.rwstate,__FILE__,__LINE__);
		_log->addVelocities("Velocities after Broad-Phase rollback",_bc,res.rwstate,first.islandState,__FILE__,__LINE__);
	}

	return dt;
}

double RWPEIsland::rollback(bool discontinuity, IntegrateSample& sample0, IntegrateSample& sampleH, ContactDetectorData& cdData) const {
	const bool doLog = _log->doLog();

	double threshold;
	if (_map.has("RWPERollbackThreshold"))
		threshold = _map.get<double>("RWPERollbackThreshold");
	else
		threshold = _defaultMap.get<double>("RWPERollbackThreshold");

	const std::vector<std::size_t> newContactsH = RWPEUtil::getMarkedContacts(sampleH.forwardContacts,sampleH.forwardTrack,RWPEUtil::MARK_RAW);
	std::vector<Contact> contactsToUse;
	for (std::size_t i = 0; i < newContactsH.size(); i++) {
		//const ContactStrategyTracking::UserData::Ptr data = sampleH.forwardTrack.getUserData(newContactsH[i]);
		// Construct a candidate RWPEContact and if velocity is small enough we just treat it as a known contact (avoiding rollback and bouncing)
		const Contact &c = sampleH.forwardContacts[newContactsH[i]];
		const RWPEBody* const bodyA = _bc->getBody(c.getFrameA());
		const RWPEBody* const bodyB = _bc->getBody(c.getFrameB());
		if (bodyA == NULL)
			RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameA()->getName() << "\".");
		if (bodyB == NULL)
			RW_THROW("Could not find a RWPEBody for frame \"" << c.getFrameB()->getName() << "\".");
		RWPEContact* const rwpecontact = new RWPEContact(bodyA,bodyB,_materialMap->getFrictionModel(*bodyA,*bodyB),c,sampleH.rwstate);
		const Vector3D<> velI = rwpecontact->getVelocityParentW(sampleH.islandState,sampleH.rwstate).linear();
		const Vector3D<> velJ = rwpecontact->getVelocityChildW(sampleH.islandState,sampleH.rwstate).linear();
		const Vector3D<> nij = rwpecontact->getNormalW(sampleH.islandState);
		const double penetratingVel = dot(velI-velJ,nij);
		if (penetratingVel >= 1e-1) { // default in collision solver
			contactsToUse.push_back(c);
		}
		delete rwpecontact;
	}

	double minDist;
	if (contactsToUse.size() > 0) {
		minDist = RWPEUtil::minDistance(contactsToUse);
		if (minDist > -threshold) {
			if (doLog)
				_log->log("Taking full step",RWPE_LOCATION);
			return sampleH.time;
		//} else {
			//if (doLog)
				//_log->log("Not Required",RWPE_LOCATION) << "Distance for new contact(s) at step size " << sampleH.time << " was low enough (" << minDist << ") that rollback is not required!";
			//RWPE_DEBUG_ROLLBACK("Distance for new contact(s) at step size " << sampleH.time << " was low enough (" << minDist << ") that rollback is not required!");
		}
	} else {
		if (doLog)
			_log->log("No contacts",RWPE_LOCATION) << "Contacts: " << sampleH.forwardContacts.size() << " With large enough penetrating velocity: " << contactsToUse.size() << " Tracking: " << sampleH.forwardTrack.getSize() << " (zero new).";
		return sampleH.time;
	}

	// Construct backward tracking
	sampleH.backwardTrack = sampleH.forwardTrack;
	sampleH.backwardContacts = sampleH.forwardContacts;
	if (doLog) {
		_log->addContacts("Initial Contacts",sampleH.backwardContacts,RWPE_LOCATION);
		_log->addContacts("New Contacts",RWPEUtil::getContacts(sampleH.forwardContacts,newContactsH),RWPE_LOCATION);
		_log->addContacts("New Contacts with ingoing velocity",contactsToUse,RWPE_LOCATION);
	}

	// Do a backwards tracking of the contacts to time dt=0
	sample0.backwardTrack = sampleH.backwardTrack;
	sample0.backwardContacts = _detector->updateContacts(sample0.rwstate,cdData,sample0.backwardTrack,_log->makeScope("Update Contacts",RWPE_LOCATION));
	if (doLog)
		_log->addContacts("Backtracked Contacts",sample0.backwardContacts,RWPE_LOCATION);
	const std::vector<std::size_t> newContacts0backward = RWPEUtil::getMarkedContacts(sample0.backwardContacts,sample0.backwardTrack,RWPEUtil::MARK_RAW);
	if (newContacts0backward.size() == 0) {
		if (doLog)
			_log->log("Tracking failed",RWPE_LOCATION) << "The new contact(s) could not be traced back to time dt = 0!\n  - taking complete step instead.";
		return sampleH.time;
	} else {
		const std::vector<Contact> contactsBefore = RWPEUtil::getContacts(sample0.backwardContacts,newContacts0backward);
		if (doLog)
			_log->addContacts("Tracked Contacts at dt = 0",contactsBefore,RWPE_LOCATION);
		RWPEUtil::removePenetrating(sample0.backwardContacts,sample0.backwardTrack,RWPEUtil::MARK_RAW);
		const std::vector<std::size_t> ids = RWPEUtil::getMarkedContacts(sample0.backwardContacts,sample0.backwardTrack,RWPEUtil::MARK_RAW);
		const std::vector<Contact> contactsBeforePenetratingRemoved = RWPEUtil::getContacts(sample0.backwardContacts,ids);
		if (contactsBefore.size() != contactsBeforePenetratingRemoved.size()) {
			if (doLog)
				_log->log("Removed invalid contacts",RWPE_LOCATION) << sample0.backwardContacts.size() << " contacts left after removing penetrating contacts.";
			//RW_THROW("Contact were penetrating initially - rollback not possible.");
		}

		if (contactsBefore.size() > 0) {
			const double minDist = RWPEUtil::minDistance(contactsBefore);
			if (minDist < threshold) {
				if (doLog)
					_log->log("Repeating Step with more contacts",RWPE_LOCATION) << "Distance at dt = 0 was low enough (" << minDist << ") that previous step is repeated with more new contacts!";
				if (_state->getRepetitions() < 5) {
					sample0.forwardContacts = _detector->findContacts(sampleH.rwstate,cdData,sample0.forwardTrack,_log->makeScope("Find Contacts",RWPE_LOCATION));
					RWPEUtil::mark(sample0.forwardContacts,sample0.forwardTrack,RWPEUtil::MARK_RAW,RWPEUtil::MARK_NEW);
					sample0.forwardContacts = _detector->findContacts(sample0.rwstate,cdData,sample0.forwardTrack,_log->makeScope("Find Contacts",RWPE_LOCATION));
					//storeResults(cdData,sample0,sample0.rwstate);
					const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();
					position(sample0,cdData);
					const RWPEIslandState& islandState = sample0.islandState;
					*_state = islandState;
					_state->setLastTimeStep(0);
					_state->setRepetitions(_state->getRepetitions()+1);
					// Update RobWork bodies
					BOOST_FOREACH(const RWPEBody* body, bodies) {
						body->updateRW(sample0.rwstate,*_state);
					}
					// Keep only the new contacts for bouncing that are within a certain distance
					RWPEUtil::remove(sample0.forwardContacts,sample0.forwardTrack,RWPEUtil::MARK_RAW);
					if (doLog)
						_log->addContacts("All contacts for next step",sample0.forwardContacts,RWPE_LOCATION);
					RWPEUtil::removeContactsOutsideThreshold(sample0.forwardContacts,sample0.forwardTrack,0.0001,RWPEUtil::MARK_NEW);
					if (doLog)
						_log->addContacts("All contacts after removing contacts outside threshold",sample0.forwardContacts,RWPE_LOCATION);

					// Update the constraints
					RWPEUtil::updateTemporaryContacts(sample0.forwardContacts,sample0.forwardTrack,_bc,*_state,sample0.rwstate);

					// Store contact information in state
					_state->setContactData(cdData);
					_state->setContacts(sample0.forwardContacts,sample0.forwardTrack);

					return 0;
				} else {
					if (doLog)
						_log->log("Too many repetitions",RWPE_LOCATION) << " - as there have already been 5 repetitions there is probably an issue with tracking contacts - instead taking complete step.";
					RW_THROW("Too many repetitions!");
					//storeResults(cdData,sampleH,state);
				}
			} else {
				// Ordinary Rollback
				if (doLog)
					_log->log("Performing Ordinary Rollback",RWPE_LOCATION) << "Performing narrow-phase rollback on  " << contactsBefore.size() << " new contact(s).";
				const IntegrateSample sampleNew = rollbackNarrowPhase(discontinuity,sample0,sampleH,cdData);
				if (doLog)
					_log->log("New Timestep",RWPE_LOCATION) << "Rollback decreased timestep to " << sampleNew.time << " from " << sampleH.time << ".";
				sampleH.time = sampleNew.time;
				sampleH.rwstate = sampleNew.rwstate;
				sampleH.islandState = sampleNew.islandState;
				sampleH.forwardContacts = _detector->findContacts(sampleH.rwstate,cdData,sampleH.forwardTrack,_log->makeScope("Find Contacts",RWPE_LOCATION));
				//sampleH.forwardContacts = _detector->findContacts(sampleH.rwstate,cdData,sampleH.forwardTrack);
			}
		} else {
			// Tracking must have failed!
			if (doLog) {
				std::ostream& lstr = _log->log("Tracking failed",RWPE_LOCATION);
				lstr << "There are no contacts that are non-penetrating at dt = 0, hence rollback is impossible. Contact tracking must have failed.\n";
				lstr << " - one complete step is performed even though the penetration is too big.";
			}
		}
	}
	// Default if something fails
	return sampleH.time;
}

RWPEIsland::IntegrateSample RWPEIsland::rollbackNarrowPhase(bool discontinuity, IntegrateSample& sample0, IntegrateSample& sampleH, ContactDetectorData& cdData) const {
	const bool doLog = _log->doLog();

	double threshold;
	if (_map.has("RWPERollbackThreshold"))
		threshold = _map.get<double>("RWPERollbackThreshold");
	else
		threshold = _defaultMap.get<double>("RWPERollbackThreshold");

	int iterations;
	if (_map.has("RWPERollbackIterations"))
		iterations = _map.get<int>("RWPERollbackIterations");
	else
		iterations = _defaultMap.get<int>("RWPERollbackIterations");
	RW_ASSERT(iterations >= 0);

	const std::vector<std::size_t> newContactIds0 = RWPEUtil::getMarkedContacts(sample0.backwardContacts,sample0.backwardTrack,RWPEUtil::MARK_RAW);
	const std::vector<std::size_t> newContactIdsH = RWPEUtil::getMarkedContacts(sampleH.backwardContacts,sampleH.backwardTrack,RWPEUtil::MARK_RAW);
	const std::vector<Contact> newContacts0 = RWPEUtil::getContacts(sample0.backwardContacts,newContactIds0);
	const std::vector<Contact> newContactsH = RWPEUtil::getContacts(sampleH.backwardContacts,newContactIdsH);
	const RWPERollbackMethod::Sample rollbackSample0(sample0.time,newContacts0);
	const RWPERollbackMethod::Sample rollbackSampleH(sampleH.time,newContactsH);

	std::string methodID;
	if (_map.has("RWPERollbackMethod"))
		methodID = _map.get<std::string>("RWPERollbackMethod");
	else
		methodID = _defaultMap.get<std::string>("RWPERollbackMethod");
	if (doLog)
		_log->log("Rollback Method",RWPE_LOCATION) << "Chosen Rollback Method: " << methodID;
	const RWPERollbackMethod::Ptr rollback = RWPERollbackMethod::Factory::makeMethod(methodID);
	RW_ASSERT(!(rollback == NULL));
	RWPERollbackMethod::RollbackData* rollbackData = rollback->createData();
	RWPERollbackMethod::SampleSet rollbackSamples;
	rollbackSamples.insert(rollbackSample0);
	rollbackSamples.insert(rollbackSampleH);

	bool doRollback = false;
	IntegrateSample sampleNew = sample0;
	_log->beginSection("Iterative Search",RWPE_LOCATION);
	for (unsigned int iteration = 1; iteration <= (unsigned int)iterations; iteration++) {
		doRollback = true;
		RW_ASSERT(rollbackSamples.size() >= 2);
		const double dtTry = rollback->getTimestep(rollbackSamples,rollbackData);
		RW_ASSERT(dtTry > rollbackSamples.begin()->time);
		RW_ASSERT(dtTry < rollbackSamples.rbegin()->time);

		// For logging
		std::vector<double> values;
		std::vector<std::string> labels;
		if (doLog) {
			labels.push_back("Iteration");
			labels.push_back("Test Step-size");
			values.push_back(iteration);
			values.push_back(dtTry);
		}

		RWPEUtil::positionUpdate(dtTry, _gravity, discontinuity, _bc, sampleNew.islandState, sampleNew.rwstate);
		if (doLog)
			_log->addPositions("Positions",_bc,sampleNew.islandState,RWPE_LOCATION);
		sampleNew.backwardTrack = sampleH.backwardTrack;
		sampleNew.backwardContacts = _detector->updateContacts(sampleNew.rwstate,cdData,sampleNew.backwardTrack,_log->makeScope("Update Contacts",RWPE_LOCATION));
		sampleNew.time = dtTry;
		RWPEUtil::removeKnown(sampleNew.backwardContacts,sampleNew.backwardTrack,_bc,sampleNew.islandState);
		if (sampleNew.backwardContacts.size() > 0) {
			const double minDist = RWPEUtil::minDistance(sampleNew.backwardContacts);
			if (doLog) {
				labels.push_back("Minimum distance");
				values.push_back(minDist);
			}
			if (fabs(minDist) < threshold) {
				if (doLog)
					_log->log("Done",RWPE_LOCATION) << "Distance is less than " << threshold << " (" << fabs(minDist) << ") - rollback done.";
				doRollback = false;
			} else {
				//if (minDist < 0)
				//	RWPEUtil::removeNonPenetrating(sampleNew.backwardContacts,sampleNew.backwardTrack,RWPEUtil::MARK_RAW);
				if (doLog)
					_log->addContacts("Rollback Contacts",sampleNew.backwardContacts,RWPE_LOCATION);
				const RWPERollbackMethod::Sample rollbackSample = RWPERollbackMethod::Sample(dtTry,sampleNew.backwardContacts);
				rollbackSamples.insert(rollbackSample);
			}
		} else {
			if (doLog)
				_log->log("Failed",RWPE_LOCATION) << " - lost all contacts - rollback done." << methodID;
			doRollback = false;
		}

		if (doLog)
			_log->addValues("Values",values,labels,RWPE_LOCATION);

		if (!doRollback)
			break;

		if (iteration == (unsigned int)iterations) {
			std::stringstream err;
			err << "Maximum of " << iterations << " iterations exceeded!";
			_log->log("Fatal Failure",RWPE_LOCATION) << err.str();
			RW_THROW("RWPEIsland (doStep): " << err.str());
		}

		sampleNew = sample0;
	}
	_log->endSection(__LINE__);
	delete rollbackData;

	if (doLog)
		_log->addPositions("New Positions",_bc,sampleNew.rwstate,RWPE_LOCATION);

	return sampleNew;
}

void RWPEIsland::constraintSolver(double dt, bool discontinuity, const RWPEIslandState& islandState0, RWPEIslandState& islandStateH, const State& rwstate) const {
	std::string solverID;
	if (_map.has("RWPEConstraintSolver"))
		solverID = _map.get<std::string>("RWPEConstraintSolver");
	else
		solverID = _defaultMap.get<std::string>("RWPEConstraintSolver");
	const RWPEConstraintSolver* const solver = RWPEConstraintSolver::Factory::makeSolver(solverID,_bc,_gravity);
	std::string resolverID;
	if (_map.has("RWPEContactResolver"))
		resolverID = _map.get<std::string>("RWPEContactResolver");
	else
		resolverID = _defaultMap.get<std::string>("RWPEContactResolver");
	const RWPEContactResolver* const resolver = RWPEContactResolver::Factory::makeResolver(resolverID,solver);
	if (resolver == NULL)
		RW_THROW("RWPEIsland (constraintSolver): the RWPEContactResolver with name \"" << resolverID << "\" could not be created!");
	resolver->solve(std::vector<RWPEContact*>(),dt,discontinuity,*_materialMap,rwstate,islandState0,islandStateH,_map,_log);
	delete resolver;
	delete solver;

	// Check size of forces
	double maxForce;
	if (_map.has("RWPEConstraintMaxForce"))
		maxForce = _map.get<double>("RWPEConstraintMaxForce");
	else
		maxForce = _defaultMap.get<double>("RWPEConstraintMaxForce");
	const RWPEBodyConstraintGraph::ConstraintList constraints = _bc->getConstraints(islandStateH);
	BOOST_FOREACH(const RWPEConstraint* const constraint, constraints) {
		const double force = constraint->getWrench(islandStateH).force().norm2();
		if (force > maxForce) {
			RW_THROW("Constraint force on " << force << "N exceeded limit of " << maxForce << "N.");
		}
	}

	if (_log->doLog()) {
		const RWPEBodyConstraintGraph::ConstraintList contacts = _bc->getTemporaryConstraints(&islandStateH);
		if (contacts.size() > 0) {
			std::vector<const RWPEContact*> rwpecontacts;
			BOOST_FOREACH(const RWPEConstraint* const constraint, contacts) {
				if (const RWPEContact* const c = dynamic_cast<const RWPEContact*>(constraint)) {
					rwpecontacts.push_back(c);
				}
			}
			_log->addContacts("Contact Positions",rwpecontacts,RWPE_LOCATION);
			_log->addContactWrenches("Contact Wrench (applied)",rwpecontacts,islandStateH,RWPELogUtil::APPLIED,RWPE_LOCATION);
			_log->addContactWrenches("Contact Wrench (constraint)",rwpecontacts,islandStateH,RWPELogUtil::CONSTRAINT,RWPE_LOCATION);
			_log->addContactWrenches("Contact Wrench (total)",rwpecontacts,islandStateH,RWPELogUtil::TOTAL,RWPE_LOCATION);
		}
	}
}

void RWPEIsland::storeResults(ContactDetectorData& cdData, IntegrateSample& sample, State& rwstate) const {
	const bool doLog = _log->doLog();

	// Update tactile sensors
	if (sample.time > 0) {
		_log->beginSection("Sensor Update",RWPE_LOCATION);
		RWPEUtil::updateSensors(_sensors,_state->getTime(),sample.time,_state->getLastTimeStep(),_bc,*_state,*_state,rwstate,*_log);
		_log->endSection(__LINE__);
	}

	const RWPEBodyConstraintGraph::BodyList bodies = _bc->getBodies();

	position(sample,cdData);

	const RWPEIslandState& islandState = sample.islandState;
	*_state = islandState;
	_state->setLastTimeStep(sample.time);
	if (sample.time == 0)
		_state->setRepetitions(_state->getRepetitions()+1);
	else
		_state->setRepetitions(0);
	_state->setTime(_state->getTime()+sample.time);
	rwstate = sample.rwstate;

	// Update RobWork bodies
	BOOST_FOREACH(const RWPEBody* body, bodies) {
		body->updateRW(rwstate,*_state);
	}

	// Keep only the new contacts for bouncing that are within a certain distance
	RWPEUtil::remove(sample.forwardContacts,sample.forwardTrack,RWPEUtil::MARK_RAW);
	if (doLog)
		_log->addContacts("All contacts for next step",sample.forwardContacts,RWPE_LOCATION);
	RWPEUtil::removeContactsOutsideThreshold(sample.forwardContacts,sample.forwardTrack,0.0001,RWPEUtil::MARK_NEW);
	if (doLog)
		_log->addContacts("All contacts after removing contacts outside threshold",sample.forwardContacts,RWPE_LOCATION);

	// Update the constraints
	RWPEUtil::updateTemporaryContacts(sample.forwardContacts,sample.forwardTrack,_bc,*_state,rwstate);

	// Store contact information in state
	_state->setContactData(cdData);
	_state->setContacts(sample.forwardContacts,sample.forwardTrack);
}
