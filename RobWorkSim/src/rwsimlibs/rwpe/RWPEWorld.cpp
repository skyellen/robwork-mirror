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
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEBodyDynamic.hpp"
#include "RWPEIsland.hpp"
#include "RWPEWorld.hpp"

#include "RWPEBroadPhase.hpp"
#include "RWPEContact.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwlibs::simulation;
using namespace rwsim::contacts;
using namespace rwsim::drawable;
using namespace rwsim::dynamics;
using namespace rwsimlibs::rwpe;

RWPEWorld::RWPEWorld(const std::string &name):
	_dwc(NULL),
	_name(name),
	_bc(NULL),
	_bp(NULL),
	_time(0)
{
}

RWPEWorld::~RWPEWorld() {
	exitPhysics();
}

void RWPEWorld::load(DynamicWorkCell::Ptr dwc) {
	if (!(_dwc == NULL))
		RW_THROW("RWPEWorld (load): dynamic workcell has already been loaded!");
	RW_ASSERT(dwc != NULL);
	_dwc = dwc;
	//_materialMap = new RWPEMaterialMap(dwc->getContactData(),dwc->getMaterialData());
	//_detector = ContactDetector::makeDefault(dwc->getWorkcell());
	//RWPE_DEBUG_CONTACTS("Default contact detector constructed based on dynamic workcell:" << std::endl << _detector);
	//std::cout << "Default contact detector constructed based on dynamic workcell:" << std::endl << _detector << std::endl;
	_map = dwc->getEngineSettings();
}

void RWPEWorld::load(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc) {
	if (!(_dwc == NULL))
		RW_THROW("RWPEWorld (load): dynamic workcell has already been loaded!");
	RW_ASSERT(!(dwc == NULL));
	_dwc = dwc;
	_map = dwc->getEngineSettings();
}

bool RWPEWorld::setContactDetector(ContactDetector::Ptr detector) {
	return false;
}

void RWPEWorld::step(double dt, double t, State &state, ThreadTask::Ptr task, bool forceTime) {
	BOOST_FOREACH(RWPEIsland* island, _islands) {
		island->step(dt,state);
	}
}

void RWPEWorld::step(double dt, State& state) {
}

void RWPEWorld::resetScene(State &state) {
}

void RWPEWorld::initPhysics(State &state) {
	if (_dwc == NULL)
		RW_THROW("RWPEWorld (initPhysics): no dynamic workcell given - please use load first!");
	if (_bc != NULL)
		RW_THROW("RWPEWorld (initPhysics): was called earlier!");
	// Create the root body-constraint manager
	_bc = new RWPEBodyConstraintGraph();
	_bc->initFromDWC(_dwc);

	// Create Broad-Phase collision checking
	if (_bp != NULL)
		delete _bp;
	_bp = new RWPEBroadPhase(_dwc);
	BOOST_FOREACH(const RWPEBody* body, _bc->getBodies()) {
		_bp->addObject(body->get()->getObject());
	}

	// Add controllers
	//const DynamicWorkCell::ControllerList& controllers = _dwc->getControllers();
	//BOOST_FOREACH(SimulatedController::Ptr controller, controllers){
	//	addController(controller);
	//}

	// Add sensors
    BOOST_FOREACH(SimulatedSensor::Ptr sensor, _dwc->getSensors()){
    	addSensor(sensor, state);
	}

	resetScene(state);
}

void RWPEWorld::initPhysics(State &state, RWPEBodyConstraintGraph* bc) {
	if (_dwc == NULL)
		RW_THROW("RWPEWorld (initPhysics): no dynamic workcell given - please use load first!");
	if (_bc != NULL)
		RW_THROW("RWPEWorld (initPhysics): was called earlier!");
	// Instead of creating a root body-constraint manager, we work only on the component given as input
	_bc = bc;
	// Create Broad-Phase collision checking that only included the bodies we are working with
	if (_bp != NULL)
		delete _bp;
	_bp = new RWPEBroadPhase(_dwc);
	BOOST_FOREACH(const RWPEBody* const body, _bc->getBodies()) {
		_bp->addObject(body->get()->getObject());
	}

	ProximityFilterStrategy* const filter = _bp->getProximityFilterStrategy();
	const ProximityFilter::Ptr filterUp = filter->update(state);
	std::cout << "Frame pairs for collision check in " << _name << std::endl;
	while (!filterUp->isEmpty()) {
		const FramePair pair = filterUp->frontAndPop();
		std::cout << " - " << pair.first->getName() << " - " << pair.second->getName() << std::endl;
	}

	// Add controllers
	//const DynamicWorkCell::ControllerList& controllers = _dwc->getControllers();
	//BOOST_FOREACH(SimulatedController::Ptr controller, controllers){
	//	addController(controller);
	//}

	// Add sensors
    //BOOST_FOREACH(SimulatedSensor::Ptr sensor, _dwc->getSensors()){
    //	addSensor(sensor, state);
	//}

	resetScene(state);
}

void RWPEWorld::exitPhysics() {
	delete _bc;
	_bc = NULL;
	for (std::list<RWPEIsland*>::iterator it = _islands.begin(); it != _islands.end(); it++) {
		delete *it;
	}
	_islands.clear();
}

double RWPEWorld::getTime() {
	return _time;
}

void RWPEWorld::setEnabled(Body::Ptr body, bool enabled) {
}

void RWPEWorld::setDynamicsEnabled(Body::Ptr body, bool enabled) {
}

SimulatorDebugRender::Ptr RWPEWorld::createDebugRender() {
	return NULL;
}

PropertyMap& RWPEWorld::getPropertyMap() {
	return _map;
}

void RWPEWorld::emitPropertyChanged() {
}

void RWPEWorld::addController(SimulatedController::Ptr controller) {
}

void RWPEWorld::removeController(SimulatedController::Ptr controller) {
}

void RWPEWorld::addBody(Body::Ptr body, State &state) {
}

void RWPEWorld::addDevice(DynamicDevice::Ptr dev, State &state) {
}

void RWPEWorld::addSensor(SimulatedSensor::Ptr sensor, State &state) {
}

void RWPEWorld::removeSensor(SimulatedSensor::Ptr sensor) {
}

void RWPEWorld::attach(Body::Ptr b1, Body::Ptr b2) {
}

void RWPEWorld::detach(Body::Ptr b1, Body::Ptr b2) {
}

std::vector<SimulatedSensor::Ptr> RWPEWorld::getSensors() {
	std::vector<SimulatedSensor::Ptr> none;
	return none;
}

std::list<RWPEIsland*> RWPEWorld::getIslands() const {
	return _islands;
}

void RWPEWorld::addBody(const RWPEBody* body) {
	_bc->addBody(body);
}

void RWPEWorld::removeBody(const RWPEBody* body) {
	_bc->removeBody(body);
}

std::string RWPEWorld::getName() const {
	return _name;
}

void RWPEWorld::setName(const std::string &name) {
	_name = name;
}

const RWPEBodyConstraintGraph* RWPEWorld::getManager() const {
	return _bc;
}
