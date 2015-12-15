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

#include "RWPEBody.hpp"
#include "RWPEBodyConstraintGraph.hpp"
#include "RWPEPhysics.hpp"
#include "RWPEWorld.hpp"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwlibs::simulation;

using namespace rwsim::contacts;
using namespace rwsim::drawable;
using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rwsimlibs::rwpe;

class RWPEPhysics::MainThread: public ThreadTask {
public:
	MainThread(RWPEPhysics* engine, double dt, const State &state, ThreadTask::Ptr parent):
		ThreadTask(parent),
		_engine(engine),
		_dt(dt),
		_state(state)
	{
	}

	void run() {
		_engine->doStep(_dt,_state);
	}

private:
	RWPEPhysics* const _engine;
	const double _dt;
	State _state;
};

RWPEPhysics::RWPEPhysics():
	_dwc(NULL),
	_bc(NULL),
	_render(NULL),
	_time(0),
	_timeSync(false)
{
}

RWPEPhysics::~RWPEPhysics() {
	if (_bc != NULL)
		exitPhysics();
}

void RWPEPhysics::load(DynamicWorkCell::Ptr dwc) {
	if (!(_dwc == NULL))
		RW_THROW("RWPEPhysics (load): dynamic workcell has already been loaded!");
	RW_ASSERT(dwc != NULL);
	_dwc = dwc;
	//_materialMap = new RWPEMaterialMap(dwc->getContactData(),dwc->getMaterialData());
	_detector = ContactDetector::makeDefault(dwc->getWorkcell());
	std::cout << "Default contact detector constructed based on dynamic workcell:" << std::endl << _detector << std::endl;
	_map = dwc->getEngineSettings();
}

bool RWPEPhysics::setContactDetector(ContactDetector::Ptr detector) {
	return false;
}

void RWPEPhysics::step(double dt, State& state) {
	step(dt,state,NULL);
}

void RWPEPhysics::step(double dt, State& state, ThreadTask::Ptr task) {
	if (task == NULL) {
		doStep(dt,state);
	} else {
		_task = ownedPtr(new MainThread(this,dt,state,task));
		task->addSubTask(_task);
	}
}

void RWPEPhysics::doStep(double dt, State& state) {
	if (_worlds.size() == 0)
		RW_THROW("Please call initPhysics before using the simulator.");
	_time += dt;
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		world.second->step(dt,_time,state,_task,_timeSync);
	}
}

void RWPEPhysics::resetScene(State& state) {
	_time = 0;
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		world.second->resetScene(state);
	}
}

#include <rw/proximity/BasicFilterStrategy.hpp>
void RWPEPhysics::initPhysics(State& state) {
	if (_dwc == NULL)
		RW_THROW("RWPEPhysics (initPhysics): no dynamic workcell given - please use load first!");
	if (_worlds.size() > 0)
		RW_THROW("RWPEPhysics (initPhysics): was called earlier!");
	// Create the root body-constraint manager
	_bc = new RWPEBodyConstraintGraph();
	_bc->initFromDWC(_dwc);
	// Use ProximitySetup in workcell to construct list of all pairs of objects that can ever be in contact
	BasicFilterStrategy filter(_dwc->getWorkcell());
	const ProximityFilter::Ptr filterUp = filter.update(state);
	std::list<std::pair<const RWPEBody*, const RWPEBody*> > pairs;
	while (!filterUp->isEmpty()) {
		const FramePair pair = filterUp->frontAndPop();
		const RWPEBody* const bodyA = _bc->getBody(pair.first);
		const RWPEBody* const bodyB = _bc->getBody(pair.second);
		RW_ASSERT(bodyA != NULL && bodyB != NULL);
		pairs.push_back(std::make_pair(bodyA,bodyB));
	}
	// Dynamic components are then extracted from the body-constraint map
	const std::set<RWPEBodyConstraintGraph*> components = _bc->getDynamicComponents(pairs);
	std::size_t i = 0;
	BOOST_FOREACH(RWPEBodyConstraintGraph* const component, components) {
		i++;
		std::stringstream sname;
		sname << "World-" << i;
		RWPEWorld* const newWorld = new RWPEWorld(sname.str());
		_worlds.insert(std::make_pair(sname.str(),newWorld));
		newWorld->load(_dwc);
		newWorld->initPhysics(state,component);
#if 0
		std::stringstream sbodies;
		BOOST_FOREACH(const RWPEBody* const body, component->getBodies()) {
			sbodies << " " << body->get()->getName();
		}
		RWPE_DEBUG_GENERAL("Created " << sname.str() << " with bodies:" << sbodies.str());
#endif
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

void RWPEPhysics::exitPhysics() {
	if (_bc == NULL)
		RW_THROW("RWPEPhysics (exitPhysics): initPhysics has not been called!");
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		delete world.second;
	}
	_worlds.clear();
	delete _bc;
	_bc = NULL;
}

double RWPEPhysics::getTime() {
	return _time;
}

void RWPEPhysics::setEnabled(Body::Ptr body, bool enabled) {
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		world.second->emitPropertyChanged();
	}
}

void RWPEPhysics::setDynamicsEnabled(Body::Ptr body, bool enabled) {

}

SimulatorDebugRender::Ptr RWPEPhysics::createDebugRender() {
	return NULL;
}

PropertyMap& RWPEPhysics::getPropertyMap() {
	return _map;
}

void RWPEPhysics::emitPropertyChanged() {
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		world.second->emitPropertyChanged();
	}
}

void RWPEPhysics::addController(SimulatedController::Ptr controller) {
	RW_THROW("RWPEPhysics (addController): This is not yet supported!");
}

void RWPEPhysics::removeController(SimulatedController::Ptr controller) {
	RW_THROW("RWPEPhysics (removeController): This is not yet supported!");
}

void RWPEPhysics::addBody(Body::Ptr body, State &state) {
	RW_THROW("RWPEPhysics (addBody): This is not yet supported!");
}

void RWPEPhysics::addDevice(DynamicDevice::Ptr dev, State &state) {
	RW_THROW("RWPEPhysics (addDevice): This is not yet supported!");
}

void RWPEPhysics::addSensor(SimulatedSensor::Ptr sensor, State &state) {
	const Frame* const bframe = sensor->getFrame();
	RW_ASSERT(bframe!=NULL);
	if(_bc->getBody(bframe) == NULL)
		RW_THROW("RWPEPhysics (addSensor): The frame (" << bframe->getName() << ") that the sensor is being attached to is not a body in the simulator! Did you remember to run initphysics?");
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		const RWPEBody* const body = world.second->getManager()->getBody(bframe);
		if (body != NULL) {
			world.second->addSensor(sensor,state);
		}
	}
}

void RWPEPhysics::removeSensor(SimulatedSensor::Ptr sensor) {
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		world.second->removeSensor(sensor);
	}
}

void RWPEPhysics::attach(Body::Ptr b1, Body::Ptr b2) {
	RW_THROW("RWPEPhysics (attach): This is not yet supported!");
}

void RWPEPhysics::detach(Body::Ptr b1, Body::Ptr b2) {
	RW_THROW("RWPEPhysics (detach): This is not yet supported!");
}

std::vector<SimulatedSensor::Ptr> RWPEPhysics::getSensors() {
	std::vector<SimulatedSensor::Ptr> ret;
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(WorldMap::value_type& world, _worlds) {
		BOOST_FOREACH(const SimulatedSensor::Ptr sensor, world.second->getSensors()) {
			bool found = false;
			BOOST_FOREACH(const SimulatedSensor::Ptr sensorB, ret) {
				if (sensor == sensorB) {
					found = true;
					break;
				}
			}
			if (!found) {
				ret.push_back(sensor);
			}
		}
	}
	return ret;
}

bool RWPEPhysics::createNewWorld(const std::string &name) {
	RW_THROW("RWPEPhysics (createNewWorld): This is not yet supported!");
	/*if (_worlds.count(name) == 0) {
		RWPEWorld* const world = new RWPEWorld(_dwc,name);
		_worlds.insert(std::make_pair<std::string,RWPEWorld*>(name,world));
		return true;
	} else
		return false;
		*/
}

bool RWPEPhysics::deleteWorld(const std::string &name) {
	RW_THROW("RWPEPhysics (deleteWorld): This is not yet supported!");
	/*
	if (_worlds.erase(name) > 0)
		return true;
	else
		return false;
		*/
}

std::vector<std::string> RWPEPhysics::getWorldNames() const {
	std::vector<std::string> res;
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(const WorldMap::value_type& world, _worlds) {
		res.push_back(world.second->getName());
	}
	return res;
}

std::vector<const RWPEWorld*> RWPEPhysics::getWorlds() const {
	std::vector<const RWPEWorld*> res;
	typedef std::map<std::string, RWPEWorld*> WorldMap;
	BOOST_FOREACH(const WorldMap::value_type& world, _worlds) {
		res.push_back(world.second);
	}
	return res;
}

bool RWPEPhysics::isTimeSynchronized() const {
	return _timeSync;
}

void RWPEPhysics::setTimeSynchronization(bool active) {
	_timeSync = active;
}
