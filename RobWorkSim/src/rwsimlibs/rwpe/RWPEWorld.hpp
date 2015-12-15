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

#ifndef RWSIMLIBS_RWPE_RWPEWORLD_HPP_
#define RWSIMLIBS_RWPE_RWPEWORLD_HPP_

/**
 * @file RWPEWorld.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEWorld
 */

#include <rw/common/Ptr.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

// Forward declarations
class RWPEBodyConstraintGraph;
class RWPEIsland;
class RWPEBody;
class RWPEContact;
class RWPEBroadPhase;

//! @{
/**
 * @brief The RWPEWorld engine.
 *
 * A RWPEWorld consist of objects that can never be in contact with objects in other
 * worlds. If only one world is needed, the RWPEWorld engine can be used right away.
 * If the dynamics can be split into multiple worlds, use the RWPEPhysics engine
 * which will manage multiple worlds automatically.
 */
class RWPEWorld: public rwsim::simulator::PhysicsEngine {
public:
    //! Smart pointer type of RWPEWorld
    typedef rw::common::Ptr<RWPEWorld> Ptr;

	/**
	 * @brief Create a new RWPEWorld physics engine.
	 * @param name [in] (optional) name identifying this world.
	 */
	RWPEWorld(const std::string &name = "");

	//! @brief Destructor
	virtual ~RWPEWorld();

	std::list<RWPEIsland*> getIslands() const;

	void addBody(const RWPEBody* body);

	void removeBody(const RWPEBody* body);

	//! @copydoc PhysicsEngine::load
	virtual void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	virtual void load(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	//! @copydoc PhysicsEngine::setContactDetector
	bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	void step(double dt, double until, rw::kinematics::State& state, rw::common::Ptr<rw::common::ThreadTask> task = NULL, bool forceTime = false);

	//! @copydoc PhysicsEngine::step
	void step(double dt, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::resetScene
	void resetScene(rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::initPhysics
	void initPhysics(rw::kinematics::State& state);

	void initPhysics(rw::kinematics::State& state, RWPEBodyConstraintGraph* bc);

	//! @copydoc PhysicsEngine::exitPhysics
	void exitPhysics();

	//! @copydoc PhysicsEngine::getTime
	double getTime();

	//! @copydoc PhysicsEngine::setEnabled
	void setEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);

	//! @copydoc PhysicsEngine::setDynamicsEnabled
	void setDynamicsEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);

	//! @copydoc PhysicsEngine::createDebugRenderer
	rwsim::drawable::SimulatorDebugRender::Ptr createDebugRender();

	//! @copydoc PhysicsEngine::getPropertyMap
	rw::common::PropertyMap& getPropertyMap();

	//! @copydoc PhysicsEngine::emitPropertyChanged
	void emitPropertyChanged();

	//! @copydoc PhysicsEngine::addController
	void addController(rwlibs::simulation::SimulatedController::Ptr controller);

	//! @copydoc PhysicsEngine::removeController
	void removeController(rwlibs::simulation::SimulatedController::Ptr controller);

	//! @copydoc PhysicsEngine::addBody
	void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::addDevice
	void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::addSensor
	void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State &state);

	//! @copydoc PhysicsEngine::removeSensor
	void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

	//! @copydoc PhysicsEngine::attach
	void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::detach
	void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::getSensors
	std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors();

	std::string getName() const;
	void setName(const std::string &name);

	const RWPEBodyConstraintGraph* getManager() const;

private:
	rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	std::string _name;

	RWPEBodyConstraintGraph* _bc;
	RWPEBroadPhase* _bp;

	std::list<RWPEIsland*> _islands;
	std::map<RWPEBody* const, RWPEIsland*> _dynBodyToIslandMap;
	std::map<RWPEBody* const, std::vector<RWPEIsland*> > _nondynBodyToIslandMap;

	double _time;
	rw::common::PropertyMap _map;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEWORLD_HPP_ */
