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

#ifndef RWSIMLIBS_RWPE_RWPEPHYSICS_HPP_
#define RWSIMLIBS_RWPE_RWPEPHYSICS_HPP_

/**
 * @file RWPEPhysics.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEPhysics
 */

#include <rw/common/Ptr.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

// Forward declarations
class RWPEWorld;
class RWPEPhysicsDebugRender;
class RWPEBodyConstraintGraph;

//! @{
/**
 * @brief The RWPEPhysics engine.
 *
 * Use this engine as an alternative to the Open Dynamics Engine.
 */
class RWPEPhysics: public rwsim::simulator::PhysicsEngine {
public:
    //! Smart pointer type of RWPEPhysics
	typedef rw::common::Ptr<RWPEPhysics> Ptr;

	//! @brief Construct empty engine.
	RWPEPhysics();

	//! @brief Default destructor
	virtual ~RWPEPhysics();

	//! @copydoc PhysicsEngine::load
	virtual void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	//! @copydoc PhysicsEngine::setContactDetector
	bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	//! @copydoc PhysicsEngine::step
	void step(double dt, rw::kinematics::State& state);

	/**
	 * @brief Perform a step asynchronously - assigns work to given ThreadTask.
	 *
	 * The work is assigned to the given task, which in turn has the ability to use a ThreadPool to parallelize it.
	 * Remember to request execution of the task afterwards.
	 * The state will be changing until task is finished.
	 *
	 * @param dt [in] the timestep in seconds.
	 * @param state [in/out] the state before the timestep is taken.
	 * @param task [in] (optional) pointer to the task to assign work to.
	 */
	void step(double dt, rw::kinematics::State& state, rw::common::Ptr<rw::common::ThreadTask> task = NULL);

	//! @copydoc PhysicsEngine::resetScene
	void resetScene(rw::kinematics::State& state);

	/**
	 * @copydoc PhysicsEngine::initPhysics
	 *
	 * Initialize one world automatically based on the given state.
	 */
	void initPhysics(rw::kinematics::State& state);

	/**
	 * @copydoc PhysicsEngine::exitPhysics
	 */
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

	/**
	 * @brief Add an additional independent world with the given name.
	 *
	 * @param name [in] the name of the new world.
	 * @return true if world did not already exist.
	 */
	bool createNewWorld(const std::string &name);

	/**
	 * @brief Delete an existing world.
	 *
	 * @param name [in] the name of the world to delete.
	 * @return true if world was found and deleted.
	 */
	bool deleteWorld(const std::string &name);

	/**
	 * @brief Get a list of all names of current worlds in the simulator.
	 *
	 * @return list of names.
	 */
	std::vector<std::string> getWorldNames() const;

	/**
	 * @brief Get a list of all current worlds in the simulator.
	 *
	 * @return list of pointers to the current worlds.
	 */
	std::vector<const RWPEWorld*> getWorlds() const;

	/**
	 * @brief See the current state of time synchronization.
	 * @see RWPEPhysics#setTimeSynchronization
	 *
	 * @return true if enabled, false otherwise.
	 */
	bool isTimeSynchronized() const;

	/**
	 * @brief Change the used strategy for synchronization of time across different worlds.
	 *
	 * If set to active, all worlds are forced to have the same time after a simulation step.
	 * If not active the time for each world are allowed to deviate with a maximum equal to the step size.
	 *
	 * @param active [in] whether or not to use time synchronization.
	 */
	void setTimeSynchronization(bool active = false);

private:
	class MainThread;
	void doStep(double dt, rw::kinematics::State& state);

	rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	RWPEBodyConstraintGraph* _bc;
    const rw::common::Ptr<RWPEPhysicsDebugRender> _render;
	const rw::common::PropertyMap _propertyMap;
	rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;

	rw::common::Ptr<rw::common::ThreadTask> _task;
	std::map<std::string, RWPEWorld*> _worlds;
	double _time;
	bool _timeSync;

	rw::common::PropertyMap _map;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEPHYSICS_HPP_ */
