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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTISLAND_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTISLAND_HPP_

/**
 * @file Island.hpp
 *
 * \copydoc rwsimlibs::tntphysics::Island
 */

#include <rw/common/Ptr.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>

// Forward declarations
namespace rw { namespace common { class ThreadTask; } }
namespace rwsim { namespace contacts { class ContactDetector; } }
namespace rwsim { namespace contacts { class ContactDetectorData; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTBroadPhase;
class TNTBodyConstraintManager;
class TNTConstraintCorrection;
class TNTIslandState;
class TNTMaterialMap;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The TNTIsland engine.
 *
 * A TNTIsland is a basic engine that will solve for all bodies, constraints and contacts in one
 * large equation system. If the system can be decomposed into smaller independent TNTIslands,
 * it is the responsibility of the user to do this.
 *
 * Under normal circumstances this engine should not be used by the user directly, as it will be
 * too computationally expensive to solve dynamics for large dynamic workcells as one unit.
 * Instead the TNTWorld engine should be used, which will decompose and manage independent
 * TNTIslands dynamically to achieve best possible performance. It will also manage time
 * synchronization between islands.
 *
 * Consider using the TNTIsland directly in rare circumstances if the dynamic system is extremely
 * simple, and the overhead in managing independent islands would be too big.
 */
class TNTIsland: public rwsim::simulator::PhysicsEngine {
public:
    //! Smart pointer type of TNTIsland
    typedef rw::common::Ptr<TNTIsland> Ptr;

	//! @brief Create a new empty TNTIsland.
	TNTIsland();

	/**
	 * @brief Create a new empty TNTIsland.
	 * @param detector [in] the contact detector to use.
	 */
	TNTIsland(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	/**
	 * @brief Create a new TNTIsland physics engine based on a complete dynamic workcell.
	 * @param dwc [in] the dynamic workcell to create island from.
	 * @param detector [in] (optional) the contact detector to use. If none given a default detector is created.
	 */
	TNTIsland(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, rw::common::Ptr<rwsim::contacts::ContactDetector> detector = NULL);

	//! @brief Destructor
	~TNTIsland();

	/**
	 * @brief Get the currently used gravity in world frame.
	 * @return the gravity.
	 */
	const rw::math::Vector3D<>& getGravity() const;

	/**
	 * @brief Set the gravity.
	 * @param gravity [in] the gravity given in world frame.
	 */
	void setGravity(const rw::math::Vector3D<> &gravity);

	/**
	 * @brief Step the simulator by adding work to ThreadTask.
	 *
	 * The work is added as a single subtask to the given ThreadTask.
	 * @note This functions returns immediately and the state should not be accessed
	 * before work added to ThreadTask has finished.
	 *
	 * @param dt [in] the timestep.
	 * @param state [in/out] the state.
	 * @param task [in] the task to add work to.
	 */
	void step(double dt, rw::kinematics::State& state, rw::common::Ptr<rw::common::ThreadTask> task);

	/**
	 * @brief Get a copy of the internal state of the engine.
	 * @return a copy of the state.
	 */
	TNTIslandState getState() const;

	/**
	 * @brief Reset the internal engine state.
	 * @param state [in] the state to reset to.
	 */
	void resetScene(const TNTIslandState &state);

	//! @copydoc PhysicsEngine::load
	virtual void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	//! @copydoc PhysicsEngine::setContactDetector
	bool setContactDetector(rw::common::Ptr<rwsim::contacts::ContactDetector> detector);

	//! @copydoc PhysicsEngine::step
	void step(double dt, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::resetScene
	void resetScene(rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::initPhysics
	void initPhysics(rw::kinematics::State& state);

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

	//! @copydoc PhysicsEngine::getPropertyMap
	const rw::common::PropertyMap& getPropertyMap() const;

	/**
	 * @brief Get a map with the default properties.
	 * @return default PropertyMap.
	 */
	const rw::common::PropertyMap& getPropertyMapDefault() const;

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

private:
	static rw::common::PropertyMap getDefaultPropertyMap();

	class StepTask;
	struct IntegrateSample;
	struct IntegrateSampleCompare {
		bool operator()(const IntegrateSample& s1, const IntegrateSample& s2) const;
	};
	typedef std::set<IntegrateSample, IntegrateSampleCompare> IntegrateBuffer;

	void doStep(double dt, rw::kinematics::State& state);
	void solveConstraints(double dt, TNTIslandState& tntstate, const rw::kinematics::State& rwstate) const;
	double integrateBroadPhase(double dt, const IntegrateSample& first, IntegrateSample& res, rwsim::contacts::ContactDetectorData& cdData) const;
	IntegrateSample integrateRollback(const IntegrateSample& sample0, const IntegrateSample& sampleH, rwsim::contacts::ContactDetectorData& cdData) const;
	void storeResults(rwsim::contacts::ContactDetectorData& cdData, IntegrateSample& sample, rw::kinematics::State& rwstate) const;

	const TNTConstraintCorrection* const _correction;
	std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;

	rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	const TNTMaterialMap* _materialMap;

	TNTBroadPhase* _bp;
	rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;
	std::list<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;

	rw::common::Ptr<rw::common::ThreadTask> _task;
	TNTBodyConstraintManager* _bc;
	TNTIslandState* _state;
	rw::math::Vector3D<> _gravity;

	rw::common::PropertyMap _map;
	rw::common::PropertyMap _defaultMap;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTISLAND_HPP_ */
