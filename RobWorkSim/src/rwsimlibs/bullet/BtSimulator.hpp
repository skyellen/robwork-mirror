/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_BULLET_BTSIMULATOR_HPP_
#define RWSIMLIBS_BULLET_BTSIMULATOR_HPP_

/**
 * @file BtSimulator.hpp
 *
 * \copydoc rwsimlibs::bullet::BtSimulator
 */

#include <rwsim/simulator/PhysicsEngine.hpp>

// Forward Declarations
class btDynamicsWorld;
class btDiscreteDynamicsWorld;
class btTypedConstraint;
class btBroadphaseInterface;
class btCollisionDispatcher;
class btConstraintSolver;
class btCollisionConfiguration;

namespace rwsim { namespace contacts { class ContactDetectorData; } }

namespace rwsimlibs {
namespace bullet {
class BtBody;
class BtDevice;
class BtConstraint;
class BtTactileSensor;

//! @addtogroup rwsimlibs_bullet
/**
 * @brief A physics engine that uses Bullet Physics as the underlying engine.
 */
class BtSimulator: public rwsim::simulator::PhysicsEngine {
public:
	//! Construct new simulator.
	BtSimulator();

	/**
	 * @brief Construct new simulator.
	 * @param dwc [in] the dynamic workcell.
	 */
	BtSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

	//! @brief Destructor.
	virtual ~BtSimulator();

	//! @copydoc PhysicsEngine::load
	void load(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

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

	//! @copydoc PhysicsEngine::createDebugRender
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
	void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::addDevice
	void addDevice(rwsim::dynamics::DynamicDevice::Ptr device, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::addSensor
	void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State& state);

	//! @copydoc PhysicsEngine::removeSensor
	void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

	//! @copydoc PhysicsEngine::attach
	void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::detach
	void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

	//! @copydoc PhysicsEngine::getSensors
	std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors();

	/**
	 * @brief Add a Constraint between two bodies.
	 * @param constraint [in] a pointer to the RobWork constraint.
	 */
	void addConstraint(rw::common::Ptr<const rwsim::dynamics::Constraint> constraint);

private:
	rw::common::PropertyMap _propertyMap;
	std::vector<rwlibs::simulation::SimulatedSensor::Ptr> _sensors;

	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btBroadphaseInterface* m_overlappingPairCache;
	btCollisionDispatcher* m_dispatcher;
	btConstraintSolver* m_solver;
	btCollisionConfiguration* m_collisionConfiguration;

	rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	rwsim::dynamics::MaterialDataMap _materialMap;
	rwsim::dynamics::ContactDataMap _contactMap;

	std::vector<rwsimlibs::bullet::BtBody*> _btBodies;
	std::map<const rw::kinematics::Frame*, rwsimlibs::bullet::BtBody*> _rwFrameToBtBody;
	std::map<rwsimlibs::bullet::BtBody*, rw::kinematics::Frame*> _rwBtBodyToFrame;

	std::vector<BtConstraint*> _constraints;
	std::vector<BtTactileSensor*> _btSensors;

	std::vector<rwsim::dynamics::DynamicDevice::Ptr> _devices;

	std::vector<BtDevice*> _btDevices;

	std::map<rw::models::Joint*, btTypedConstraint*> _jointToConstraintMap;

	rwsim::drawable::SimulatorDebugRender::Ptr _render;

	std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;

	rw::common::Ptr<rwsim::contacts::ContactDetector> _detector;
	rw::common::Ptr<rwsim::contacts::ContactDetectorData> _detectorData;

	double _time, _dt;
	bool _initPhysicsHasBeenRun;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTSIMULATOR_HPP_ */
