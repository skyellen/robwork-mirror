/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIM_SWIG_REMOTETYPES_HPP_
#define RWSIM_SWIG_REMOTETYPES_HPP_

#include <rwlibs/swig/ScriptTypes.hpp>

#include <rwsim/contacts/ContactDetector.hpp>
#include <rwsim/contacts/ContactDetectorData.hpp>
#include <rwsim/contacts/ContactDetectorTracking.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/ContactStrategy.hpp>

#include <rwsim/control/BodyController.hpp>
#include <rwsim/control/PoseController.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>

#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/BodyUtil.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/FixedBody.hpp>

#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <rwsim/simulator/AssemblySimulator.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>

#include <rwsimlibs/ode/ODESimulator.hpp>

/**
#ifdef __cplusplus
extern "C" {
#endif
    int luaopen_rwsim(struct lua_State* L); // declare the wrapped module
#ifdef __cplusplus
}
#endif
*/

namespace rwsim {
namespace swig {

	// contacts
	typedef rwsim::contacts::ContactDetector ContactDetector;
	typedef rwsim::contacts::ContactDetectorData ContactDetectorData;
	typedef rwsim::contacts::ContactDetectorTracking ContactDetectorTracking;
	typedef rwsim::contacts::ContactStrategy ContactStrategy;
	typedef rwsim::contacts::Contact Contact;

	// control
	typedef rwsim::control::PoseController PoseController;
	typedef rwsim::control::PDParam PDParam;
	typedef rwsim::control::PDController PDController;
	typedef rwsim::control::BodyController BodyController;
	typedef rwsim::control::SerialDeviceController SerialDeviceController;

	// drawable

	// dynamics
    typedef rwsim::dynamics::DynamicWorkCell DynamicWorkCell;
    typedef rwsim::dynamics::Body Body;
    typedef rwsim::dynamics::RigidBody RigidBody;
    typedef rwsim::dynamics::FixedBody FixedBody;
    typedef rwsim::dynamics::Constraint Constraint;
    typedef rwsim::dynamics::Constraint::SpringParams SpringParams;
    typedef rwsim::dynamics::KinematicBody KinematicBody;
    typedef rwsim::dynamics::BodyInfo BodyInfo;
    typedef rwsim::dynamics::DynamicDevice DynamicDevice;
    typedef rwsim::dynamics::RigidDevice RigidDevice;
    typedef rwsim::dynamics::SuctionCup SuctionCup;

	// loaders
    typedef rwsim::loaders::DynamicWorkCellLoader DynamicWorkCellLoader;

	// rwphysics

	// sensor
    typedef rwsim::sensor::SimulatedFTSensor SimulatedFTSensor;

	// simulator
    typedef rwsim::simulator::AssemblySimulator AssemblySimulator;
    typedef rwsim::simulator::DynamicSimulator DynamicSimulator;
    typedef rwsim::simulator::ThreadSimulator ThreadSimulator;
    typedef rwsim::simulator::ThreadSimulator::StepCallback ThreadSimulatorStepCallback;
    typedef rwsim::simulator::PhysicsEngine PhysicsEngine;
    typedef rwsim::simulator::PhysicsEngine::Factory PhysicsEngineFactory;
    typedef rwsim::simulator::GraspTaskSimulator GraspTaskSimulator;

	// util

	// rwsimlibs bullet

	// rwsimlibs gui

	// rwsimlibs ode
    typedef rwsim::simulator::ODESimulator ODESimulator;

	// rwsimlibs plugins

	// rwsimlibs swig

	// rwsimlibs tools

	// helper functions
    template <typename T>
    std::string toString(const T& x)
    {
        std::ostringstream buf;
        buf << x;
        return buf.str();
    }

    // general functions
    rw::common::Ptr<DynamicWorkCell> getDynamicWorkCell();

    /**
     * @brief set current dynamic workcell instance
     */
    void setDynamicWorkCell(rw::common::Ptr<DynamicWorkCell> dwc);

    /**
     * @brief add instance of simulator
     * @param sim [in]
     * @param id [in] id of simulator
     */
    void addSimulatorInstance(rw::common::Ptr<ThreadSimulator> sim, const std::string& id);

    //! @brief get first available simulator instance
    rw::common::Ptr<ThreadSimulator> getSimulatorInstance();

    rw::common::Ptr<ThreadSimulator> getSimulatorInstance(const std::string& id);

    void removeSimulatorInstance(const std::string& id);

    std::vector<std::string> getSimulatorInstances();

}
}

#endif /* REMOTETYPES_HPP_ */
