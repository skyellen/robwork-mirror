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

#ifndef RWSIM_SIMULATOR_SIMULATOR_HPP_
#define RWSIM_SIMULATOR_SIMULATOR_HPP_

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>

#include <rwlibs/simulation/Simulator.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include "PhysicsEngine.hpp"

#include <rw/kinematics/State.hpp>
#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/drawable/SimulatorDebugRender.hpp>

namespace rwsim {
namespace simulator {
	//! @addtogroup simulator @{

	/**
	 * @brief A general physics engine interface for simulating dynamics
	 * of objects and robot devices.
	 *
	 * The general step looks like this:;
	 *
	 * Foreach controller=_controllers
	 *  controller->update(dt,state);
     *
	 * physicsEngine->step(dt, state);
	 *
	 * Foreach sensor=_sensors
	 *  sensor->update(dt,state)
	 *
	 */
	class DynamicSimulator: public rwlibs::simulation::Simulator
	{
	public:

	    typedef rw::common::Ptr<DynamicSimulator> Ptr;

	    DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell, PhysicsEngine::Ptr pengine);

        DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell);

		/**
		 * @brief destructor
		 */
		virtual ~DynamicSimulator(){};

		/**
		 * @brief cleans up the allocated storage fo bullet physics
		 */
		void exitPhysics();

		/**
		 * @brief gets the the current simulated time
		 */
		double getTime();

		/**
		 * Enables or disables a body
		 * @param body
		 * @param enabled
		 */
		void setEnabled(dynamics::Body* body, bool enabled);

		/**
		 * @brief create a debug render for the specific implementation
		 * @return NULL if no render is available else a valid render
		 */
		drawable::SimulatorDebugRender::Ptr createDebugRender();

		/**
		 * @brief
		 */
		rw::common::PropertyMap& getPropertyMap();

		/**
		 * @brief add a simulated controller to this simulator
		 */
		void addController(rwlibs::simulation::SimulatedController::Ptr controller);

		/**
		 * @brief removes a simulated controller from this simulator
		 * @param controller
		 */
		void removeController(rwlibs::simulation::SimulatedController::Ptr controller);

		/**
		 * @brief add a simulated sensor to this simulator
		 */
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

		/**
		 * @brief add a simulated sensor to this simulator
		 */
		void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

		/**
		 * @brief get the list of simulated sensors
		 * @return
		 */
		 std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors();

		 // Simulator interface

         /**
          * @copydoc Simulator::step
          */
         void step(double dt, rw::kinematics::State& state);

         /**
          * @copydoc Simulator::reset
          *
          * @note resets position, velocity and acceleration of all bodies to that defined in state or
          * by the default behavior of body/device interfaces.
          */
         void reset(rw::kinematics::State& state);

         /**
          * @copydoc Simulator::init
          * @note initialize simulator physics with state
          */
		 void init(rw::kinematics::State& state);

		 /**
          * @copydoc Simulator::setEnabled
          * @note this only has an effect if the frame \b f is successfully mapped to any
          * fo the bodies in the scene eg. a body frame of one of the bodies.
		  */
		 void setEnabled(rw::kinematics::Frame* f, bool);

	private:
		 rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		 PhysicsEngine::Ptr _pengine;
	};

	//! @}
}
}
#endif /*Simulator_HPP_*/

