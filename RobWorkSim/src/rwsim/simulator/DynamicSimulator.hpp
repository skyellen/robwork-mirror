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

#include <rwlibs/simulation/SimulatedSensor.hpp>

#include <rwlibs/simulation/Simulator.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <rw/trajectory/Trajectory.hpp>
#include <rwsim/drawable/SimulatorDebugRender.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rwlibs { namespace simulation { class SimulatedController; } }

namespace rwsim {
namespace simulator {
	class PhysicsEngine;

	//! @addtogroup rwsim_simulator
	//! @{

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
	    //! @brief smart pointer type of this class
	    typedef rw::common::Ptr<DynamicSimulator> Ptr;

	    /**
	     * @brief constructor
	     * @param dworkcell
	     * @param pengine
	     * @return
	     */
	    DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell, rw::common::Ptr<PhysicsEngine> pengine);

	    /**
	     * @brief constructor
	     * @param dworkcell
	     * @return
	     */
        DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell);

		/**
		 * @brief destructor
		 */
		virtual ~DynamicSimulator(){};

		/**
		 * @brief cleans up the allocated storage
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
		void setEnabled(dynamics::Body::Ptr body, bool enabled);

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
		void addController(rw::common::Ptr<rwlibs::simulation::SimulatedController> controller);

		/**
		 * @brief removes a simulated controller from this simulator
		 * @param controller
		 */
		void removeController(rw::common::Ptr<rwlibs::simulation::SimulatedController> controller);

		void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State &state);
		void addBody(rwsim::dynamics::Body::Ptr body){ addBody(body,_state); };

		void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State &state);
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev){ addDevice(dev,_state);};

		/**
		 * @brief add a simulated sensor to this simulator
		 */
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State &state);
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){ addSensor(sensor,_state);};

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
         void step(double dt);

         rw::kinematics::State& getState();

         /**
          * @copydoc Simulator::reset
          *
          * @note resets position, velocity and acceleration of all bodies to that defined in state or
          * by the default behavior of body/device interfaces.
          */
         void reset(const rw::kinematics::State& state);

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

		 void setDynamicsEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);
		 // interfaces for manipulating/controlling bodies

		 /**
		  * @brief set a target position of a body. This will add forces/velocities to a body such that it
		  * moves toward the target pose.
		  * @param body [in] the body to control
		  * @param t3d [in] the target pose
		  * @param state [in] current state
		  */
		 void setTarget(dynamics::Body::Ptr body, const rw::math::Transform3D<>& t3d, rw::kinematics::State& state);
		 void setTarget(dynamics::Body::Ptr body, const rw::math::Transform3D<>& t3d){ setTarget(body,t3d,_state);}

		 void setTarget(dynamics::Body::Ptr body, rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj);
		 /**
		  * @brief Set a velocity target.
		  * @param body [in] the body that should move.
		  * @param velocity [in] the velocity target.
		  */
		 void setTarget(dynamics::Body::Ptr body, const rw::math::VelocityScrew6D<>& velocity);

		 /**
		  * @brief disables the target control of body \b body.
		  * @param body
		  */
		 void disableBodyControl( dynamics::Body::Ptr body );

		 void disableBodyControl( );

		 rwsim::control::BodyController::Ptr getBodyController(){ return _bodyController; }

		 void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		 void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		 rwsim::dynamics::DynamicWorkCell::Ptr getDynamicWorkCell(){ return _dwc; }
	private:
		 rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		 rw::common::Ptr<PhysicsEngine> _pengine;
		 rwsim::control::BodyController::Ptr _bodyController;
		 rw::kinematics::State _state;
	};

	//! @}
}
}
#endif /*Simulator_HPP_*/

