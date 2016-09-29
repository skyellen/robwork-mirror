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
	     * @brief Constructor.
	     * @param dworkcell [in] the dynamic workcell.
	     * @param pengine [in] the physics engine to use.
	     */
	    DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell, rw::common::Ptr<PhysicsEngine> pengine);

	    /**
	     * @brief Constructor for a DynamicSimulator using a default PhysicsEngine.
	     * @param dworkcell [in] the dynamic workcell.
	     */
        DynamicSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dworkcell);

		/**
		 * @brief destructor
		 */
		virtual ~DynamicSimulator(){}

		/**
		 * @brief cleans up the allocated storage
		 */
		void exitPhysics();

		/**
		 * @brief gets the the current simulated time
		 * @return current simulated time.
		 */
		double getTime();

		/**
		 * @brief Enables or disables a body
		 * @param body [in] the body.
		 * @param enabled [in] boolean indicating whether body should be enabled or disabled.
		 */
		void setEnabled(dynamics::Body::Ptr body, bool enabled);

		/**
		 * @brief create a debug render for the specific implementation
		 * @return NULL if no render is available else a valid render
		 */
		drawable::SimulatorDebugRender::Ptr createDebugRender();

		/**
		 * @brief Get the properties used by the simulator.
		 * @return a reference to the properties.
		 */
		rw::common::PropertyMap& getPropertyMap();

		/**
		 * @brief add a simulated controller to this simulator
		 * @param controller [in] the controller to add.
		 */
		void addController(rw::common::Ptr<rwlibs::simulation::SimulatedController> controller);

		/**
		 * @brief removes a simulated controller from this simulator
		 * @param controller [in] the controller to remove.
		 */
		void removeController(rw::common::Ptr<rwlibs::simulation::SimulatedController> controller);

		/**
		 * @copydoc addBody(rwsim::dynamics::Body::Ptr)
		 * @param state [in] the state giving the initial pose of the body.
		 */
		void addBody(rwsim::dynamics::Body::Ptr body, rw::kinematics::State &state);

		/**
		 * @brief Add a body to the simulator.
		 * @param body [in] the body to add.
		 */
		void addBody(rwsim::dynamics::Body::Ptr body){ addBody(body,_state); }

		/**
		 * @copydoc addDevice(rwsim::dynamics::DynamicDevice::Ptr)
		 * @param state [in/out] the state giving the initial configuration of the device, which might be changed.
		 */
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev, rw::kinematics::State &state);

		/**
		 * @brief Add a device to the simulator.
		 * @param dev [in] the device to add.
		 */
		void addDevice(rwsim::dynamics::DynamicDevice::Ptr dev){ addDevice(dev,_state);}

		/**
		 * @copydoc addSensor(rwlibs::simulation::SimulatedSensor::Ptr)
		 * @param state [in/out] if the sensor is not registered in the state, it will be registered.
		 */
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor, rw::kinematics::State &state);

		/**
		 * @brief Add a simulated sensor to the simulator.
		 * @param sensor [in] the sensor to add.
		 */
		void addSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor){ addSensor(sensor,_state);}

		/**
		 * @brief Remove a simulated sensor from the simulator.
		 * @param sensor [in] the sensor to remove.
		 */
		void removeSensor(rwlibs::simulation::SimulatedSensor::Ptr sensor);

		/**
		 * @brief get the list of simulated sensors
		 * @return vector of sensors.
		 */
		 std::vector<rwlibs::simulation::SimulatedSensor::Ptr> getSensors();

		 // Simulator interface

         /**
          * @copydoc Simulator::step
          */
         void step(double dt);

         //! @copydoc Simulator::getState
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

		 /**
		  * @brief Enable or disable a body in the simulation.
		  * @param body [in] the body.
		  * @param enabled [in] boolean indicating if body should be enabled or not.
		  */
		 void setDynamicsEnabled(rwsim::dynamics::Body::Ptr body, bool enabled);
		 // interfaces for manipulating/controlling bodies

		 /**
		  * @brief Set a target position of a body. This will add forces/velocities to a body such that it
		  * moves toward the target pose.
		  * @param body [in] the body to control
		  * @param target [in] the target pose
		  * @param state [in] the current state.
		  * @param maxLinVel [in] (optional) maximum linear velocity of the body in \f$\frac{m}{s}\f$.
		  * @param maxLinAcc [in] (optional) maximum linear acceleration of the body in \f$\frac{m}{s^2}\f$.
		  * @param maxAngVel [in] (optional) maximum angular velocity of the body in \f$\frac{rad}{s}\f$.
		  * @param maxAngAcc [in] (optional) maximum angular acceleration of the body in \f$\frac{rad}{s^2}\f$.
		  */
		 void setTarget(
				 rwsim::dynamics::Body::Ptr body,
				 const rw::math::Transform3D<>& target,
				 rw::kinematics::State& state,
				 double maxLinVel = 0.5,
				 double maxLinAcc = 1.0,
				 double maxAngVel = 0.4,
				 double maxAngAcc = 1.0
		 );

		 /**
		  * @brief Set a target position of a body. This will add forces/velocities to a body such that it
		  * moves toward the target pose.
		  * @param body [in] the body to control
		  * @param target [in] the target pose
		  * @param maxLinVel [in] (optional) maximum linear velocity of the body in \f$\frac{m}{s}\f$.
		  * @param maxLinAcc [in] (optional) maximum linear acceleration of the body in \f$\frac{m}{s^2}\f$.
		  * @param maxAngVel [in] (optional) maximum angular velocity of the body in \f$\frac{rad}{s}\f$.
		  * @param maxAngAcc [in] (optional) maximum angular acceleration of the body in \f$\frac{rad}{s^2}\f$.
		  */
		 void setTarget(
				 rwsim::dynamics::Body::Ptr body,
				 const rw::math::Transform3D<>& target,
				 double maxLinVel = 0.5,
				 double maxLinAcc = 1.0,
				 double maxAngVel = 0.4,
				 double maxAngAcc = 1.0)
		 {
			 setTarget(body,target,_state,maxLinVel,maxLinAcc,maxAngVel,maxAngAcc);
		 }

		 /**
		  * @brief Set a target trajectory for a body.
		  * @param body [in] the body.
		  * @param traj [in] the trajectory.
		  */
		 void setTarget(dynamics::Body::Ptr body, rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj);

		 /**
		  * @brief Set a velocity target.
		  * @param body [in] the body that should move.
		  * @param velocity [in] the velocity target.
		  */
		 void setTarget(dynamics::Body::Ptr body, const rw::math::VelocityScrew6D<>& velocity);

		 /**
		  * @brief disables the target control of body \b body.
		  * @param body [in] the body.
		  */
		 void disableBodyControl( dynamics::Body::Ptr body );

		 //! @brief Disable all control for all bodies.
		 void disableBodyControl( );

		 /**
		  * @brief Get the body controller.
		  * @return the controller.
		  */
		 rwsim::control::BodyController::Ptr getBodyController(){ return _bodyController; }

		 /**
		  * @brief Attach bodies.
		  * @param b1 [in] first body.
		  * @param b2 [in] second body.
		  */
		 void attach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		 /**
		  * @brief Detach bodies.
		  * @param b1 [in] first body.
		  * @param b2 [in] second body.
		  */
		 void detach(rwsim::dynamics::Body::Ptr b1, rwsim::dynamics::Body::Ptr b2);

		 /**
		  * @brief Get the dynamic workcell used by the simulator.
		  * @return the dynamic workcell.
		  */
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

