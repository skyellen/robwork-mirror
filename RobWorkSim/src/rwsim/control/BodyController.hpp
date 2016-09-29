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

#ifndef RWSIM_CONTROL_BODYCONTROLLER_HPP_
#define RWSIM_CONTROL_BODYCONTROLLER_HPP_

/**
 * @file control/BodyController.hpp
 *
 * \copydoc rwsim::control::BodyController
 */

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rwlibs/control/Controller.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <boost/thread/mutex.hpp>

namespace rwsim { namespace dynamics { class Body; } }
namespace rwsim { namespace dynamics { class KinematicBody; } }

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief A Controller that use a PD loop to follow a trajectory
	 * generated from different target types. If the body is a Kinematic body then
	 * the velocities of the body is directly controlled, else wrenches are used to
	 * control the body.
	 */
	class BodyController: public rwlibs::control::Controller, public rwlibs::simulation::SimulatedController {
	public:
    	//! @brief Smart pointer type to this class
	    typedef rw::common::Ptr<BodyController> Ptr;

	    /**
	     * @brief Construct new controller.
	     * @param name [in] the name of the controller.
	     */
		BodyController(const std::string& name);

		//! @brief Destructor.
		virtual ~BodyController();

		/**
		 * @brief Sets the target transform of a body. The target is defined in world frame.
		 * @param body [in] the body to set target for.
		 * @param target [in] the target transformation in world frame.
		 * @param state [in] the state giving the current position.
		 * @param maxLinVel [in] (optional) maximum linear velocity of the body in \f$\frac{m}{s}\f$.
		 * @param maxLinAcc [in] (optional) maximum linear acceleration of the body in \f$\frac{m}{s^2}\f$.
		 * @param maxAngVel [in] (optional) maximum angular velocity of the body in \f$\frac{rad}{s}\f$.
		 * @param maxAngAcc [in] (optional) maximum angular acceleration of the body in \f$\frac{rad}{s^2}\f$.
		 */
		void setTarget(
				rw::common::Ptr<rwsim::dynamics::Body> body,
				const rw::math::Transform3D<>& target,
				const rw::kinematics::State& state,
				double maxLinVel = 0.5,
				double maxLinAcc = 1.0,
				double maxAngVel = 0.4,
				double maxAngAcc = 1.0
		);

		/**
		 * @brief Set a target trajectory of a body. The initial configuration of the trajectory must match
		 * the current configuration of the body.
		 *
		 * A Kinematic body will follow the exact path of the trajectory whereas a RigidBody
		 * will use a PD controller to follow the trajectory
		 *
		 * @param body [in] the body that should be moved
		 * @param traj [in] the trajectory.
		 */
		void setTarget(rw::common::Ptr<rwsim::dynamics::Body> body,
		               rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj);

		/**
		 * @brief Set a velocity target.
		 * @param body [in] the body that should move.
		 * @param velocity [in] the velocity target.
		 */
		void setTarget(rw::common::Ptr<rwsim::dynamics::Body> body,
				const rw::math::VelocityScrew6D<> &velocity);

		/**
		 * @brief Set the force target of a body, the forces will be added such that the force
		 * on the body in each timestep will be timestep/[force;torque]. In other words the wrench
		 * [force;torque] is stretched over one second.
		 *
		 * The wrench is defined in world coordinates.
		 * @param body [in] the body.
		 * @param force [in] the force.
		 * @param torque [in] the torque.
		 */
		void setForceTarget(rw::common::Ptr<rwsim::dynamics::Body> body,
		                    rw::math::Vector3D<> force,
		                    rw::math::Vector3D<> torque);

		/**
		 * @brief Get the current Cartesian target
		 * @param body [in] the body for which to get the target
		 * @return 6D Cartesian target
		 */
		rw::math::Transform3D<> getTarget(rw::common::Ptr<rwsim::dynamics::Body> body);

		/**
		 * @brief Get the current target trajectory for body \b body
		 * @param body [in] body for which to get the target
		 * @return target trajectory
		 */
		rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr getTargetTrajectory(rw::common::Ptr<rwsim::dynamics::Body> body);

		/**
		 * @brief Disable control of a specific body.
		 * @param body [in] the body.
		 */
		void disableBodyControl(rw::common::Ptr<rwsim::dynamics::Body> body);

		//! @brief Disable control of all bodies.
		void disableBodyControl();

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getControllerName
		std::string getControllerName(){ return getName(); };

		//! @copydoc SimulatedController::setEnabled
        void setEnabled(bool enabled){ _enabled = enabled; };

		//! @copydoc SimulatedController::isEnabled
        bool isEnabled() const { return _enabled; } ;

		//! @copydoc SimulatedController::getControllerHandle
        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this;}

	private:
		BodyController();

	private:
        struct TargetData;

        static void updateKinematicBody(rwsim::dynamics::KinematicBody* body,
        		TargetData& tdata,
				const rwlibs::simulation::Simulator::UpdateInfo& info,
				rw::kinematics::State& state);

        static void updateBody(rwsim::dynamics::Body* body,
        		TargetData& tdata,
        		const rwlibs::simulation::Simulator::UpdateInfo& info,
				rw::kinematics::State& state);

		std::map<rwsim::dynamics::Body*, TargetData*> _bodyMap;
		bool _enabled;
		boost::mutex _mutex;
	};

	//! @}
}
}

#endif /*RWSIM_CONTROL_BODYCONTROLLER_HPP_*/
