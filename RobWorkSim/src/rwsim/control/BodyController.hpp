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
 * @file BodyController.hpp
 *
 * \copydoc rwsim::control::BodyController
 */

#include <rwlibs/control/Controller.hpp>
#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/control/SyncVelocityRamp.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <list>

namespace rwsim {
namespace control {
	//! @addtogroup rwsim_control
	//! @{

	/**
	 * @brief a Controller that use a PD loop to follow a trajectory
	 * generated from different target types. If the body is a Kinematic body then
	 * the velocities of the body is directly controlled. else wrenches are used to
	 * control the body.
	 */
	class BodyController: public rwlibs::control::Controller, public rwlibs::simulation::SimulatedController {
	public:

	    typedef rw::common::Ptr<BodyController> Ptr;

		BodyController(const std::string& name);

		/**
		 * @brief destructor
		 */
		virtual ~BodyController(){};

		/**
		 * @brief sets the target transform of a body. The target is defined in world frame.
		 * @param body
		 * @param target
		 * @param state
		 */
		void setTarget(rwsim::dynamics::Body::Ptr body, const rw::math::Transform3D<>& target, rw::kinematics::State& state);

		/**
		 * @brief set a target trajectory of a body. The initial configuration of the trajectory must match
		 * the current configuration of the body.
		 *
		 * A Kinematic body will follow the exact path of the trajectory whereas a RigidBody
		 * will use a PD controller to follow the trajectory
		 *
		 * @param body [in] the body that should be moved
		 * @param traj
		 * @param state
		 */
		void setTarget(rwsim::dynamics::Body::Ptr body,
		               rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr traj,
		               rw::kinematics::State& state);

	/**
	 * @brief Set a velocity target.
	 * @param body [in] the body that should move.
	 * @param velocity [in] the velocity target.
	 * @param state [in/out] the state to update with the new targets.
	 */
	void setTarget(rwsim::dynamics::Body::Ptr body,
			const rw::math::VelocityScrew6D<> &velocity,
			rw::kinematics::State& state);

		/**
		 * @brief set the force target of a body, the forces will be added such that the force
		 * on the body in each timestep will be timestep/[force;torque]. In other words the wrench
		 * [force;torque] is stretched over one second.
		 *
		 * The wrench is defined in world coordinates.
		 * @param body
		 * @param force
		 * @param torque
		 * @param state
		 */
		void setForceTarget(rwsim::dynamics::Body::Ptr body,
		                    rw::math::Vector3D<> force,
		                    rw::math::Vector3D<> torque,
		                    rw::kinematics::State& state);

		/**
		 * @brief get the current Cartesian target
		 * @param body [in] the body for which to get the target
		 * @return 6D Cartesian target
		 */
		rw::math::Transform3D<> getTarget(rwsim::dynamics::Body::Ptr body);

		/**
		 * @brief get the current target trajectory for body \b body
		 * @param body [in] body for which to get the target
		 * @return target trajectory
		 */
		rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr getTargetTrajectory(rwsim::dynamics::Body::Ptr body);


		void disableBodyControl(rwsim::dynamics::Body::Ptr body);

		void disableBodyControl();

		//! @copydoc SimulatedController::update
		void update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state);

		//! @copydoc SimulatedController::reset
		void reset(const rw::kinematics::State& state);

		//! @copydoc SimulatedController::getController
		Controller* getController(){ return this; };

		std::string getControllerName(){ return getName(); };

        void setEnabled(bool enabled){ _enabled = enabled; };

        bool isEnabled() const { return _enabled; } ;


        rwlibs::control::Controller::Ptr getControllerHandle(rwlibs::simulation::Simulator::Ptr sim){ return this;}


        typedef enum {Pose6DController, TrajectoryController, VelocityController, ForceController} ControlType;
        struct TargetData {
            TargetData():_type(Pose6DController),_enabled(false){ reset();}
            TargetData(ControlType type):_type(type),_enabled(true){ reset(); }
            void reset(){
                _time = 0;
                _lastTime = 0;
                _lastDt = 0;
            }
            ControlType _type;
            rw::trajectory::Trajectory<rw::math::Transform3D<> >::Ptr _traj;
			rw::math::VelocityScrew6D<> _velocity;
            rw::math::Transform3D<> _target;
            rw::math::Vector3D<> _force, _torque;
            double _time, // current time on the trajectory
                   _lastTime, // the time on the trajectory taken at the last non-rollback step
                   _lastDt; // the simulated starting time
            bool _enabled;
        };

	private:
		BodyController();

	private:
		std::map<rwsim::dynamics::Body*, TargetData> _bodyMap;
		std::list<rwsim::dynamics::Body::Ptr> _bodies;
		bool _enabled;
		boost::mutex _mutex;
	};

	//! @}
}
}

#endif /*PDController_HPP_*/
