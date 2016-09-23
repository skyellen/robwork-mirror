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

#ifndef RWSIM_DYNAMICS_RIGIDDEVICE_HPP_
#define RWSIM_DYNAMICS_RIGIDDEVICE_HPP_

#include <rw/math/Q.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/models/JointDevice.hpp>
#include "DynamicDevice.hpp"
//#include "RigidJoint.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{


	/**
	 * @brief the Rigid device is composed of a set of links where one or multiple
	 * constraints (joints) connect the links. The RigidDevice has motors on all active joints.
	 * These motors can be operated in either velocity control mode or force control mode.
	 *
	 * To extend this to other types of control please use a JointController from rwsim::control
	 *
	 * The Rigid device is created as a wrapper on top of a kinematic device from RobWork.
	 * This makes all constraints hard and only constraints (that is joints) that robwork supports
	 * in its device class is supported here.
	 */
	class RigidDevice : public DynamicDevice {
	public:
		//! @brief Smart pointer type for a dynamic device.
	    typedef rw::common::Ptr<RigidDevice> Ptr;

		/**
		 * @brief Construct new kinematic device.
		 * @param base [in] base of the device.
		 * @param objects [in] vector of links. Each linksis given as the dynamic body parameters and the object geometry.
		 * @param dev [in] the kinematic model.
		 */
		RigidDevice(dynamics::Body::Ptr base,
					const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
					rw::models::JointDevice::Ptr dev);

		//! @brief Destructor.
		virtual ~RigidDevice(){}

		/**
		 * @brief set the force limits of all motors of this device
		 * @param force [in] vector or force limits
		 */
		void setMotorForceLimits(const rw::math::Q& force);

		/**
		 * @brief get the force limits of all motors of this device
		 */
		rw::math::Q getMotorForceLimits();

		/**
		 * @brief get velocities of all motorized joints
		 * @param state [in] the state.
		 * @return the joint velocities.
		 */
		rw::math::Q getJointVelocities(const rw::kinematics::State& state);

		/**
		 * @brief Get the velocity of a motorized joint.
		 * @param i [in] the joint to get velocity for.
		 * @param state [in] the state.
		 * @return the velocity for the given joint.
		 */
		double getJointVelocity(int i, const rw::kinematics::State& state);

		/**
		 * @brief set the joint velocities
		 * @param q [in] joint velocities
		 * @param state [in] the state in which to set the velocities
		 * @return
		 */
        void setJointVelocities(const rw::math::Q& q, rw::kinematics::State& state);

		/**
		 * @brief Set the velocity of a motorized joint.
		 * @param vel [in] the velocity.
		 * @param i [in] the joint to set velocity for.
		 * @param state [in] the state in which to set the velocity.
		 */
        void setJointVelocity(double vel, int i, rw::kinematics::State& state);

        //! @brief Possible control modes for the motors in each joint.
		typedef enum {
			Force,  //!< For force control.
			Velocity//!< For velocity control.
		} MotorControlMode;

		/**
		 * @brief get the modes of all motors
		 */
		std::vector<MotorControlMode> getMotorModes(const rw::kinematics::State& state);

		/**
		 * @brief Get the control mode for a single motor.
		 * @param i [in] the joint number.
		 * @param state [in] the state.
		 * @return the control mode.
		 */
        MotorControlMode getMotorMode(int i, const rw::kinematics::State& state);

		/**
		 * @brief get the target off all motors
		 * @param state [in]
		 * @return
		 */
		rw::math::Q getMotorTargets(const rw::kinematics::State& state);

		/**
		 * @brief Get the target for a single motor.
		 * @param i [in] the joint number.
		 * @param state [in] the state.
		 * @return the current target.
		 */
        double getMotorTarget(int i, const rw::kinematics::State& state);

		/**
		 * @brief set target of all motors
		 * @param q [in] target of motors in either force[N]/torque[Nm] or velocity [m/s]/[rad/s]
		 * @param state [in/out]
		 */
		void setMotorTargets(const rw::math::Q& q, rw::kinematics::State& state);

		/**
		 * @brief Set force targets for all motors.
		 * @param force [in] the force targets.
		 * @param state [out] the state to update with target.
		 */
		void setMotorForceTargets(const rw::math::Q& force, rw::kinematics::State& state);

		/**
		 * @brief Set velocity targets for all motors.
		 * @param vel [in] the velocity targets.
		 * @param state [out] the state to update with target.
		 */
		void setMotorVelocityTargets(const rw::math::Q& vel, rw::kinematics::State& state);


		/**
		 * @brief set the target of motor \b i. the target may be a desired force or a desired
		 * velocity depending on the current mode of the motor.
		 * @param q [in] the target value in either force[N]/torque[Nm] or velocity [m/s]/[rad/s]
		 * @param i [in] the index of the motor
		 * @param state [in/out]
		 */
		void setMotorTarget(double q, int i, rw::kinematics::State& state);

		/**
		 * @brief Set force target for a single motor.
		 * @param force [in] the force target.
		 * @param i [in] the index of the motor.
		 * @param state [out] the state to update with target.
		 */
		void setMotorForceTarget(double force, int i, rw::kinematics::State& state);

		/**
		 * @brief Set velocity target for a single motor.
		 * @param vel [in] the velocity target.
		 * @param i [in] the index of the motor.
		 * @param state [out] the state to update with target.
		 */
		void setMotorVelocityTarget(double vel, int i, rw::kinematics::State& state);

		/**
		 * @brief Get the kinematic model of the device.
		 * @return the kinematic model.
		 */
		rw::models::JointDevice::Ptr getJointDevice(){ return _jdev;} ;

		/**
		 * @brief Get the links of the device.
		 * @return the links.
		 */
        const std::vector<Body::Ptr>& getLinks(){  return _links; }

	private:
        // all state variables are declared here
		rw::kinematics::StatelessData<double> _velocity;
		rw::kinematics::StatelessData<double> _target;
        rw::kinematics::StatelessData<char> _mode;


		// these should all be part of the state...
		rw::math::Q _forceLimits;
		//std::vector<dynamics::RigidJoint*> _bodies;
        std::vector<Body::Ptr> _links;
        rw::models::JointDevice::Ptr _jdev;
	};
}
}

#endif /*RIGIDDEVICE_HPP_*/
