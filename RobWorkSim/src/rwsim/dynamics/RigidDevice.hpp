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
	    typedef rw::common::Ptr<RigidDevice> Ptr;
		/**
		 *
		 * @param bodies
		 * @param dev
		 * @param wc
		 * @return
		 */
		RigidDevice(dynamics::Body::Ptr base,
					const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
					rw::models::JointDevice::Ptr dev);

		/**
		 *
		 * @return
		 */
		virtual ~RigidDevice(){};

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
		 * @param state
		 * @return
		 */
		rw::math::Q getJointVelocities(const rw::kinematics::State& state);
		double getJointVelocity(int i, const rw::kinematics::State& state);

		/**
		 * @brief set the joint velocities
		 * @param q [in] joint velocities
		 * @param state [in] the state in which to set the velocities
		 * @return
		 */
        void setJointVelocities(const rw::math::Q& q, rw::kinematics::State& state);
        void setJointVelocity(double vel, int i, rw::kinematics::State& state);

		typedef enum{Force, Velocity} MotorControlMode;
		/**
		 * @brief get the modes of all motors
		 */
		std::vector<MotorControlMode> getMotorModes(const rw::kinematics::State& state);
        MotorControlMode getMotorMode(int i, const rw::kinematics::State& state);

		/**
		 * @brief get the target off all motors
		 * @param state [in]
		 * @return
		 */
		rw::math::Q getMotorTargets(const rw::kinematics::State& state);
        double getMotorTarget(int i, const rw::kinematics::State& state);

		/**
		 * @brief set target of all motors
		 * @param q [in] target of motors in either force[N]/torque[Nm] or velocity [m/s]/[rad/s]
		 * @param state [in/out]
		 */
		void setMotorTargets(const rw::math::Q& q, rw::kinematics::State& state);
		void setMotorForceTargets(const rw::math::Q& force, rw::kinematics::State& state);
		void setMotorVelocityTargets(const rw::math::Q& vel, rw::kinematics::State& state);


		/**
		 * @brief set the target of motor \b i. the target may be a desired force or a desired
		 * velocity depending on the current mode of the motor.
		 * @param q [in] the target value in either force[N]/torque[Nm] or velocity [m/s]/[rad/s]
		 * @param i [in] the index of the motor
		 * @param state [in/out]
		 */
		void setMotorTarget(double q, int i, rw::kinematics::State& state);
		void setMotorForceTarget(double force, int i, rw::kinematics::State& state);
		void setMotorVelocityTarget(double vel, int i, rw::kinematics::State& state);


/*
		void setActualVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
			RW_ASSERT(vel.size()==_velocity.getN());


			_actualVel = vel;
		}

		rw::math::Q getActualVelocity(const rw::kinematics::State& state){
			return _actualVel;
		}
*/

		//std::vector<dynamics::RigidJoint*> getRigidJoints(){ return _bodies; }
		//void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state);

		rw::models::JointDevice::Ptr getJointDevice(){ return _jdev;} ;

        const std::vector<Body::Ptr>& getLinks(){  return _links; }

        /**
         * @copydoc rw::kinematics::StatelessObject::registerStateData
         */
        //virtual void registerStateData(rw::kinematics::StateStructure::Ptr statestructure);

	public: ///// DEPRECATED FUNCTIONS
        //rw::math::Q getForceLimit() { return getMotorForceLimits(); }
        // void setVelocity(rw::math::Q& vel, rw::kinematics::State& state){ setJointVelocities(vel, state);}

	private:

		//rw::math::Q _vel, _actualVel;
		//rw::math::Q _torque;
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
