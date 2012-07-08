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
#include <rw/math/Math.hpp>
#include <rw/models/JointDevice.hpp>
#include "DynamicDevice.hpp"
#include "RigidJoint.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics
	//! @{


	/**
	 * @brief the Rigid device is composed of a set of links where one or multiple
	 * constraints connect the links. The RigidDevice extends a velocity interface which
	 * may be used directly or through a controller.
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
		RigidDevice(dynamics::Body* base,
					const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
					rw::models::JointDevice::Ptr dev);

		/**
		 *
		 * @return
		 */
		virtual ~RigidDevice(){};

		/**
		 *
		 * @param force
		 */
		void setForceLimit(const rw::math::Q& force){
			_force = force;
		}

		/**
		 *
		 * @return
		 */
		rw::math::Q getForceLimit(){
			return _force;
		}

		rw::math::Q getVelocity(const rw::kinematics::State& state){
			return _vel;
		}

		typedef enum{Force, Velocity} MotorType;
		std::vector<MotorType> getMotorCfg();


		void setJointTarget(const rw::math::Q& vel, const rw::kinematics::State& state);
		void setTargetForce(const rw::math::Q& force, const rw::kinematics::State& state);

		void setVelocity(const rw::math::Q& vel, const rw::kinematics::State& state);

		void setActualVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
			RW_ASSERT(vel.size()==_actualVel.size());
			_actualVel = vel;
		}

		rw::math::Q getActualVelocity(const rw::kinematics::State& state){
			return _actualVel;
		}

		//std::vector<dynamics::RigidJoint*> getRigidJoints(){ return _bodies; }

		void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state);

		rw::models::JointDevice::Ptr getJointDevice(){ return _jdev;} ;

        const std::vector<Body*>& getLinks(){  return _links; }

		rw::math::Q _torque;
	private:
		rw::math::Q _vel, _actualVel;

		// these should all be part of the state...
		rw::math::Q _force;
		//std::vector<dynamics::RigidJoint*> _bodies;
        std::vector<Body*> _links;
        rw::models::JointDevice::Ptr _jdev;
	};
}
}

#endif /*RIGIDDEVICE_HPP_*/
