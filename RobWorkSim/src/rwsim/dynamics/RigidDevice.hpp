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

#include "DynamicDevice.hpp"
#include "RigidJoint.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics
	//! @{


	/**
	 *
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
					const std::vector<dynamics::RigidJoint*>& bodies,
					rw::models::Device *dev,
					rw::models::WorkCell* wc):
			DynamicDevice(base,dev,wc),
			_vel( rw::math::Q::zero(dev->getDOF()) ),
			_actualVel( rw::math::Q::zero(dev->getDOF()) ),
			_force( rw::math::Q::zero(dev->getDOF()) ),
			_bodies(bodies)
		{

		}

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

		void setVelocity(const rw::math::Q& vel, const rw::kinematics::State& state);

		const std::vector<dynamics::RigidJoint*>& getBodies(){
			return _bodies;
		}

		void setActualVelocity(const rw::math::Q& vel, const rw::kinematics::State& state){
			RW_ASSERT(vel.size()==_actualVel.size());
			_actualVel = vel;
		}

		rw::math::Q getActualVelocity(const rw::kinematics::State& state){
			return _actualVel;
		}

		std::vector<dynamics::RigidJoint*> getRigidJoints(){ return _bodies; }

		void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state);
		rw::math::Q _torque;
	private:
		rw::math::Q _vel, _actualVel;
		rw::math::Q _force;
		std::vector<dynamics::RigidJoint*> _bodies;
	};
}
}

#endif /*RIGIDDEVICE_HPP_*/
