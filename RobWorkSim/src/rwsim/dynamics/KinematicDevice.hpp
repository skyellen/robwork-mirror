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

#ifndef RWSIM_DYNAMICS_KINEMATICDEVICE_HPP_
#define RWSIM_DYNAMICS_KINEMATICDEVICE_HPP_

#include <rw/models/JointDevice.hpp>
#include <rw/models/Object.hpp>

#include "DynamicDevice.hpp"
#include "KinematicBody.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics
	//! @{

	/**
	 * @brief a kinematic device is able to influence the simulated environment
	 * but the device is not influenced by any external force as is the RigidDevice.
	 *
	 * This class is especially usefull for animating robot devices in a
	 * simulated environment.
	 */
	class KinematicDevice: public DynamicDevice {
	public:
		KinematicDevice(dynamics::Body *base,
	                    const std::vector<std::pair<BodyInfo,rw::models::Object::Ptr> >& objects,
						rw::models::JointDevice::Ptr dev);

		/**
		 * @brief destructor
		 * @return
		 */
		virtual ~KinematicDevice();

		virtual void setQ(const rw::math::Q &q, const rw::kinematics::State& state){_q = q;};

		virtual rw::math::Q getQ(const rw::kinematics::State& state){return _q;}

		rw::math::Q getVelocity(const rw::kinematics::State& state){return _velQ;};

		void setVelocity(const rw::math::Q &vel, const rw::kinematics::State& state){ _velQ = vel;};

		/**
		 * @brief get the kinematic bodies that this KinematicDevice controls. The
		 * bodies are ordered such that device joint \b i maps to kinematic body  \b i
		 * @return all bodies that the device controls.
		 */
		const std::vector<Body*>& getLinks(){ return _links; }

		// parameters for velocity profile
		void setMaxAcc(const rw::math::Q& acc);
		rw::math::Q getMaxAcc();

		void setMaxVel(const rw::math::Q& vel);
		rw::math::Q getMaxVel();

		void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state){

		}

		rw::models::JointDevice::Ptr getJointDevice(){ return _jdev; }
	private:
		std::vector<Body*> _links;
		rw::math::Q _maxVel, _maxAcc;
		rw::math::Q _q, _velQ;
		rw::models::JointDevice::Ptr _jdev;
	};
	//! @}
}
}

#endif /*KINEMATICDEVICE_HPP_*/
