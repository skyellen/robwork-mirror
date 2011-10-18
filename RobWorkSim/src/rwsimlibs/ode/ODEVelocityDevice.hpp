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


#ifndef RWSIM_SIMULATOR_ODEVELOCITYDEVICE_HPP_
#define RWSIM_SIMULATOR_ODEVELOCITYDEVICE_HPP_

#include <ode/ode.h>

#include <vector>

#include "ODEJoint.hpp"
#include <rw/math/Q.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include "ODEJoint.hpp"
#include "ODEDevice.hpp"

namespace rwsim {
namespace simulator {

	class ODEVelocityDevice: public ODEDevice {
	public:

		/**
		 * @brief constructor
		 */
		ODEVelocityDevice(
			dynamics::RigidDevice *rdev,
			std::vector<ODEJoint*> odejoints,
			rw::math::Q maxForce);

		/**
		 * @brief destructor
		 */
		virtual ~ODEVelocityDevice();

		/**
		 * @copydoc ODEDevice::reset
		 */
		void reset(rw::kinematics::State& state);

		/**
		 * @copydoc ODEDevice::update
		 */
		void update(double dt, rw::kinematics::State& state);

		/**
		 * @copydoc ODEDevice::postUpdate
		 */
		void postUpdate(rw::kinematics::State& state);

		/**
		 *
		 * @param rdev
		 * @param base
		 * @return
		 */
		static ODEVelocityDevice* makeDevice(dynamics::RigidDevice *rdev,
											dBodyID base,
											dSpaceID space,
											dWorldID worldId);

	private:
		dynamics::RigidDevice *_rdev;
		std::vector<ODEJoint*> _odeJoints;
		rw::math::Q _maxForce;
		rw::math::Q _lastQ;
		double _lastDt;
	};
}
}
#endif /* ODEVELOCITYDEVICE_HPP_ */
