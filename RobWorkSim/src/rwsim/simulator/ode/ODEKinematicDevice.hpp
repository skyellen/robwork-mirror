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


#ifndef RWSIM_SIMULATOR_ODEKINEMATICDEVICE_HPP_
#define RWSIM_SIMULATOR_ODEKINEMATICDEVICE_HPP_

#include <ode/ode.h>

#include <vector>

#include <rw/math/Q.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>

#include "ODEJoint.hpp"
#include "ODEDevice.hpp"

namespace rwsim {
namespace simulator {
	/**
	 * @brief A bridge between the RW KinematicDevice and kinematicly controlled
	 * ODE dBodies.
	 */
	class ODEKinematicDevice: public ODEDevice {
	public:

		ODEKinematicDevice(dynamics::KinematicDevice *rdev, const std::vector<dBodyID>& kbodies);

		virtual ~ODEKinematicDevice();

		void reset(rw::kinematics::State& state);

		/**
		 * @brief
		 * @param dt
		 * @param state
		 */
		void update(double dt, rw::kinematics::State& state);

		void postUpdate(rw::kinematics::State& state);

	private:
		dynamics::KinematicDevice *_kdev;
		rw::math::Q _maxVel;
		std::vector<dBodyID> _kbodies;
	};
}
}
#endif /* ODEVELOCITYDEVICE_HPP_ */
