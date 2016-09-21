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

#include <rw/math/Q.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include "ODEDevice.hpp"

namespace rwsim {
namespace simulator {
    class ODESimulator;
	class ODEJoint;
    /**
     * @brief A bridge between the RW RigidDevice and a set
     * of connected joints and rigid bodies.
     *
     * ODE does not support simulation of
     * devices using a reduced coordinate scheme. Only maximal coordinates
     * are supported. This class maps an
     * articulated body (Device) in reduced coordinates (RobWork) to a articulated body
     * in maximal coordinates (ODE).
     */
	class ODEVelocityDevice: public ODEDevice {
	public:

		/**
		 * @brief constructor
		 */
		ODEVelocityDevice(
		    ODEBody *base,
			dynamics::RigidDevice *rdev,
            const rw::kinematics::State &state,
			ODESimulator *sim
			);

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
		void update(const rwlibs::simulation::Simulator::UpdateInfo& dt, rw::kinematics::State& state);

		/**
		 * @copydoc ODEDevice::postUpdate
		 */
		void postUpdate(rw::kinematics::State& state);

		//! @copydoc ODEDevice::getBodies
        std::vector<ODEBody*> getBodies(){ return _ode_bodies; }

		//static ODEVelocityDevice* makeDevice(dynamics::RigidDevice *rdev, dBodyID base, dSpaceID space, dWorldID worldId);
	private:
		void init(dynamics::RigidDevice *rdev,
		          const rw::kinematics::State &state,
		          dSpaceID spaceId,
		          ODEBody* baseODEBody);
	private:
		dynamics::RigidDevice *_rdev;
		std::vector<ODEJoint*> _odeJoints;
		std::vector<ODEBody*> _ode_bodies;
		std::vector<rwsim::dynamics::RigidDevice::MotorControlMode> _modes;

		rw::math::Q _maxForce;
		rw::math::Q _lastQ;
		double _lastDt;
		ODESimulator *_sim;
	};
}
}
#endif /* ODEVELOCITYDEVICE_HPP_ */
