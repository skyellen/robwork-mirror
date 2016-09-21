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

#ifndef RWSIM_SIMULATOR_ODEDEVICE_HPP_
#define RWSIM_SIMULATOR_ODEDEVICE_HPP_

#include <rwlibs/simulation/Simulator.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {

    class ODEBody;

	/**
	 * @brief interface for classes (ODEDevices) that control a set of ode bodies
	 * that map to a RWSim dynamic device type.
	 */
	class ODEDevice {
	public:
		/**
		 * @brief destructor
		 */
		virtual ~ODEDevice(){};

		/**
		 * @brief resets the ODE device to the state values of the RWSim device.
		 * @param state
		 */
		virtual void reset(rw::kinematics::State& state) = 0;

		/**
		 * @brief the update call is made prior to the simulation step. In this
		 * method states of the ODE bodies and joints (forces, velocities, eg)
		 * can be updated from the state of the RWSim device.
		 * @param dt
		 * @param state [out] ODEDevice state values are copied to \b state
		 */
		virtual void update(const rwlibs::simulation::Simulator::UpdateInfo& dt, rw::kinematics::State& state) = 0;

		/**
		 * @brief The post update is called after a simulation step has
		 * been performed. Here the modified states (force,velocity,position)
		 * of the ODE device is written back to the \b state object.
		 * @param state
		 */
		virtual void postUpdate(rw::kinematics::State& state) = 0;

		/**
		 * @brief Get the ODE bodies in the device.
		 * @return a vector of bodies.
		 */
		virtual std::vector<ODEBody*> getBodies() = 0;

	protected:
		ODEDevice(){};
	};
}
}
#endif /* ODEDEVICE_HPP_ */
