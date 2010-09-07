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

#ifndef RWSIM_SIMULATOR_PHYSICSENGINEFACTORY_HPP_
#define RWSIM_SIMULATOR_PHYSICSENGINEFACTORY_HPP_

#include <RWSimConfig.hpp>

#include <vector>

#include <rwsim/dynamics/DynamicWorkcell.hpp>
#include "Simulator.hpp"

#include <RWSimConfig.hpp>

#ifdef RWSIM_HAVE_RWPHYS
#include <rwsim/simulator/rwphysics/RWSimulator.hpp>
#endif

#ifdef RWSIM_HAVE_ODE
#include <rwsim/simulator/ode/ODESimulator.hpp>
#endif

#ifdef RWSIM_HAVE_BULLET
#include <rwsim/simulator/bullet/BtSimulator.hpp>
#endif

namespace rwsim {
namespace simulator {

	class PhysicsEngineFactory {
	public:
		static std::vector<std::string> getEngineIDs();

		static bool hasEngineID(const std::string& engineID);

		static Simulator* newPhysicsEngine(const std::string& engineID,
										   rwsim::dynamics::DynamicWorkcell* dwc);

        static Simulator* newPhysicsEngine(rwsim::dynamics::DynamicWorkcell* dwc);

	};
}
}


#endif /* PHYSICSENGINEFACTORY_HPP_ */
