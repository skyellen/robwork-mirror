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

#ifndef RWSIM_SIMULATOR_BODYCONTROLLER_HPP_
#define RWSIM_SIMULATOR_BODYCONTROLLER_HPP_

//#include <rwsim/dynamics/Body.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {
	//! @addtogroup rwsim_dynamics
	//! @{
    /**
     * @brief The body controller is a pure interface through which bodies are controlled
     */
    class BodyController
    {
    public:

    	/**
    	 * @brief add external forces to the bodies that this
    	 * BodyManipulator controls.
    	 */
    	virtual void addForces(rw::kinematics::State &state, double time) = 0;

    	/**
    	 * @brief resets the state of the body controller to \b state
    	 */
    	virtual void reset(rw::kinematics::State &state) = 0;

    	/**
    	 * @brief return the list of bodies that this controller controls
    	 */
    	//virtual std::vector<Body*>& getBodies() = 0;

    };
    //! @}
}
}
#endif /*BODYCONTROLLER_HPP_*/
