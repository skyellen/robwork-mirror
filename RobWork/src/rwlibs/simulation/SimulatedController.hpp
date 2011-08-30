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

#ifndef RWLIBS_SIMULATION_CONTROLLER_HPP_
#define RWLIBS_SIMULATION_CONTROLLER_HPP_

//! @file SimulatedController.hpp

#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>
#include <rwlibs/control/Controller.hpp>
#include <rw/kinematics/StatelessObject.hpp>
#include "Simulator.hpp"

namespace rwlibs {
namespace simulation {
    //! @addtogroup simulation
	// @{
    /**
     * @brief interface of a simulated controller
     */
    class SimulatedController: public rw::kinematics::StatelessObject {
    public:
        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<SimulatedController> Ptr;


        /**
         *  @brief get the name of this controller
         *  @return name of this controller
         */
        virtual std::string getControllerName() = 0;

        /**
         * @brief updates/steps the controller with time step \b dt. It will update
         * the state \b state accordingly
         * @param dt [in] timestep in seconds
         * @param state [in/out] the current state
         */
        virtual void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state) = 0;

        /**
         * @brief reset the controller to the applied state
         * @param state [in] the state to reset to
         */
        virtual void reset(const rw::kinematics::State& state) = 0;

        /**
         * @brief get the controller associated with this simulated controller
         * @return
         */
        virtual rwlibs::control::Controller* getController() = 0;


    };
    //! @}
}
}

#endif /*CONTROLLER_HPP_*/
