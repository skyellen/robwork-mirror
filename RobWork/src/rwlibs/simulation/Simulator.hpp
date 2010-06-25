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

#ifndef RWLIBS_SIMULATION_SIMULATOR_HPP_
#define RWLIBS_SIMULATION_SIMULATOR_HPP_

//! @file Simulator.hpp

#include <rw/kinematics/State.hpp>
#include <rw/common/PropertyMap.hpp>

namespace rwlibs {
namespace simulation {
    //! @addtogroup simulation
    /**
     * @brief interface of a general simulator
     */
    class Simulator
    {
    public:

        /**
         * @brief
         */
        virtual ~Simulator(){};

        /**
         * @brief take a step forward in time with timestep \b dt.
         * @param dt [in] the time step
         * @param state [in/out] current state and output as updated state
         */
        virtual void step(double dt, rw::kinematics::State& state) = 0;

        /**
         * @brief reset velocity and acceleration of all bodies to 0. And sets the position of all bodies
         * to that described in state
         */
        virtual void reset(rw::kinematics::State& state) = 0;

        /**
         * @brief initialize simulator physics with state
         */
        virtual void init(rw::kinematics::State& state) = 0;

        /**
         * @brief gets the the current simulated time
         */
        virtual double getTime() = 0;

        /**
         * Enables or disables simulation of a frame
         * @param frame
         * @param enabled
         */
        virtual void setEnabled(rw::kinematics::Frame* frame, bool enabled) = 0;

        /**
         * @brief
         */
        virtual rw::common::PropertyMap& getPropertyMap() = 0;

        /**
         * @brief
         */
        virtual void emitPropertyChanged() = 0;

    };
    //! @}
}
}

#endif /*Simulator_HPP_*/

