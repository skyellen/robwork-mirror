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

    	typedef rw::common::Ptr<Simulator> Ptr;

        /**
         * @brief step info is used when updating controllers, devices and bodies.
         */
        struct UpdateInfo {
            UpdateInfo():dt(0.0),time(0.0),rollback(false){}
            UpdateInfo(double dt_step):dt(dt_step),time(0.0),rollback(false){}
            /**
             * @brief the timestep which is about to take place
             */
            double dt;

            /**
             * @brief the timestep taken in previous step.
             */
            double dt_prev;

            /**
             * @brief current simulation time
             */
            double time;

            /**
             * @brief if true then the previous step was unsuccessfull and a new step with
             * new parameters is executed. Typically the dt is reduces. The time should be the same
             * as previous call.
             */
            bool rollback;
        };

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
         * @brief initialize simulator with state variables
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

    };
    //! @}
}
}

#endif /*Simulator_HPP_*/

