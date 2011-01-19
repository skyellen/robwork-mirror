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


#ifndef RWLIBS_SIMULATION_SIMULATEDSENSOR_HPP_
#define RWLIBS_SIMULATION_SIMULATEDSENSOR_HPP_

//! @file SimulatedSensor.hpp

#include <rw/kinematics/State.hpp>
#include <rw/sensor/Sensor.hpp>
#include <rw/common/Ptr.hpp>

namespace rwlibs {
namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief simulated sensor interface
     */
    class SimulatedSensor {
    public:

        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<SimulatedSensor> Ptr;

        /**
         * @brief steps the the SimulatedSensor with time \b dt and saves any state
         *  changes in \b state.
         * @param dt [in] the time step in seconds
         * @param state [out] changes of the SimulatedSensor is saved in state.
         */
        virtual void update(double dt, rw::kinematics::State& state) = 0;

        /**
         * @brief Resets the state of the SimulatedSensor to that of \b state
         * @param state [in] the state that the sensor is reset too.
         */
        virtual void reset(const rw::kinematics::State& state) = 0;

        /**
         * @brief Returns the rw::kinematics::Frame associated to the sensor. The frame may be NULL
         *
         * @return the rw::kinematics::Frame associated to the sensor
         */
        //virtual rw::kinematics::Frame* getFrame() = 0;
        virtual rw::sensor::Sensor* getSensor() = 0;

    };

    //! @}
}
}

#endif /* SIMULATEDSENSOR_HPP_ */
