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
#include <rw/kinematics/StatelessObject.hpp>
#include "Simulator.hpp"
#include <rw/kinematics/Frame.hpp>

namespace rwlibs {
namespace simulation {
    //! @addtogroup simulation
	// @{

    /**
     * @brief simulated sensor interface
     */
    class SimulatedSensor: public rw::sensor::Sensor {
    public:
        //! @brief smart pointer type of this class
        typedef rw::common::Ptr<SimulatedSensor> Ptr;

    protected:
        //! constructor
        SimulatedSensor(const std::string& name):Sensor(name){};

        //! constructor
        SimulatedSensor(const std::string& name, const std::string& desc):Sensor(name,desc){};

    public:

        //! @brief destructor
        virtual ~SimulatedSensor(){};

        /**
         * @brief steps the the SimulatedSensor with time \b dt and saves any state
         *  changes in \b state.
         * @param dt [in] the time step in seconds
         * @param state [out] changes of the SimulatedSensor is saved in state.
         */
        virtual void update(const Simulator::UpdateInfo& info, rw::kinematics::State& state) = 0;

        /**
         * @brief Resets the state of the SimulatedSensor to that of \b state
         * @param state [in] the state that the sensor is reset too.
         */
        virtual void reset(const rw::kinematics::State& state) = 0;

        /**
         * @brief get the "real" sensor interface of this simulated sensor. This might be
         * NULL if the simulatedsensor does not implement a specific Sensor interface.
         */
        virtual rw::sensor::Sensor::Ptr getSensor() = 0;

        /**
         * @copydoc rw::sensor::Sensor::attachTo
         *
         * @note we want to make sure that the frame is also changed on the "real" sensor
         */
        void attachTo(rw::kinematics::Frame* frame) {
            rw::sensor::Sensor::attachTo(frame);
            if(getSensor())
                getSensor()->attachTo( frame );
        }

    };
    //! @}
}
}

#endif /* SIMULATEDSENSOR_HPP_ */
