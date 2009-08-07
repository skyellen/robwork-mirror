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


#ifndef SIMULATEDSENSOR_HPP_
#define SIMULATEDSENSOR_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/sensor/Sensor.hpp>

namespace rwlibs {
namespace simulation {

/**
 * @brief simulated sensor
 */
class SimulatedSensor {

public:

    /**
     * @brief Updates the state of the SimulatedSensor and saves any state
     *  changes in \b state.
     * @param dt
     * @param state
     */
    virtual void update(double dt, rw::kinematics::State& state) = 0;


    virtual void reset(const rw::kinematics::State& state) = 0;

    virtual rw::sensor::Sensor* getSensor() = 0;

};

}}

#endif /* SIMULATEDSENSOR_HPP_ */
