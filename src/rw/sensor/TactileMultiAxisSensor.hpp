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


#ifndef RW_SENSOR_TACTILEMULTIAXISSENSOR_HPP
#define RW_SENSOR_TACTILEMULTIAXISSENSOR_HPP

#include "Sensor.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/multi_array.hpp>

#include <rw/kinematics/State.hpp>

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>


namespace rw {
namespace sensor {

/**
 * @brief
 */
class TactileMultiAxisSensor : public Sensor {
public:
    /**
     * @param name
     * @param frame
     * @return
     */
    TactileMultiAxisSensor(const std::string& name, rw::kinematics::Frame* frame):
        Sensor(frame, name)
    {

    }

    /**
     * @brief destructor
     * @return
     */
    virtual ~TactileMultiAxisSensor(){};

    /**
     * @brief acquires force data from the tactile cells
     * @param state
     */
    virtual void acquire() = 0;

    /**
     * @brief gets the maximum force in Newton that this sensor can measure on any of its
     * axis.
     * @return max force in Newton.
     */
    virtual double getMaxForce() = 0;

    /**
     * @brief gets the maximum torque in Newton Meter (N m)that this sensor can measure on any of its
     * axis.
     * @return max torque in Newton Meter(N m).
     */
    virtual double getMaxTorque() = 0;

};

}
}

#endif /*RW_SENSOR_TACTILEARRAY_HPP*/
