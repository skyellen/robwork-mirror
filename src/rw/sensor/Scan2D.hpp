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


#ifndef RW_SENSOR_SCAN2D_HPP
#define RW_SENSOR_SCAN2D_HPP

#include "SensorData.hpp"

#include <vector>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/*@{*/

/**
 * @brief data structure for range scanner data
 */
class Scan2D: public SensorData {
public:
	Scan2D(){};
	virtual ~Scan2D(){};

    std::vector<float> _angle;
    std::vector<float> _depth;
};

/*@}*/

} //end namespace sensor
} //end namespace rw

#endif /*RW_SENSOR_SCAN2D_HPP*/
