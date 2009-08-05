/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
