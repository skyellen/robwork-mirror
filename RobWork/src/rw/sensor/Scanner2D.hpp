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

#ifndef RW_SENSOR_SCANNER2D_HPP
#define RW_SENSOR_SCANNER2D_HPP

/**
 * @file Scanner2D.hpp
 */

#include "Scanner.hpp"
#include <rw/geometry/PointCloud.hpp>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief The Scanner2D sensor encapsulate the basic interface of a
 * 2 dimensional range scanning device such as SICK or Hokyuo laser
 * range scanners.
 *
 *  The interface supports any range scanner that measures distance in
 * an arc around the origin of the sensor.
 */

class Scanner2D: public Scanner
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Scanner2D> Ptr;

protected:

    /**
     * @brief constructor
     * @param name [in] name of scanner sensor
     * @param description [in] description of scanner sensor
     */
    Scanner2D(const std::string& name, const std::string& description = "") :
        Scanner(name, description)
    {
    }

public:
    /**
     * @brief destructor
     */
    virtual ~Scanner2D();

    /**
     * @brief gets the last acquired scan as a depth image
     * of height 1.
     */
    virtual const rw::geometry::PointCloud& getScan() const = 0;

    /**
     * @brief Returns the angular range of the scanner.
     *
     * @return Angular range in radians
     */
    virtual double getAngularRange() const = 0;

    /**
     * @brief Returns the number of scan points
     */
    virtual size_t getMeasurementCount() const = 0;

};

/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER2D_HPP_*/
