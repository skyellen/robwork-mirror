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

#ifndef RW_SENSOR_SCANNER25D_HPP
#define RW_SENSOR_SCANNER25D_HPP

#include "Scanner.hpp"
#include <rw/geometry/PointCloud.hpp>

#include <rw/common/Ptr.hpp>

namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief an interface describing a 3D scanner sensor. The scanner takes
 * pictures in the oposite direction of the z-axis of the frame that it is
 * attached to. The x-y plane forms the image plane such that the xy-origin is
 * located in the bottom left corner of the image.
 *
 */
class Scanner25D: public Scanner
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<Scanner25D> Ptr;

protected:

    /**
     * @brief constructor
     * @param frame [in] the frame that the scanner is attached to
     * @param name [in] name of scanner sensor
     */
    Scanner25D(const std::string& name, const std::string& desc = "") :
        Scanner(name, desc)
    {
    }

public:
    /**
     * @brief Destructor. Closes scanner connection if not already closed.
     */
    virtual ~Scanner25D();

    /**
     * @brief gets the last acquired image
     * @return the image that was last acquired.
     */
    virtual const rw::geometry::PointCloud& getScan() = 0;

};

/*@}*/

}
}

#endif /*RW_SENSOR_SCANNER3D_HPP*/
