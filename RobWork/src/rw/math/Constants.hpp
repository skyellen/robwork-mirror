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


#ifndef RW_MATH_CONSTANTS_HPP
#define RW_MATH_CONSTANTS_HPP

/**
 * @file Constants.hpp
 */

namespace rw { namespace math {

    /** @addtogroup math */
    /* @{ */

    /** Definition of Pi */
	const double Pi = 3.1415926535897932384626433832795;

    /** Converts inch to meter */
    const double Inch2Meter = 0.0254;

    /** Converts meter to inch */
    const double Meter2Inch = 1 / Inch2Meter;

    /** Convert degrees to radians */
    const double Deg2Rad = Pi / 180;

    /** Converts radians to degrees */
    const double Rad2Deg = 180 / Pi;

    /* @} */
}} // end namespaces

#endif // end include guard
