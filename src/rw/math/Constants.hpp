/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rw_math_Constants_HPP
#define rw_math_Constants_HPP

/**
 * @file Constants.hpp
 */

#include <cmath>

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
