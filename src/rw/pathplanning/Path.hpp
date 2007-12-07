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

#ifndef rw_pathplanning_Path_HPP
#define rw_pathplanning_Path_HPP

/**
 * @file Path.hpp
 */

#include <rw/math/Q.hpp>
#include <list>

namespace rw { namespace pathplanning {

    /**
     * @brief A list of joint configurations
     */
    typedef std::list<math::Q> Path;

}} // end namespaces

#endif // end include guard
