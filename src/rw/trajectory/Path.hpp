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

#ifndef RW_TRAJECTORY_PATH_HPP
#define RW_TRAJECTORY_PATH_HPP

/**
   @file Path.hpp
*/

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <vector>
#include "Timed.hpp"

namespace rw { namespace trajectory {

    /**
       @brief std::vector of rw::math::Q
    */
    typedef std::vector<rw::math::Q> QPath;

    /**
       @brief std::vector of rw::kinematics::State
    */
    typedef std::vector<rw::kinematics::State> StatePath;

    /**
       @brief std::vector of rw::math::Q with associated times
    */
    typedef std::vector<TimedQ> TimedQPath;

    /**
       @brief std::vector of rw::kinematics::State with associated times
    */
    typedef std::vector<TimedState> TimedStatePath;

}} // end namespaces

#endif // end include guard
