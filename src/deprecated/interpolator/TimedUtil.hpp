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


#ifndef RW_ROBP_TIMEDUTIL_HPP
#define RW_ROBP_TIMEDUTIL_HPP

#include "Timed.hpp"
#include "TimedStatePath.hpp"

#include <rw/math/Q.hpp>


/**
   @file TimedUtil.hpp
   @brief Class rw::interpolator::TimedUtil
*/

#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace models {
    class WorkCell;
}}

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
       @brief Construction of paths of Timed values.
    */
    class TimedUtil
    {
    public:
        //! A tuple (time, state).
        typedef Timed<kinematics::State> TimedState;

        //! A tuple (time, q).
        typedef Timed<math::Q> TimedQ;

        /**
           @brief A path of time stamped configurations.

           The time stamp of the first configuration is zero, and the time for
           the remaining configurations are computed using the joint speed
           velocities \b speed.
         */
        static std::vector<TimedQ> makeTimedQPath(
            const math::Q& speed,
            const std::vector<math::Q>& path);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is zero, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b workcell.
         */
        static TimedStatePath makeTimedStatePath(
            const models::WorkCell& workcell,
            const std::vector<kinematics::State>& path);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
