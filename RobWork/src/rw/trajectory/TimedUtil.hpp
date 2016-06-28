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


#ifndef RW_TRAJECTORY_TIMEDUTIL_HPP
#define RW_TRAJECTORY_TIMEDUTIL_HPP

#include "Timed.hpp"
#include "Path.hpp"

#include <rw/math/Q.hpp>

/**
   @file TimedUtil.hpp
   @brief Class rw::trajectory::TimedUtil
*/

namespace rw { namespace kinematics { class State; }}
namespace rw { namespace models { class Device; }}
namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
       @brief Construction of paths of Timed values.
    */
    class TimedUtil
    {
    public:
        /**
           @brief A path of time stamped configurations.

           The time stamp of the first configuration is \b offset, and the time
           for the remaining configurations are computed using the joint speed
           velocities \b speed.
        */
        static TimedQPath makeTimedQPath(
            const math::Q& speed, const QPath& path, double offset = 0);

        /**
           @brief A path of time stamped configurations.

           The time stamp of the first configuration is \b offset , and the time for
           the remaining configurations are computed using the joint speed
           velocities of \b device.
         */
        static TimedQPath makeTimedQPath(
            const models::Device& device, const QPath& path, double offset = 0);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is \offset, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b workcell.
         */
        static TimedStatePath makeTimedStatePath(
            const models::WorkCell& workcell,
            const StatePath& path,
			double offset = 0);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is \b offset, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b device.
        */
        static TimedStatePath makeTimedStatePath(
            const models::Device& device,
            const QPath& path,
            const kinematics::State& state,
			double offset = 0);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
