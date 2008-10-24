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

#ifndef RW_TRAJECTORY_TIMEDUTIL_HPP
#define RW_TRAJECTORY_TIMEDUTIL_HPP

#include "Timed.hpp"
#include "Path.hpp"

#include <rw/math/Q.hpp>

/**
   @file TimedUtil.hpp
   @brief Class rw::trajectory::TimedUtil
*/

#include <rw/kinematics/State.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <boost/foreach.hpp>

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

           The time stamp of the first state is zero, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b workcell.
         */
        static TimedStatePath makeTimedStatePath(
            const models::WorkCell& workcell,
            const StatePath& path);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is zero, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b device.
        */
        static TimedStatePath makeTimedStatePath(
            const models::Device& device,
            const QPath& path,
            const kinematics::State& state);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
