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
        static std::vector<TimedQ> MakeTimedQPath(const math::Q& speed,
                                                  const std::vector<math::Q>& path);

        /**
           @brief A path of time stamped states.

           The time stamp of the first state is zero, and the time for the
           remaining states are computed using the maximum joint speed
           velocities of \b workcell.
         */
        static TimedStatePath MakeTimedStatePath(const models::WorkCell& workcell,
                                                 const std::vector<kinematics::State>& path);
        
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
