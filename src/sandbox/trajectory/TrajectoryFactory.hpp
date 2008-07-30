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

#ifndef RW_TRAJECTORY_TRAJECTORYFACTORY_HPP
#define RW_TRAJECTORY_TRAJECTORYFACTORY_HPP

#include "Timed.hpp"
#include "Path.hpp"
#include <rw/trajectory/Trajectory.hpp>
#include <rw/kinematics/State.hpp>

#include <vector>
#include <memory>



namespace rw { namespace models {
    class WorkCell;
}}

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/


    /**
     * @brief Trajectory constructors
     */
    class TrajectoryFactory
    {
    public:



        /**
         * @brief A trajectory for the straight line path \b path that is linearly
         * traversed with maximum speeds of the devices of \b workcell.
         * The path must be of length at least two.
         */
//        static std::auto_ptr<Trajectory<rw::kinematics::State> > makeLinearTrajectory(const StatePath& path);

        static Trajectory<rw::kinematics::State> makeLinearTrajectory(const StatePath& path);




        /**
         * @brief A trajectory for the path \b path that is
         * linearly traversed to match the provided time values.
         *
         *  The path must be of length at least two.
         */
        static Trajectory<rw::kinematics::State> makeLinearTrajectory(const TimedStatePath& path);

        /**
         * @brief A trajectory containing no states.
         *
         *  The end time of the trajectory is negative. Calling the get() method will
         *  throw an exception always, because the trajectory range is empty.
         */
        static Trajectory<rw::kinematics::State> makeEmptyStateTrajectory();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
