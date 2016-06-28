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


#ifndef RW_TRAJECTORY_TRAJECTORYFACTORY_HPP
#define RW_TRAJECTORY_TRAJECTORYFACTORY_HPP

#include "Timed.hpp"
#include "Path.hpp"
#include <rw/math/Metric.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Ptr.hpp>

#include <vector>

namespace rw { namespace models { class Device; }}
namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace trajectory {

    //template <class T>
    //class Trajectory;

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Trajectory constructors
     */
    class TrajectoryFactory
    {
    public:
        /**
           @brief A trajectory with value \b state and velocity and acceleration
           0. The trajectory runs from time 0 to DBL_MAX.
        */
		static StateTrajectory::Ptr makeFixedTrajectory(const rw::kinematics::State& state, double duration);

        /**
         * @brief A trajectory with a fixed value q and zero velocity and acceleration.
         *
         * @param q [in] The fixed value of the trajectory
         * @param duration [in] The duration of the trajectory
         */
		static QTrajectory::Ptr makeFixedTrajectory(const rw::math::Q& q, double duration);
        
        /**
           @brief A trajectory for the path \b path that is
           linearly traversed to match the provided time values.

           The path must be of length at least two.
        */
		static StateTrajectory::Ptr makeLinearTrajectory(const TimedStatePath& path);

        /**
           @brief A trajectory for the straight line path \b path that is
           linearly traversed with maximum speeds of the devices of \b workcell.

           The path must be of length at least two.
        */
		static StateTrajectory::Ptr makeLinearTrajectory(const StatePath& path,
			const models::WorkCell& workcell);

        
        /**
           @brief A trajectory for the straight line path \b path being
           traversed with a time distance of 1 between adjacent states.

           This function is not very useful for anything, but it happens to be
           used in the trajectory module test.
        */
		static StateTrajectory::Ptr makeLinearTrajectoryUnitStep(const StatePath& path);
        

        /**
           @brief A linearly traversed trajectory for the path \b path.
        */
		static QTrajectory::Ptr makeLinearTrajectory(const TimedQPath& path);

        /**
           @brief A linearly traversed trajectory for the path \b path with time
           values set to match the joint velocities \b speed.
        */
		static QTrajectory::Ptr makeLinearTrajectory(const QPath& path, const rw::math::Q& speeds);

        /**
           @brief A linearly traversed trajectory for the path \b path of device
           \b device with time values set to match the maximum joint velocities
           of the \b device.
        */
		static QTrajectory::Ptr makeLinearTrajectory(const QPath& path, const models::Device& device);


        /**
         * @brief Constructs a linear trajectory for the path \b path in which the
         * time corresponds to the length measured with \b metric
         */
		static QTrajectory::Ptr makeLinearTrajectory(const QPath& path, rw::math::QMetric::Ptr metric);


		/**
		 * @brief Constructs a linear trajectory for the path \b path. Times represents the
		 * time for each segment
		 *
		 * @param path [in] path containing poses
		 * @param times [in] times for each segment
		 */
		static Transform3DTrajectory::Ptr makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times);

		/**
		 * @brief Constructs a linear trajectory for the path \b path. The \b metric is used to calculate the 
		 * length of each segment
		 * @param path [in] path containing poses
		 * @param metric [in] metric for calculating length of segments
		 */
		static Transform3DTrajectory::Ptr makeLinearTrajectory(const Transform3DPath& path, const rw::math::Transform3DMetric::Ptr metric);

        /**
           @brief A trajectory containing no states.

           The end time of the trajectory is negative. Calling the get() method
           will throw an exception always, because the trajectory range is
           empty.
        */
		static StateTrajectory::Ptr makeEmptyStateTrajectory();

        /**
           @brief A trajectory containing no configurations.

           The end time of the trajectory is negative. Calling the get() method
           will throw an exception always, because the trajectory range is
           empty.
        */
		static QTrajectory::Ptr makeEmptyQTrajectory();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
