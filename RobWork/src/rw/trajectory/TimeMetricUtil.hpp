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


#ifndef RW_TRAJECTORY_TIMEMETRICUTIL_HPP
#define RW_TRAJECTORY_TIMEMETRICUTIL_HPP

namespace rw { namespace kinematics { class State; }}
namespace rw { namespace math { class Q; }}
namespace rw { namespace models { class Device; }}
namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace trajectory {
	/**
	 * @brief methods for analyzing time distance of various paths
	 */
	class TimeMetricUtil
	{
	public:
	    /**
	     * @brief The time it takes to move from \b from to \b to in a straight line
	     * with maximum device velocities given by \b workcell.
	     *
	     * @param from [in] Start state
	     * @param to [in] End state
	     * @param workcell [in] Workcell associated with the states
	     * @return The time
	     */
	    static double timeDistance(
            const rw::kinematics::State& from,
            const rw::kinematics::State& to,
            const rw::models::WorkCell& workcell);

	    /**
	     * @brief The time it takes to move from \b from to \b to in a straight line
	     * with maximum velocities \b velocity.
	     *
	     * This is an example of a scaled max-norm.
	     *
	     * @param from [in] Start configuration
	     * @param to [in] End configurationp
	     * @param velocity [in] Max velocity of the joints
	     * @return The time
	     */
	    static double timeDistance(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const rw::math::Q& velocity);

	    /**
	     * @brief The time it takes to move from \b from to \b to in a straight
	     * line with the maximum velocities given of \b device.
	     *
	     * @param from [in] Start configuration
	     * @param to [in] End configuration
	     * @param device [in] The device to time for
	     * @return The time
	     */
	    static double timeDistance(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const rw::models::Device& device);

	    /**
	     * @brief The time it takes to move from \b from to \b to in a straight
	     * line with the maximum velocities given of \b device.
	     *
	     * @param from [in] Start state
	     * @param to [in] End state
	     * @param device [in] The device to time for
	     * @return The time
	     */
	    static double timeDistance(
            const rw::kinematics::State& from,
            const rw::kinematics::State& to,
            const rw::models::Device& device);
	};

}} // end namespaces

#endif // end include guard
