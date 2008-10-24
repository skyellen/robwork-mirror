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

#ifndef RW_MODELS_TIMEMETRICUTIL_HPP
#define RW_MODELS_TIMEMETRICUTIL_HPP

#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include "WorkCell.hpp"
#include "Device.hpp"

namespace rw { namespace models {

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
