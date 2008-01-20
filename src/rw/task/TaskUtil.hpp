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

#ifndef RW_TASK_TaskUtil_HPP
#define RW_TASK_TaskUtil_HPP

/**
 * @file Action.hpp
 */

#include "Task.hpp"

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Class with various utility functions for task solving
     */
	class TaskUtil
	{
	public:
		static rw::math::Transform3D<> getBaseTransform(
            const Trajectory &trajectory, const Target &target);
		static rw::math::Transform3D<> getWorldTransform(
            const Trajectory &trajectory, const Target &target);

		static rw::interpolator::Pose6dStraightSegment getPoseInterpolator(
            const Trajectory &trajectory, const Link &link);

		static rw::interpolator::Pose6dStraightSegment getStraigtInterpolator(
            rw::math::Transform3D<> a, rw::math::Transform3D<> b);
		static rw::interpolator::Pose6dStraightSegment getCircularInterpolator(
            rw::math::Transform3D<> a, rw::math::Transform3D<> b);

		static double getLength(
            const Trajectory &trajectory, const rw::task::Link &link);

		static std::vector<rw::kinematics::State> getStatePath(const Task &task);

		static Link CombineLinks(const Link link1, const Link &link2);
	};

}} // end namespaces

#endif
