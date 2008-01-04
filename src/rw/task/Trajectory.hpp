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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TASK_TRAJECTORY_HPP
#define RW_TASK_TRAJECTORY_HPP

/**
 * @file Trajectory.hpp
 */

#include "Target.hpp"
#include "Link.hpp"

#include <iostream>
#include <string>
#include <list>

#include <rw/models/WorkCell.hpp>
#include <rw/models/DeviceModel.hpp>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for motion trajectories in task objects.
     *
	 * TODO: Longer description
     */

	class Trajectory
	{
	public:
		typedef std::list<Target>::iterator target_iterator;
		typedef std::list<Link>::iterator link_iterator;

		Trajectory(
            rw::models::WorkCell *workcell,
            rw::models::Device *device,
            rw::kinematics::Frame *tool_frame);

		void addTarget(const Target &target);

		void addLink(const Link &link);

		link_iterator link_begin() { return link_list.begin(); }
		target_iterator target_begin() { return target_list.begin(); }

		link_iterator link_end() { return link_list.end(); }
		target_iterator target_end() { return target_list.end(); }

		bool empty() { return link_list.empty(); }
		int nrOfLinks() { return link_list.size(); }
		int nrOfTargets() { return target_list.size(); }

		rw::math::Transform3D<> getBaseTransform(Target &target);
		rw::math::Transform3D<> getWorldTransform(Target &target);

		//krav skal være tool-constraint
		//hvis target er joint regnes transform ud....
		rw::interpolator::Pose6dStraightSegment getInterpolator(Link &link);

	private:
		rw::models::WorkCell *_workcell;
		rw::models::DeviceModel *_device;
		rw::kinematics::Frame *_tool_frame;

		rw::math::Transform3D<> _tool_to_end_transform;
		std::list<Target> target_list;
		std::list<Link> link_list;

		bool insert_link;
	};

}} // end namespaces

#endif
