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
		typedef std::list<Target>::const_iterator const_target_iterator;

		typedef std::list<Link>::iterator link_iterator;
		typedef std::list<Link>::const_iterator const_link_iterator;

		Trajectory(
            rw::models::WorkCell *workcell,
            rw::models::Device *device,
            rw::kinematics::Frame *tool_frame,
            const std::string& name = "");
        
		Trajectory(const Trajectory &trajectory);

		void addTarget(const Target &target);
		void addLink(const Link &link);

		link_iterator link_begin() { return link_list.begin(); }
		link_iterator link_end() { return link_list.end(); }

		target_iterator target_begin() { return target_list.begin(); }
		target_iterator target_end() { return target_list.end(); }

		const_link_iterator link_begin() const { return link_list.begin(); }
		const_link_iterator link_end() const { return link_list.end(); }

		const_target_iterator target_begin() const { return target_list.begin(); }
		const_target_iterator target_end() const { return target_list.end(); }

		bool empty() { return link_list.empty(); }
		int nrOfLinks() { return link_list.size(); }
		int nrOfTargets() { return target_list.size(); }

		rw::models::WorkCell *getWorkCell() const { return _workcell; }
		rw::models::Device *getDevice() const { return _device; }
    	rw::kinematics::Frame *getToolFrame() const { return _tool_frame; }

		void replaceTarget(Target &target1, Target &target2);
		void replaceLink(Link &link1, Link &link2);

		void storeState(const rw::kinematics::State &state)
        { _state = state; }

		rw::kinematics::State getState() const { return _state; }

        const std::string& getName() const { return _name; }

	private:
		rw::models::WorkCell *_workcell;
		rw::models::DeviceModel *_device;
		rw::kinematics::Frame *_tool_frame;
        std::string _name;

		std::list<Target> target_list;
		std::list<Link> link_list;

		bool insert_link;

		rw::kinematics::State _state;
	};

}} // end namespaces

#endif
