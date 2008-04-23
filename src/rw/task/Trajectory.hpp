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
   @file Trajectory.hpp
*/

#include "Target.hpp"
#include "Link.hpp"
#include "Entity.hpp"

#include <string>
#include <list>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
       @brief Trajectories in task objects.
     */
	class Trajectory : public Entity
	{
	public:
		typedef std::list<Target>::iterator target_iterator;
		typedef std::list<Target>::const_iterator const_target_iterator;

		typedef std::list<Link>::iterator link_iterator;
		typedef std::list<Link>::const_iterator const_link_iterator;

		Trajectory(
            const Entity& entity,
            rw::models::WorkCell *workcell,
            rw::models::Device *device,
            rw::kinematics::Frame *tool_frame);

		Trajectory(const Trajectory &trajectory);

		void addTarget(const Target &target);
		void addLink(const Link &link);

        std::pair<link_iterator, link_iterator>
        getLinks()
        { return std::make_pair(link_list.begin(), link_list.end()); }

        std::pair<const_link_iterator, const_link_iterator>
        getLinks() const
        { return std::make_pair(link_list.begin(), link_list.end()); }

        std::pair<target_iterator, target_iterator> getTargets()
        { return std::make_pair(target_list.begin(), target_list.end()); }

        std::pair<const_target_iterator, const_target_iterator>
        getTargets() const
        { return std::make_pair(target_list.begin(), target_list.end()); }

		bool emptyLinks() { return link_list.empty(); }
		bool emptyTargets() { return target_list.empty(); }

		int sizeLinks() { return link_list.size(); }
		int sizeTargets() { return target_list.size(); }

		rw::models::WorkCell& getWorkCell() const { return *_workcell; }
		rw::models::Device& getDevice() const { return *_device; }
    	rw::kinematics::Frame& getToolFrame() const { return *_tool_frame; }

	private:
		rw::models::WorkCell *_workcell;
		rw::models::Device *_device;
		rw::kinematics::Frame *_tool_frame;

		std::list<Target> target_list;
		std::list<Link> link_list;

		bool insert_link;
	};

}} // end namespaces

#endif
