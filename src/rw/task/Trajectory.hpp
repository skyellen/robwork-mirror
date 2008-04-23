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
       Trajectory describes a motion to carry out for a particular device and
       tcp.

       A trajectory contains a sequence of targets seperated by links. If a pair
       of targets are not seperated by a link, it means that there are no
       constraints (aside from collision avoidance) on the motion connecting the
       targets.
    */
	class Trajectory : public Entity
	{
	public:
        //! Variant type for trajectory elements.
        typedef boost::variant<Link, Target> Element;

        //! Value type.
        typedef Element value_type;

		//! Iterator for the sequence of trajectory elements.
		typedef std::list<value_type>::iterator iterator;

		//! Const iterator for the sequence of trajectory elements.
		typedef std::list<value_type>::const_iterator const_iterator;

        /**
           Constructor

           \b workcell, \b device and \b tool_frame must all be non-null.

           \b workcell, \b device and \b tool_frame are stored in the
           trajectory, but ownership is not taken.
        */
		Trajectory(
            const Entity& entity,
            rw::models::WorkCell* workcell,
            rw::models::Device* device,
            rw::kinematics::Frame* tool_frame,
            const std::vector<Element>& elements);

        /**
           The workcell for which the motion is planned.
        */
		rw::models::WorkCell& getWorkCell() const { return *_workcell; }

        /**
           The device for which to execute the motion.
        */
		rw::models::Device& getDevice() const { return *_device; }

        /**
           The tool of the device.
        */
    	rw::kinematics::Frame& getToolFrame() const { return *_tool_frame; }

        /**
           The targets and links describing the motion to carry out.
        */
        std::pair<iterator, iterator> getElements()
        { return std::make_pair(_elements.begin(), _elements.end()); }

        /**
           The targets and links describing the motion to carry out.
        */
        std::pair<const_iterator, const_iterator> getElements() const
        { return std::make_pair(_elements.begin(), _elements.end()); }

	private:
		rw::models::WorkCell *_workcell;
		rw::models::Device *_device;
		rw::kinematics::Frame *_tool_frame;

        std::list<value_type> _elements;
	};

}} // end namespaces

#endif
