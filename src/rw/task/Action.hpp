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

#ifndef RW_TASK_Action_HPP 
#define RW_TASK_Action_HPP 

/**
 * @file Action.hpp
 */

#include "Property.hpp"
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <iostream>
#include <string.h>

#include <boost/variant.hpp>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for task action specifications.
     *
	 * TODO: Longer description
     */
	class AttachFrameAction
	{
	public:
		AttachFrameAction(
            rw::kinematics::MovableFrame *child,
            rw::kinematics::Frame *parent)
            :
            _child(child),
            _parent(parent)
		{}

		rw::kinematics::Frame *getParent() { return _parent; }
		rw::kinematics::MovableFrame *getChild() { return _child; }

	private:
		rw::kinematics::MovableFrame *_child;
		rw::kinematics::Frame *_parent;
	};

	class NoAction {};

	class Action
	{	
	public:
		typedef boost::variant<AttachFrameAction, NoAction> ActionType;

		Action(const std::string &name) :
            _action_type(NoAction()),
            _name(name)
		{}
	
		Action(const ActionType &action_type, const std::string &name) :
            _action_type(action_type),
            _name(name)
		{}

		~Action() {}

		Property &Properties() { return _properties; }

		ActionType getActionType() { return _action_type; }

	private:
		ActionType _action_type;
		std::string _name;
		Property _properties;
	};

}} // end namespaces

#endif
