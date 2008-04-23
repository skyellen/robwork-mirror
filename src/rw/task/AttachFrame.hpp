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

#ifndef RW_TASK_ATTACHFRAME_HPP
#define RW_TASK_ATTACHFRAME_HPP

/**
   @file AttachFrame.hpp
*/

#include "Entity.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/common/PropertyMap.hpp>

#include <string>

namespace rw { namespace task {

	/** @addtogroup task */
    /*@{*/

    /**
       AttachFrame represents the action of executing a DAF command.
    */
	class AttachFrame : public Entity
	{
	public:
        /**
           Attach \b child to \b parent.

           Child and parent must be non-null.
         */
		AttachFrame(
            const Entity& entity,
            rw::kinematics::MovableFrame* child,
            rw::kinematics::Frame* parent);

        /**
           The parent to which to the child is attached.
        */
		rw::kinematics::Frame& getParent() const { return *_parent; }

        /**
           The child frame to attach to the parent.
        */
		rw::kinematics::MovableFrame& getChild() const { return *_child; }

    private:
		rw::kinematics::MovableFrame *_child;
		rw::kinematics::Frame *_parent;
	};

    /**@}*/
}} // end namespaces

#endif // end include guard
