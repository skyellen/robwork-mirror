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
       @brief AttachFrame represents the action of executing a DAF command.
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
