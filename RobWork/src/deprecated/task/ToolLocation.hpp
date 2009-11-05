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


#ifndef RW_TASK_TOOLLOCATION_HPP
#define RW_TASK_TOOLLOCATION_HPP

/**
   @file ToolLocation.hpp
*/

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief ToolLocation specifies a 3D transform to reach with the tool of a
       device.

       The tool location is a tuple of (\b transform, \b frame): Relative to \b
       frame the tool should reach \b transform. The transform of \b frame
       depends on the current state of the workcell and therefore the tool
       transform to reach relative to the world frame can not be computed in
       advance.
    */
	class ToolLocation
	{
	public:
        /**
           Constructor
        */
		ToolLocation(
            const rw::math::Transform3D<> &transform,
            rw::kinematics::Frame *frame);

        /**
           The transform relative to the frame to reach.
        */
		const rw::math::Transform3D<> &getTransform() const { return _transform; }

        /**
           The frame relative to which the transform is given.
        */
		rw::kinematics::Frame& getFrame() const { return *_frame; }

	private:
		rw::math::Transform3D<> _transform;
		rw::kinematics::Frame* _frame;
	};

    /**@}*/
}} // end namespaces

#endif // end include guard
