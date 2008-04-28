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
