/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_TASK_CIRCULARTOOLCONSTRAINT_HPP
#define RW_TASK_CIRCULARTOOLCONSTRAINT_HPP

/**
   @file CircularToolConstraint.hpp
*/

#include "ToolSpeed.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace task {

    /** @addtogroup task */
    /*@{*/

    /**
       @brief CircularToolConstraint specifies that the tool in between a pair
       of targets should follow a circular curve.

       The circle is described by a point relative to a frame through which the
       circular arc should pass.
    */
    class CircularToolConstraint
    {
    public:
        /**
           Constructor
        */
        CircularToolConstraint(
            const ToolSpeed &tool_speed,
            const rw::math::Vector3D<>& via_point,
            rw::kinematics::Frame* via_frame)
            :
            _tool_speed(tool_speed),
            _via_point(via_point),
            _via_frame(via_frame)
        {}

        /**
           The tool speed along the curve.
        */
        const ToolSpeed& getToolSpeed() const { return _tool_speed; }

        /**
           The intermediary point on the curve relative to the frame.
        */
        const rw::math::Vector3D<>& getPoint() const { return _via_point; }

        /**
           The frame relative to which the intermediary point is given.
        */
        rw::kinematics::Frame* getFrame() const { return _via_frame; }

    private:
        ToolSpeed _tool_speed;
        rw::math::Vector3D<> _via_point;
        rw::kinematics::Frame* _via_frame;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
