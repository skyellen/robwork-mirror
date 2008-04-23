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

#ifndef RW_TASK_LINK_HPP
#define RW_TASK_LINK_HPP

/**
 * @file Link.hpp
 */

#include "Target.hpp"
#include "Entity.hpp"
#include "ToolSpeed.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/pathplanning/Path.hpp>

#include <boost/variant.hpp>
#include <string>

namespace rw { namespace task {
    class Target;

    /** @addtogroup task */
    /*@{*/

    class NoConstraint {};

    class LinearJointConstraint {};

    class LinearToolConstraint
    {
    public:
        LinearToolConstraint(const ToolSpeed &tool_speed) :
            _tool_speed(tool_speed)
        {}

    private:
        ToolSpeed _tool_speed;
    };

    class CircularToolConstraint
    {
    public:
        CircularToolConstraint(
            const ToolSpeed &tool_speed,
            const rw::math::Vector3D<> &via_point,
            rw::kinematics::Frame *via_frame)
            :
            _tool_speed(tool_speed),
            _via_point(via_point),
            _via_frame(via_frame)
        {}

    private:
        ToolSpeed _tool_speed;
        rw::math::Vector3D<> _via_point;
        rw::kinematics::Frame *_via_frame;
    };

    class Link : public Entity
    {
        friend class Trajectory;

    public:
        typedef boost::variant<
            NoConstraint,
            LinearJointConstraint,
            LinearToolConstraint,
            CircularToolConstraint> value_type;

        Link(
            const Entity& entity,
            const value_type& constraint);

        Target* getNext() const { return _next; }
        Target* getPrev() const { return _prev; }

        value_type& getValue() { return _value; }
		const value_type& getValue() const { return _value; }

        void setNext(Target *next) { _next = next; }
        void setPrev(Target *prev) { _prev = prev; }

    private:
		value_type _value;
        Target* _prev;
        Target* _next;
    };

}} // end namespaces

#endif
