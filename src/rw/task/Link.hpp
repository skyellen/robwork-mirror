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

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <rw/interpolator/StraightSegment.hpp>
#include <rw/interpolator/Pose6dStraightSegment.hpp>

#include <rw/pathplanning/PathPlanner.hpp>

#include <rw/kinematics/Frame.hpp>

#include <boost/variant.hpp>

namespace rw { namespace task {
    class Target;

    /** @addtogroup task */
    /*@{*/

    /**
     * @brief Data structure for Link specifications in task trajectories.
     *
     * TODO: Longer description
     */
    class ToolSpeed
    {
    public:
        enum SpeedType {Angular, Positional};

        ToolSpeed(SpeedType speed_type, double tool_speed) :
            _speed_type(speed_type),
            _tool_speed(tool_speed)
        {}

        double getToolSpeed() { return _tool_speed; }
        SpeedType getSpeedType() { return _speed_type; }

    private:
        SpeedType _speed_type;
        double _tool_speed;

    };

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

    class Link
    {
        friend class Trajectory;

    public:
        typedef boost::variant<
            NoConstraint,
            LinearJointConstraint,
            LinearToolConstraint,
            CircularToolConstraint> MotionConstraint;

        Link(const std::string &name="");
        Link(const MotionConstraint &motion_constraint, const std::string &name="");
        ~Link();

        Property &Properties() { return _properties; };

        Target *next() const { return _next; }
        Target *prev() const { return _prev; }

        MotionConstraint &getMotionConstraint() {return _motion_constraint; }

        std::string getName() { return _name; }

        void saveSolvedPath(rw::pathplanning::Path solved_path)
        { _solved_path = solved_path; }

        rw::pathplanning::Path getSolvedPath() { return _solved_path; }

        bool isNoConstraint() const
        { return _motion_constraint.type() == typeid(NoConstraint); }

        bool isLinearJointConstraint() const
        { return _motion_constraint.type() == typeid(LinearJointConstraint); }

        bool isLinearToolConstraint() const
        { return _motion_constraint.type() == typeid(LinearToolConstraint); }

        bool isCircularToolConstraint() const
        { return _motion_constraint.type() == typeid(CircularToolConstraint); }

    private:
        MotionConstraint _motion_constraint;

        void setNext(Target *next) { _next = next; }
        void setPrev(Target *prev) { _prev = prev; }

        Property _properties;
        Target *_prev;
        Target *_next;
        rw::pathplanning::Path _solved_path;
        std::string _name;
    };

}} // end namespaces

#endif
