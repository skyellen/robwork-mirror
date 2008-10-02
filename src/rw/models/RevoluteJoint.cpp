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

#include "RevoluteJoint.hpp"

#include <rw/math/EAA.hpp>
#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

RevoluteJoint::RevoluteJoint(
    const std::string& name,
    const Transform3D<>& transform)
    :
    Joint(name),
    _transform(transform)
{}

Transform3D<> RevoluteJoint::getRevoluteTransform(
    const Transform3D<>& displacement, double q)
{
    /* Faster version further below.

    return
        displacement *
        Transform3D<>(
            Vector3D<>(0, 0, 0),
            EAA<>(0, 0, q).toRotation3D());
    */

    const double ca = cos(q);
    const double sa = sin(q);

    const double r00 = ca; const double r01 = -sa;
    const double r10 = sa; const double r11 = ca;

    const double m00 = displacement.R()(0, 0);
    const double m01 = displacement.R()(0, 1);
    const double m02 = displacement.R()(0, 2);
    const double m10 = displacement.R()(1, 0);
    const double m11 = displacement.R()(1, 1);
    const double m12 = displacement.R()(1, 2);
    const double m20 = displacement.R()(2, 0);
    const double m21 = displacement.R()(2, 1);
    const double m22 = displacement.R()(2, 2);

    return Transform3D<>(
        displacement.P(),
        Rotation3D<>(
            m00 * r00 + m01 * r10,
            m00 * r01 + m01 * r11,
            m02,

            m10 * r00 + m11 * r10,
            m10 * r01 + m11 * r11,
            m12,

            m20 * r00 + m21 * r10,
            m20 * r01 + m21 * r11,
            m22));
}

void RevoluteJoint::getJointValueTransform(
    const Transform3D<>& parent,
    double q,
    Transform3D<>& result) const
{
    const double a00 = parent.R()(0, 0);
    const double a01 = parent.R()(0, 1);
    const double a02 = parent.R()(0, 2);
    const double a10 = parent.R()(1, 0);
    const double a11 = parent.R()(1, 1);
    const double a12 = parent.R()(1, 2);
    const double a20 = parent.R()(2, 0);
    const double a21 = parent.R()(2, 1);
    const double a22 = parent.R()(2, 2);
    const double ax = parent.P()(0);
    const double ay = parent.P()(1);
    const double az = parent.P()(2);

    const double b00 = _transform.R()(0, 0);
    const double b01 = _transform.R()(0, 1);
    const double b02 = _transform.R()(0, 2);
    const double b10 = _transform.R()(1, 0);
    const double b11 = _transform.R()(1, 1);
    const double b12 = _transform.R()(1, 2);
    const double b20 = _transform.R()(2, 0);
    const double b21 = _transform.R()(2, 1);
    const double b22 = _transform.R()(2, 2);
    const double bx = _transform.P()(0);
    const double by = _transform.P()(1);
    const double bz = _transform.P()(2);

    const double a00b00 = a00 * b00;
    const double a01b10 = a01 * b10;
    const double a01b11 = a01 * b11;
    const double a00b01 = a00 * b01;
    const double a02b21 = a02 * b21;
    const double a02b20 = a02 * b20;
    const double a10b00 = a10 * b00;
    const double a11b10 = a11 * b10;
    const double a11b11 = a11 * b11;
    const double a12b20 = a12 * b20;
    const double a12b21 = a12 * b21;
    const double a10b01 = a10 * b01;
    const double a20b00 = a20 * b00;
    const double a21b10 = a21 * b10;
    const double a22b20 = a22 * b20;
    const double a20b01 = a20 * b01;
    const double a21b11 = a21 * b11;
    const double a22b21 = a22 * b21;

    const double cq = cos(q);
    const double sq = sin(q);

    result.P() = Vector3D<>(
        ax + a00 * bx + a01 * by + a02 * bz,
        ay + a10 * bx + a11 * by + a12 * bz,
        az + a20 * bx + a21 * by + a22 * bz);

    result.R() = Rotation3D<>(
        (a00b00 + a01b10 + a02b20) * cq + (a00b01 + a01b11 + a02b21) * sq,
        (a00b01 + a01b11 + a02b21) * cq - (a00b00 + a01b10 + a02b20) * sq,
        a00 * b02 + a01 * b12 + a02 * b22,

        (a10b00 + a11b10 + a12b20) * cq + (a10b01 + a11b11 + a12b21) * sq,
        (a10b01 + a11b11 + a12b21) * cq - (a10b00 + a11b10 + a12b20) * sq,
        a10 * b02 + a11 * b12 + a12 * b22,

        (a20b00 + a21b10 + a22b20) * cq + (a20b01 + a21b11 + a22b21) * sq,
        (a20b01 + a21b11 + a22b21) * cq - (a20b00 + a21b10 + a22b20) * sq,
        a20 * b02 + a21 * b12 + a22 * b22);
}

void RevoluteJoint::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    getJointValueTransform(parent, *getQ(state), result);
}

Transform3D<> RevoluteJoint::getTransform(const State& state) const
{
    const double q = *getQ(state);
    return getRevoluteTransform(_transform, q);
}
