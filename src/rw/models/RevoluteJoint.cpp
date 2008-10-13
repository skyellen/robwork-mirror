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

//----------------------------------------------------------------------
// TULRevoluteJoint
//----------------------------------------------------------------------

namespace
{
    class TULRevoluteJoint : public RevoluteJoint
    {
    public:
        TULRevoluteJoint(
            const std::string& name,
            const Transform3D<>& transform)
            :
            RevoluteJoint(name),
            _transform(transform)
        {}

    private:
        void doGetJointValueTransform(
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

            const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
            const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
            const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
            const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
            const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
            const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

            const double cq = cos(q);
            const double sq = sin(q);

            result.P() = Vector3D<>(
                ax + a00 * bx + a01 * by + a02 * bz,
                ay + a10 * bx + a11 * by + a12 * bz,
                az + a20 * bx + a21 * by + a22 * bz);

            result.R() = Rotation3D<>(
                a00b00_a01b10_a02b20 * cq + a00b01_a01b11_a02b21 * sq,
                a00b01_a01b11_a02b21 * cq - a00b00_a01b10_a02b20 * sq,
                a00 * b02 + a01 * b12 + a02 * b22,

                a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                a10b01_a11b11_a12b21 * cq - a10b00_a11b10_a12b20 * sq,
                a10 * b02 + a11 * b12 + a12 * b22,

                a20b00_a21b10_a22b20 * cq + a20b01_a21b11_a22b21 * sq,
                a20b01_a21b11_a22b21 * cq - a20b00_a21b10_a22b20 * sq,
                a20 * b02 + a21 * b12 + a22 * b22);
        }

    private:
        Transform3D<> _transform;
    };

    class TULRevoluteJoint_zero_offset : public RevoluteJoint
    {
    public:
        TULRevoluteJoint_zero_offset(
            const std::string& name,
            const Rotation3D<>& rotation)
            :
            RevoluteJoint(name),
            _transform(rotation)
        {}

    private:
        void doGetJointValueTransform(
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

            const double b00 = _transform.R()(0, 0);
            const double b01 = _transform.R()(0, 1);
            const double b02 = _transform.R()(0, 2);
            const double b10 = _transform.R()(1, 0);
            const double b11 = _transform.R()(1, 1);
            const double b12 = _transform.R()(1, 2);
            const double b20 = _transform.R()(2, 0);
            const double b21 = _transform.R()(2, 1);
            const double b22 = _transform.R()(2, 2);

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

            const double a20b01_a21b11_a22b21 = a20b01 + a21b11 + a22b21;
            const double a20b00_a21b10_a22b20 = a20b00 + a21b10 + a22b20;
            const double a10b01_a11b11_a12b21 = a10b01 + a11b11 + a12b21;
            const double a00b01_a01b11_a02b21 = a00b01 + a01b11 + a02b21;
            const double a10b00_a11b10_a12b20 = a10b00 + a11b10 + a12b20;
            const double a00b00_a01b10_a02b20 = a00b00 + a01b10 + a02b20;

            const double cq = cos(q);
            const double sq = sin(q);

            result.P() = parent.P();

            result.R() = Rotation3D<>(
                a00b00_a01b10_a02b20 * cq + a00b01_a01b11_a02b21 * sq,
                a00b01_a01b11_a02b21 * cq - a00b00_a01b10_a02b20 * sq,
                a00 * b02 + a01 * b12 + a02 * b22,

                a10b00_a11b10_a12b20 * cq + a10b01_a11b11_a12b21 * sq,
                a10b01_a11b11_a12b21 * cq - a10b00_a11b10_a12b20 * sq,
                a10 * b02 + a11 * b12 + a12 * b22,

                a20b00_a21b10_a22b20 * cq + a20b01_a21b11_a22b21 * sq,
                a20b01_a21b11_a22b21 * cq - a20b00_a21b10_a22b20 * sq,
                a20 * b02 + a21 * b12 + a22 * b22);
        }

    private:
        Transform3D<> _transform;
    };
}

//----------------------------------------------------------------------
// RevoluteJoint
//----------------------------------------------------------------------

void RevoluteJoint::getJointValueTransform(
    const Transform3D<>& parent,
    double q,
    Transform3D<>& result) const
{
    doGetJointValueTransform(parent, q, result);
}

void RevoluteJoint::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    doGetJointValueTransform(parent, *getQ(state), result);
}

//----------------------------------------------------------------------
// Constructors

RevoluteJoint* RevoluteJoint::make(
    const std::string& name,
    const Transform3D<>& transform)
{
    if (transform.P() == Vector3D<>(0, 0, 0))
        // return new TULRevoluteJoint_zero_offset(name, transform.R());
        return new TULRevoluteJoint(name, transform);
    else
        return new TULRevoluteJoint(name, transform);
}
