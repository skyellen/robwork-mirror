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
 * for detailed information about these packages.
 *********************************************************************/

#include "PrismaticJoint.hpp"

#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

namespace
{
    class TULPrismaticJoint : public PrismaticJoint
    {
    public:
        TULPrismaticJoint(
            const std::string& name,
            const Transform3D<>& transform)
            :
            PrismaticJoint(name),
            _transform(transform)
        {}

    private:
        void doGetJointValueTransform(
            const Transform3D<>& parent,
            double q,
            Transform3D<>& result) const
        {
            Rotation3D<>::rotationMultiply(parent.R(), _transform.R(), result.R());

            const double bx = _transform.P()(0);
            const double by = _transform.P()(1);
            const double bz = _transform.P()(2);

            const double b02 = _transform.R()(0, 2);
            const double b12 = _transform.R()(1, 2);
            const double b22 = _transform.R()(2, 2);
            const Vector3D<> p(bx + b02 * q, by + b12 * q, bz + b22 * q);

            Rotation3D<>::rotationVectorMultiply(parent.R(), p, result.P());
            result.P() += parent.P();
        }

    private:
        Transform3D<> _transform;
    };

    class TULPrismaticJoint_zero_offset : public PrismaticJoint
    {
    public:
        TULPrismaticJoint_zero_offset(
            const std::string& name,
            const Rotation3D<>& rotation)
            :
            PrismaticJoint(name),
            _rotation(rotation)
        {}

    private:
        void doGetJointValueTransform(
            const Transform3D<>& parent,
            double q,
            Transform3D<>& result) const
        {
            Rotation3D<>::rotationMultiply(parent.R(), _rotation, result.R());

            const double ab02 = result.R()(0, 2);
            const double ab12 = result.R()(1, 2);
            const double ab22 = result.R()(2, 2);
            result.P() =
                parent.P() +
                Vector3D<>(ab02 * q, ab12 * q, ab22 * q);
        }

    private:
        Rotation3D<> _rotation;
    };
}

//----------------------------------------------------------------------
// PrismaticJoint
//----------------------------------------------------------------------

void PrismaticJoint::getJointValueTransform(
    const Transform3D<>& parent,
    double q,
    Transform3D<>& result) const
{
    doGetJointValueTransform(parent, q, result);
}

void PrismaticJoint::doGetTransform(
    const Transform3D<>& parent,
    const State& state,
    Transform3D<>& result) const
{
    getJointValueTransform(parent, *getQ(state), result);
}

//----------------------------------------------------------------------
// Constructors

PrismaticJoint* PrismaticJoint::make(
    const std::string& name,
    const Transform3D<>& transform)
{
    if (transform.P() == Vector3D<>(0, 0, 0))
		return new TULPrismaticJoint_zero_offset(name, transform.R());
    else
        return new TULPrismaticJoint(name, transform);

    // More cases can be added for joints with a change in rotation of zero
    // (which is also a common case).
}
