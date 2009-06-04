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
#include <rw/math/Rotation3D.hpp>
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;






//----------------------------------------------------------------------
// PrismaticJoint
//----------------------------------------------------------------------


PrismaticJoint::PrismaticJoint(const std::string& name, const math::Transform3D<>& transform):
    Joint(name,1)
{
    const Rotation3D<>& rot = transform.R();
    if (rot == Rotation3D<>::identity())
        _impl = new PrismaticJointZeroRotationImpl(transform.P());
    else if (transform.P() == Vector3D<>(0, 0, 0))
        _impl = new PrismaticJointZeroOffsetImpl(transform.R());
    else
        _impl = new PrismaticJointImplBasic(transform);

}
/*
void PrismaticJoint::getJointValueTransform(const Transform3D<>& parent,
                                            double q,
                                            Transform3D<>& result) const
{
    doGetJointValueTransform(parent, q, result);
}*/


void PrismaticJoint::multiplyJointTransform(const Transform3D<>& parent,
                                              const Q& q,
                                              Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, q(0), result);
}


Transform3D<> PrismaticJoint::getJointTransform(const Q& q) const
{
    return _impl->getTransform(q(0));
}


void PrismaticJoint::doMultiplyTransform(const Transform3D<>& parent,
                                         const State& state,
                                         Transform3D<>& result) const
{
    _impl->multiplyTransform(parent, getQ(state)[0], result);
}

Transform3D<> PrismaticJoint::doGetTransform(const State& state) const
{
    return _impl->getTransform(getQ(state)[0]);
}

void PrismaticJoint::getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const {
    const Vector3D<> axis = joint.R().getCol(2);

    jacobian.addPosition(axis, row, col);
}

//----------------------------------------------------------------------
// Constructors
/*
PrismaticJoint* PrismaticJoint::make(
    const std::string& name,
    const Transform3D<>& transform)
{
    if (transform.P() == Vector3D<>(0, 0, 0))
		return new PrismaticJointZeroOffsetImpl(name, transform.R());
    else
        return new PrismaticJointImpl(name, transform);

    // More cases can be added for joints with a change in rotation of zero
    // (which is also a common case).
}
*/
