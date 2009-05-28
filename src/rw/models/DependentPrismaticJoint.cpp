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

#include "DependentPrismaticJoint.hpp"
#include "PrismaticJoint.hpp"
#include <rw/math/EAA.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/macros.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

DependentPrismaticJoint::DependentPrismaticJoint(const std::string& name,
                                                 const Transform3D<>& transform,
                                                 Joint* owner,
                                                 double scale,
                                                 double offset):
    DependentJoint(name),
    _helper(name, transform),
    _owner(owner),
    _scale(scale),
    _offset(offset)
{
    RW_ASSERT(_owner);
}

void DependentPrismaticJoint::doMultiplyTransform(const Transform3D<>& parent,
                                                  const State& state,
                                                  Transform3D<>& result) const
{
    const double q_owner = _owner->getQ(state)[0];
    const double q = _scale * q_owner + _offset;

    _helper.multiplyJointTransform(parent, Q(1,q), result);
}

Transform3D<> DependentPrismaticJoint::doGetTransform(const State& state) const
{
    const double q_owner = _owner->getQ(state)[0];
    const double q = _scale * q_owner + _offset;

    return _helper.getJointTransform(Q(1,q));
}


Jacobian DependentPrismaticJoint::doGetJacobian(const kinematics::State& state) const {
    return Jacobian(6,1);
}


void DependentPrismaticJoint::getJacobian(size_t row, size_t col, const math::Transform3D<>& joint, const math::Transform3D<>& tcp, math::Jacobian& jacobian) const {
    const Vector3D<> axis = joint.R().getCol(2);

    jacobian.addPosition(_scale * axis, row, col);
}
