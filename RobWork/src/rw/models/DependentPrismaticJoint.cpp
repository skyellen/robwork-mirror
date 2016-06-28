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


#include "DependentPrismaticJoint.hpp"
#include "PrismaticJoint.hpp"
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
    const double q_owner = _owner->getData(state)[0];
    const double q = _scale * q_owner + _offset;

    _helper.multiplyJointTransform(parent, Q(1,q), result);
}

Transform3D<> DependentPrismaticJoint::doGetTransform(const State& state) const
{
    const double q_owner = _owner->getData(state)[0];
    const double q = _scale * q_owner + _offset;

    return _helper.getTransform(q);
}


Jacobian DependentPrismaticJoint::doGetJacobian(const State& state) const {
    return Jacobian(6,1);
}


void DependentPrismaticJoint::getJacobian(size_t row, size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
    const Vector3D<> axis = joint.R().getCol(2);

    jacobian.addPosition(_scale * axis, row, col);
}


void DependentPrismaticJoint::setFixedTransform( const rw::math::Transform3D<>& t3d) {
    _helper.setFixedTransform( t3d );
}

rw::math::Transform3D<> DependentPrismaticJoint::getJointTransform(const rw::kinematics::State& state) const{
    const double q_owner = _owner->getData(state)[0];
    const double q = _scale * q_owner + _offset;

    return _helper.getJointTransform(q);
}


