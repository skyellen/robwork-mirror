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


#include "BeamJoint.hpp"

#include <rw/math/EAA.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

BeamJoint::BeamJoint(const std::string& name, const Transform3D<>& transform) :
                     Joint(name, 1), _transform(transform),
                     _L(1), _E(1), _I(1)
{
    setBounds(std::pair<const math::Q, const math::Q>(math::Q(1, 0.0), math::Q(1, 100.0)));
}

BeamJoint::~BeamJoint()
{
}


Transform3D<> BeamJoint::getJointTransform(const Q& F) const
{
    return getTransformImpl(F[0]);
}


void BeamJoint::getJacobian(size_t row, size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, Jacobian& jacobian) const
{
    const Vector3D<> axis = joint.R().getCol(2);
    const Vector3D<> p = cross(axis, tcp.P() - joint.P());

    jacobian.addPosition(p, row, col);
    jacobian.addRotation(axis,row, col);
}

void BeamJoint::doMultiplyTransform(const Transform3D<>& parent,
                                        const State& state,
                                        Transform3D<>& result) const
{
}

Transform3D<> BeamJoint::doGetTransform(const State& state) const
{
    return getTransformImpl(getData(state)[0]);
}

Transform3D<> BeamJoint::getTransformImpl(double F) const
{
    // Deflection/slope at z = L with F acting at z = l
    const double y = deflection(F, _L);
    const double dy = slope(F, _L);
    const double a = atan(dy);
    
    Vector3D<> P(0.0, y, _L);
    Rotation3D<> R = RPY<>(a, 0.0, 0.0).toRotation3D();
    
    return _transform * Transform3D<>(P, R);
}