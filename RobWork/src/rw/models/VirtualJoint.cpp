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


#include "VirtualJoint.hpp"

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::math;

VirtualJoint::VirtualJoint(const std::string& name,
                           const Transform3D<>& transform,
                           size_t dof) :
    Joint(name, dof),
    _transform(transform)
{
}


void VirtualJoint::doMultiplyTransform(const Transform3D<>& parent,
                                      const State& state,
                                      Transform3D<>& result) const
{
    Transform3D<>::multiply(parent, _transform, result);
}


Transform3D<> VirtualJoint::doGetTransform(const State& state) const {
    return _transform;
}

void VirtualJoint::setFixedTransform( const rw::math::Transform3D<>& t3d){
    _transform = t3d;
}

rw::math::Transform3D<> VirtualJoint::getJointTransform(const rw::kinematics::State& state) const{
    return _transform;
}

