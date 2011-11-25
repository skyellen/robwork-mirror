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


#include "MovableFrame.hpp"

#include <rw/math/RPY.hpp>
#include <rw/math/Quaternion.hpp>
#include "Kinematics.hpp"
using namespace rw::kinematics;
using namespace rw::math;

MovableFrame::MovableFrame(const std::string& name)
    :
    Frame(7, name)
{}

void MovableFrame::doMultiplyTransform(const Transform3D<>& parent,
                                       const State& state,
                                       Transform3D<>& result) const
{
    Transform3D<>::multiply(parent, getTransform(state), result);
}


Transform3D<> MovableFrame::doGetTransform(const State& state) const {
    const double* q = getData(state);
    Quaternion<> quat(q[0], q[1], q[2], q[3]);
    const Vector3D<> pos(q[4], q[5], q[6]);
    quat.normalize();

    return Transform3D<>(pos, quat.toRotation3D());
}



void MovableFrame::setTransform(const Transform3D<>& transform, State& state)
{
    const Quaternion<> quat(transform.R());
    const Vector3D<> pos(transform.P());

    double q[7];
    q[0] = quat(0);
    q[1] = quat(1);
    q[2] = quat(2);
    q[3] = quat(3);
    q[4] = pos(0);
    q[5] = pos(1);
    q[6] = pos(2);
    setData(state, q);
}

void MovableFrame::moveTo(const Transform3D<>& wTtarget, State& state){
    // first calculate transform from refframe to parent frame
    Transform3D<> wTparent = Kinematics::worldTframe(getParent(), state);
    Transform3D<> parentTmframe = inverse(wTparent) * wTtarget;
    setTransform( parentTmframe, state );
}

void MovableFrame::moveTo(const Transform3D<>& refTtarget, Frame* refframe, State& state){
    // first calculate transform from refframe to parent frame
    Transform3D<> parentTref = Kinematics::frameTframe(getParent(), refframe, state);
    Transform3D<> parentTmframe = parentTref * refTtarget;
    setTransform( parentTmframe, state );
}
