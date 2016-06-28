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

#include "Kinematics.hpp"
using namespace rw::kinematics;
using namespace rw::math;

MovableFrame::MovableFrame(const std::string& name)
    :
    Frame(12, name)
{}

void MovableFrame::doMultiplyTransform(const Transform3D<>& parent,
                                       const State& state,
                                       Transform3D<>& result) const
{
    Transform3D<>::multiply(parent, getTransform(state), result);
}


Transform3D<> MovableFrame::doGetTransform(const State& state) const {
    const double* q = getData(state);
    //Quaternion<> quat(q[0], q[1], q[2], q[3]);

    const Vector3D<> pos(q[0], q[1], q[2]);
    const Rotation3D<> rot(q[3], q[4], q[5],
                           q[6], q[7], q[8],
                           q[9], q[10], q[11]);
    //quat.normalize();

    return Transform3D<>(pos, rot);
}



void MovableFrame::setTransform(const Transform3D<>& transform, State& state)
{
    //const Quaternion<> quat(transform.R());
    const Vector3D<> &pos = transform.P();
    const Rotation3D<> &rot = transform.R();

    double q[12];

    //q[0] = quat(0);
    //q[1] = quat(1);
    //q[2] = quat(2);
    //q[3] = quat(3);
    q[0] = pos(0);
    q[1] = pos(1);
    q[2] = pos(2);

    q[3] = rot(0,0);
    q[4] = rot(0,1);
    q[5] = rot(0,2);
    q[6] = rot(1,0);
    q[7] = rot(1,1);
    q[8] = rot(1,2);
    q[9] = rot(2,0);
    q[10] = rot(2,1);
    q[11] = rot(2,2);

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
