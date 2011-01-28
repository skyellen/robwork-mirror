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

#include "ODEUtil.hpp"

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>


using namespace rw::math;
using namespace rwsim::simulator;

void ODEUtil::toODERotation(const rw::math::Rotation3D<>& rwR, dMatrix3 R){
    R[0] = rwR(0,0);
    R[1] = rwR(0,1);
    R[2] = rwR(0,2);
    R[4] = rwR(1,0);
    R[5] = rwR(1,1);
    R[6] = rwR(1,2);
    R[8] = rwR(2,0);
    R[9] = rwR(2,1);
    R[10] = rwR(2,2);
}

void ODEUtil::setODEBodyT3D(dBodyID bodyId, const rw::math::Transform3D<>& t3d){
    dBodySetPosition(bodyId, t3d.P()[0], t3d.P()[1], t3d.P()[2] );
    rw::math::Quaternion<> rwQuat( t3d.R() );
    dMatrix3 R;
    toODERotation(t3d.R(),R);
    dBodySetRotation(bodyId, R);
}

void ODEUtil::setODEGeomT3D(dGeomID geoId, const rw::math::Transform3D<>& t3d){
    dGeomSetPosition(geoId, t3d.P()[0], t3d.P()[1], t3d.P()[2] );
    dMatrix3 R;
    toODERotation(t3d.R(),R);
    dGeomSetRotation(geoId, R);
}

rw::math::Transform3D<> ODEUtil::getODEBodyT3D(dBodyID bodyId){
    const dReal *v = dBodyGetPosition(bodyId);
    const dReal *q = dBodyGetQuaternion(bodyId);

    Vector3D<> pos(v[0],v[1],v[2]);
    Quaternion<> quat(q[1],q[2],q[3],q[0]);
    return Transform3D<>(pos,quat);
}

