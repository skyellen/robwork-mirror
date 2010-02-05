/*
 * ODEUtil.cpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#include "ODEUtil.hpp"

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>


using namespace rw::math;

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

