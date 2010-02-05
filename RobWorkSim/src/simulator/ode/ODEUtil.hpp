/*
 * ODEUtil.hpp
 *
 *  Created on: 09-09-2008
 *      Author: jimali
 */

#ifndef ODEUTIL_HPP_
#define ODEUTIL_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <ode/ode.h>

class ODEUtil {

public:
	/**
	 * @brief set the transform of a ode body
	 * @param bodyId [in] the bodyId of the ode body
	 * @param t3d [in] the rw transform
	 */
    static void setODEBodyT3D(dBodyID bodyId, const rw::math::Transform3D<>& t3d);

    static void setODEGeomT3D(dGeomID geoId, const rw::math::Transform3D<>& t3d);

    static rw::math::Transform3D<> getODEBodyT3D(dBodyID bodyId);

    static void toODEVector(const rw::math::Vector3D<>& v, dVector3 dst){
        dst[0] = v(0);
        dst[1] = v(1);
        dst[2] = v(2);
    }

    static rw::math::Vector3D<> toVector3D(const dReal *v){
         return rw::math::Vector3D<>(v[0],v[1],v[2]);
    }

    static void toODERotation(const rw::math::Rotation3D<>& rwR, dMatrix3 R);

    static void toODETransform(const rw::math::Transform3D<>& t3d, dVector3 p, dQuaternion q){
    	p[0] = t3d.P()[0];
    	p[1] = t3d.P()[1];
    	p[2] = t3d.P()[2];
        rw::math::Quaternion<> rwQuat( t3d.R() );
        q[0] = rwQuat.getQw();
        q[1] = rwQuat.getQx();
        q[2] = rwQuat.getQy();
        q[3] = rwQuat.getQz();
    }



};

#endif /* ODEUTIL_HPP_ */
