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

#ifndef RWSIM_SIMULATOR_ODEUTIL_HPP_
#define RWSIM_SIMULATOR_ODEUTIL_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <ode/ode.h>

namespace rwsim {
namespace simulator {

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
}
}
#endif /* ODEUTIL_HPP_ */
