/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BtUtil.hpp"

#include <rw/math/Quaternion.hpp>

using namespace rw::math;
using namespace rwsimlibs::bullet;

BtUtil::BtUtil() {
}

BtUtil::~BtUtil() {
}

btVector3 BtUtil::makeBtVector(const Vector3D<>& v3d) {
	return btVector3(v3d(0),v3d(1),v3d(2));
}

Vector3D<> BtUtil::toVector3D(const btVector3& v) {
    return Vector3D<>(v[0],v[1],v[2]);
}

btTransform BtUtil::makeBtTransform(const Transform3D<> &t3d) {
	btTransform btt3d;
	const Quaternion<> quat(t3d.R());

	const btVector3 btPos(t3d.P()[0],t3d.P()[1],t3d.P()[2]);
	const btQuaternion btRot(quat.getQx(),quat.getQy(),quat.getQz(),quat.getQw());

	btt3d.setOrigin(btPos);
    btt3d.setRotation(btRot);
    return btt3d;
}

Transform3D<> BtUtil::toTransform3D(const btVector3& v, const btQuaternion &q) {
	const Vector3D<> pos(v[0],v[1],v[2]);
	const Quaternion<> quat(q[0],q[1],q[2],q[3]);
	return Transform3D<>(pos,quat);
}
