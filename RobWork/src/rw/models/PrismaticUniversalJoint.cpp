/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "PrismaticUniversalJoint.hpp"

using rw::kinematics::State;
using namespace rw::math;
using namespace rw::models;

PrismaticUniversalJoint::PrismaticUniversalJoint(const std::string& name, const Transform3D<>& transform):
	Joint(name, 3),
	_T(transform)
{
}

PrismaticUniversalJoint::~PrismaticUniversalJoint() {
}

void PrismaticUniversalJoint::doMultiplyTransform(const Transform3D<>& parent, const State& state, Transform3D<>& result) const {
	result = parent*doGetTransform(state);
}

Transform3D<> PrismaticUniversalJoint::doGetTransform(const State& state) const {
    return _T*getJointTransform(state);
}

void PrismaticUniversalJoint::getJacobian(std::size_t row, std::size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
    const double q0 = getData(state)[0];
    const double q1 = getData(state)[1];
    const double q2 = getData(state)[2];
	Vector3D<> axisx = joint.R().getCol(0);
	Vector3D<> axisy = joint.R().getCol(1);
	Vector3D<> axisz = joint.R().getCol(2);
	if (!_mapping.isNull()) {
		axisx *= _mapping->df(q0);
		axisy *= _mapping->df(q1);
	}

	const Rotation3D<> RyInv = EAA<>(axisy,-q1).toRotation3D();

	const Vector3D<> delta = tcp.P() - joint.P();
	const Vector3D<> p1 = cross(RyInv*axisx, delta + q2*axisz);
	const Vector3D<> p2 = cross(axisy, delta + q2*axisz);

	jacobian.addPosition(p1, row, col);
	jacobian.addPosition(p2, row, col+1);
	jacobian.addPosition(axisz, row, col+2);
    jacobian.addRotation(RyInv*axisx,row, col);
    jacobian.addRotation(axisy,row, col+1);
}

Transform3D<> PrismaticUniversalJoint::getFixedTransform() const {
	return _T;
}

void PrismaticUniversalJoint::setFixedTransform(const Transform3D<>& t3d) {
	_T = t3d;
}

Transform3D<> PrismaticUniversalJoint::getJointTransform(const State& state) const {
    double q0 = getData(state)[0];
    double q1 = getData(state)[1];
    double z = getData(state)[2];
    if (!_mapping.isNull()) {
    	q0 = _mapping->f(q0);
    	q1 = _mapping->f(q1);
    }
    const double cq0 = cos(q0);
    const double sq0 = sin(q0);
    const double cq1 = cos(q1);
    const double sq1 = sin(q1);

    Transform3D<> result;
    result(0,0) =  cq1;
    result(0,1) = 0;
    result(0,2) = sq1;
    result(0,3) = result(0,2)*z;

    result(1,0) = sq0*sq1;
    result(1,1) = cq0;
    result(1,2) = -sq0*cq1;
    result(1,3) = result(1,2)*z;

    result(2,0) = -cq0*sq1;
    result(2,1) = sq0;
    result(2,2) = cq0*cq1;
    result(2,3) = result(2,2)*z;

    return result;
}

void PrismaticUniversalJoint::setJointMapping(Function1Diff<>::Ptr function) {
	_mapping = function;
}

void PrismaticUniversalJoint::removeJointMapping() {
	_mapping = NULL;
}
