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

#include "SphericalJoint.hpp"

using rw::kinematics::State;
using namespace rw::math;
using namespace rw::models;

SphericalJoint::SphericalJoint(const std::string& name, const Transform3D<>& transform):
	Joint(name, 3),
	_T(transform)
{
}

SphericalJoint::~SphericalJoint() {
}

void SphericalJoint::doMultiplyTransform(const Transform3D<>& parent, const State& state, Transform3D<>& result) const {
	result = parent*doGetTransform(state);
}

Transform3D<> SphericalJoint::doGetTransform(const State& state) const {
    return Transform3D<>(_T.P(),_T.R()*getJointTransform(state).R());
}

void SphericalJoint::getJacobian(std::size_t row, std::size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
    const double q0 = getData(state)[0];
    const double q1 = getData(state)[1];
    const double q2 = getData(state)[2];
	Vector3D<> axisx = joint.R().getCol(0);
	Vector3D<> axisy = joint.R().getCol(1);
	Vector3D<> axisz = joint.R().getCol(2);
	if (!_mapping.isNull()) {
		axisx *= _mapping->df(q0);
		axisy *= _mapping->df(q1);
		axisz *= _mapping->df(q2);
	}

	const Rotation3D<> RzInv = EAA<>(axisz,-q2).toRotation3D();
	const Rotation3D<> RyInv = EAA<>(RzInv*axisy,-q1).toRotation3D();

	const Vector3D<> p1 = cross(RyInv*RzInv*axisx, tcp.P() - joint.P());
	const Vector3D<> p2 = cross(RzInv*axisy, tcp.P() - joint.P());
	const Vector3D<> p3 = cross(axisz, tcp.P() - joint.P());

	jacobian.addPosition(p1, row, col);
	jacobian.addPosition(p2, row, col+1);
	jacobian.addPosition(p3, row, col+2);
    jacobian.addRotation(RyInv*RzInv*axisx,row, col);
    jacobian.addRotation(RzInv*axisy,row, col+1);
    jacobian.addRotation(axisz,row, col+2);
}

Transform3D<> SphericalJoint::getFixedTransform() const {
	return _T;
}

void SphericalJoint::setFixedTransform(const Transform3D<>& t3d) {
	_T = t3d;
}

Transform3D<> SphericalJoint::getJointTransform(const State& state) const {
    double q0 = getData(state)[0];
    double q1 = getData(state)[1];
    double q2 = getData(state)[2];
    if (!_mapping.isNull()) {
    	q0 = _mapping->f(q0);
    	q1 = _mapping->f(q1);
    	q2 = _mapping->f(q2);
    }
    const double cq0 = std::cos(q0);
    const double sq0 = std::sin(q0);
    const double cq1 = std::cos(q1);
    const double sq1 = std::sin(q1);
    const double cq2 = std::cos(q2);
    const double sq2 = std::sin(q2);

    Transform3D<> result;
    result(0,0) = cq1*cq2;
    result(0,1) = -cq1*sq2;
    result(0,2) = sq1;
    result(0,3) = 0;

    result(1,0) = sq0*sq1*cq2+cq0*sq2;
    result(1,1) = -sq0*sq1*sq2+cq0*cq2;
    result(1,2) = -sq0*cq1;
    result(1,3) = 0;

    result(2,0) = -cq0*sq1*cq2+sq0*sq2;
    result(2,1) = cq0*sq1*sq2+sq0*cq2;
    result(2,2) = cq0*cq1;
    result(2,3) = 0;

    return result;
}

void SphericalJoint::setJointMapping(Function1Diff<>::Ptr function) {
	_mapping = function;
}

void SphericalJoint::removeJointMapping() {
	_mapping = NULL;
}
