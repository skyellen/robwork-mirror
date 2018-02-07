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

#include "UniversalJoint.hpp"

using rw::kinematics::State;
using namespace rw::math;
using namespace rw::models;

UniversalJoint::UniversalJoint(const std::string& name, const Transform3D<>& transform):
	Joint(name, 2),
	_T(transform)
{
}

UniversalJoint::~UniversalJoint() {
}

void UniversalJoint::doMultiplyTransform(const Transform3D<>& parent, const State& state, Transform3D<>& result) const {
	result = parent*doGetTransform(state);
}

Transform3D<> UniversalJoint::doGetTransform(const State& state) const {
    return Transform3D<>(_T.P(),_T.R()*getJointTransform(state).R());
	double q1 = getData(state)[0];
	double q2 = getData(state)[1];

    if (!_mapping.isNull()) {
    	q1 = _mapping->f(q1);
    	q2 = _mapping->f(q2);
    }

    const double b00 = _T.R()(0, 0);
    const double b01 = _T.R()(0, 1);
    const double b02 = _T.R()(0, 2);
    const double b10 = _T.R()(1, 0);
    const double b11 = _T.R()(1, 1);
    const double b12 = _T.R()(1, 2);
    const double b20 = _T.R()(2, 0);
    const double b21 = _T.R()(2, 1);
    const double b22 = _T.R()(2, 2);
    const double bx = _T.P()(0);
    const double by = _T.P()(1);
    const double bz = _T.P()(2);

    const double cq0 = std::cos(q1);
    const double sq0 = std::sin(q1);
    const double cq1 = std::cos(q2);
    const double sq1 = std::sin(q2);

    Transform3D<> result;
    result(0,0) = b00*cq1 + b01*sq0*sq1 - b02*cq0*sq1;
    result(0,1) = b01*cq0 + b02*sq0;
    result(0,2) = b00*sq1 - b01*sq0*cq1 + b02*cq0*cq1;
    result(0,3) = bx;

    result(1,0) = b10*cq1 + b11*sq0*sq1 - b12*cq0*sq1;
    result(1,1) = b11*cq0 + b12*sq0;
    result(1,2) = b10*sq1 - b11*sq0*cq1 + b12*cq0*cq1;
    result(1,3) = by;

    result(2,0) = b20*cq1 + b21*sq0*sq1 - b22*cq0*sq1;
    result(2,1) = b21*cq0 + b22*sq0;
    result(2,2) = b20*sq1 - b21*sq0*cq1 + b22*cq0*cq1;
    result(2,3) = bz;
    return result;
}

void UniversalJoint::getJacobian(std::size_t row, std::size_t col, const Transform3D<>& joint, const Transform3D<>& tcp, const State& state, Jacobian& jacobian) const {
    const double q0 = getData(state)[0];
    const double q1 = getData(state)[1];
	Vector3D<> axisx = joint.R().getCol(0);
	Vector3D<> axisy = joint.R().getCol(1);
	if (!_mapping.isNull()) {
		axisx *= _mapping->df(q0);
		axisy *= _mapping->df(q1);
	}

	const Rotation3D<> RyInv = EAA<>(axisy,-q1).toRotation3D();

	const Vector3D<> p1 = cross(RyInv*axisx, tcp.P() - joint.P());
	const Vector3D<> p2 = cross(axisy, tcp.P() - joint.P());

	jacobian.addPosition(p1, row, col);
	jacobian.addPosition(p2, row, col+1);
    jacobian.addRotation(RyInv*axisx,row, col);
    jacobian.addRotation(axisy,row, col+1);
}

Transform3D<> UniversalJoint::getFixedTransform() const {
	return _T;
}

void UniversalJoint::setFixedTransform(const Transform3D<>& t3d) {
	_T = t3d;
}

Transform3D<> UniversalJoint::getJointTransform(const State& state) const {
    double q0 = getData(state)[0];
    double q1 = getData(state)[1];
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
    result(0,3) = 0;

    result(1,0) = sq0*sq1;
    result(1,1) = cq0;
    result(1,2) = -sq0*cq1;
    result(1,3) = 0;

    result(2,0) = -cq0*sq1;
    result(2,1) = sq0;
    result(2,2) = cq0*cq1;
    result(2,3) = 0;

    return result;
}

void UniversalJoint::setJointMapping(Function1Diff<>::Ptr function) {
	_mapping = function;
}

void UniversalJoint::removeJointMapping() {
	_mapping = NULL;
}
