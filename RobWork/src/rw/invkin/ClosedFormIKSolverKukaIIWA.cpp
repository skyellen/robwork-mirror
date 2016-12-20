/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ClosedFormIKSolverKukaIIWA.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Random.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/Joint.hpp>

using rw::invkin::ClosedFormIKSolverKukaIIWA;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;

ClosedFormIKSolverKukaIIWA::ClosedFormIKSolverKukaIIWA(const rw::common::Ptr<const rw::models::SerialDevice> device, const State& state):
	_device(device),
	_checkJointLimits(true),
	_qLow(device->getBounds().first),
	_qHigh(device->getBounds().second)
{
	_frames.push_back(_device->getBase());
	BOOST_FOREACH(const Joint* const joint, _device->getJoints()) {
		_frames.push_back(joint);
	}
	_frames.push_back(_device->getEnd());

	// The position of joint 2 is found (this is constant).
	const Transform3D<> trJ0J2 = Kinematics::frameTframe(_frames[0],_frames[2],state);
	_baseP2 = trJ0J2.P();

	// The length from the 3rd joint to the 4th joint is:
	const Transform3D<> trJ3J4 = Kinematics::frameTframe(_frames[3],_frames[4],state);
	_lJ3J4 = trJ3J4.P().norm2();
	// The length from the 2nd joint to the 4th joint
	// - notice that this must equal the length from joint 4 to joint 6
	// - the calculation of the circle intersection between two spheres in solve() relies on these two lengths being equal
	const Transform3D<> trJ2J4 = Kinematics::frameTframe(_frames[2],_frames[4],state);
	_lJ2J4 = trJ2J4.P().norm2();
	// The length from the last joint to the TCP frame is:
	const Transform3D<> trTcp = Kinematics::frameTframe(_frames[7],_frames[8],state);
	_lTcp = trTcp.P().norm2();

	_fkRange2_0 = FKRange(_frames[2], _frames[0], state);
	_fkRange3_0 = FKRange(_frames[3], _frames[0], state);
	_fkRange4_0 = FKRange(_frames[4], _frames[0], state);
	_fkRange5_0 = FKRange(_frames[5], _frames[0], state);
	_fkRange6_0 = FKRange(_frames[6], _frames[0], state);
}

ClosedFormIKSolverKukaIIWA::~ClosedFormIKSolverKukaIIWA() {
}

std::vector<Q> ClosedFormIKSolverKukaIIWA::solve(const Transform3D<>& baseTend, const State& stateArg) const {
	State state = stateArg;
	std::vector<Q> results;

	// First find the position of the last joint.
	const Vector3D<> tcpZ = baseTend.R().getCol(2);
	const Vector3D<> baseP6 = baseTend.P()-tcpZ*_lTcp; // equal to baseP7

	// Find circle where joint 4 must be (in base reference system)
	const Vector3D<> circleCenter4 = (baseP6+_baseP2)/2;
	const Vector3D<> circleNormal4 = normalize(baseP6-_baseP2);
	const double dist = (baseP6-_baseP2).norm2();
	if (dist > _lJ2J4*2) { // no possible solutions
		return results;
	}
	const double radius4 = std::sqrt(4*dist*dist*_lJ2J4*_lJ2J4-dist*dist*dist*dist)/(2*dist);

	const Vector3D<> targetP4 = circleCenter4 + randomPerpendicularVector(circleNormal4)*radius4;

	const std::pair<double,double> baseAngles = findBaseAngles(Vector2D<>(targetP4[0],targetP4[1]), state);

	// Now add up to 4 solutions for each of the possible base angles
	if (_checkJointLimits) {
		if (baseAngles.first >= _qLow[0] && baseAngles.first <= _qHigh[0])
			addBaseAngleSolutions(baseTend.R(),baseP6,targetP4,state,baseAngles.first,results);
		if (baseAngles.second >= _qLow[0] && baseAngles.second <= _qHigh[0])
			addBaseAngleSolutions(baseTend.R(),baseP6,targetP4,state,baseAngles.second,results);
	} else {
		addBaseAngleSolutions(baseTend.R(),baseP6,targetP4,state,baseAngles.first,results);
		addBaseAngleSolutions(baseTend.R(),baseP6,targetP4,state,baseAngles.second,results);
	}

	return results;
}

void ClosedFormIKSolverKukaIIWA::setCheckJointLimits(bool check) {
	_checkJointLimits = check;
}

std::pair<double,double> ClosedFormIKSolverKukaIIWA::findBaseAngles(const Vector2D<> &pos, const State& state) const {
	std::pair<double,double> res;
	if (pos[0] > 0 || pos[1] > 0) {
		res.first = std::atan2(pos[1],pos[0]);
	} else {
		const Q qcurrent = _device->getQ(state);
		res.first = qcurrent(0);
	}
	res.second = (res.first > 0) ? res.first-Pi : res.first+Pi;
	return res;
}

void ClosedFormIKSolverKukaIIWA::addBaseAngleSolutions(const Rotation3D<>& baseRend, const Vector3D<>& baseP6, const Vector3D<>& basePtarget4, State& state, double angle, std::vector<Q>& res) const {
	Q tmpQ = Q(7,angle,0,0,0,0,0,0);

	const Rotation2D<> baseR1(-angle);
	const Vector3D<> diff = basePtarget4-_baseP2;
	const Vector2D<> rot = baseR1*Vector2D<>(diff[0],diff[1]);
	tmpQ[1] = std::atan2(rot[0],diff[2]);

	if (_checkJointLimits) { // Abort if outside limits
		if (tmpQ[1] < _qLow[1] || tmpQ[1] > _qHigh[1])
			return;
	}

	_device->setQ(tmpQ, state);

	// Determine the two possible positions of joint 3 for the current base and joint 1 angles.
	const Transform3D<> j2Tbase = _fkRange2_0.get(state);
	const Vector3D<> j2p6 = j2Tbase*baseP6;
	const double theta3A = std::atan2(j2p6[2],j2p6[0]);
	const double theta3B = (theta3A > 0) ? theta3A-Pi : theta3A+Pi;

	// Add the two potential solutions given the first three joint angles
	if (_checkJointLimits) {
		if (theta3A >= _qLow[2] && theta3A <= _qHigh[2])
			addOuterSolutions(baseRend, baseP6, state, angle, tmpQ[1], theta3A, res);
		if (theta3B >= _qLow[2] && theta3B <= _qHigh[2])
			addOuterSolutions(baseRend, baseP6, state, angle, tmpQ[1], theta3B, res);
	} else {
		addOuterSolutions(baseRend, baseP6, state, angle, tmpQ[1], theta3A, res);
		addOuterSolutions(baseRend, baseP6, state, angle, tmpQ[1], theta3B, res);
	}
}

void ClosedFormIKSolverKukaIIWA::addOuterSolutions(const Rotation3D<>& baseRend, const Vector3D<>& baseP6, State& state, double angle1, double angle2, double angle3, std::vector<Q>& res) const {
	Q tmpQ = Q(7,angle1,angle2,angle3,0,0,0,0);
	_device->setQ(tmpQ, state);

	const Transform3D<> j3Tbase = _fkRange3_0.get(state);
	const Vector3D<> j3p6 = j3Tbase*baseP6;
	tmpQ[3] = -std::atan2(j3p6[0],j3p6[2]-_lJ3J4);

	if (_checkJointLimits) { // Abort if outside limits
		if (tmpQ[3] < _qLow[3] || tmpQ[3] > _qHigh[3])
			return;
	}

	_device->setQ(tmpQ, state);

	const Vector3D<> tcpZ = baseRend.getCol(2);
	const Rotation3D<> j4Rbase = _fkRange4_0.get(state).R();
	const Vector3D<> j4z6 = j4Rbase*tcpZ;
	const double theta5A = std::atan2(-j4z6[2],j4z6[0]);
	const double theta5B = (theta5A > 0) ? theta5A-Pi : theta5A+Pi;

	// Find the final joint angles
	if (_checkJointLimits) {
		if (theta5A >= _qLow[4] && theta5A <= _qHigh[4])
			addRotationSolutions(baseRend, state, angle1, angle2, angle3, tmpQ[3], theta5A, res);
		if (theta5B >= _qLow[4] && theta5B <= _qHigh[4])
			addRotationSolutions(baseRend, state, angle1, angle2, angle3, tmpQ[3], theta5B, res);
	} else {
		addRotationSolutions(baseRend, state, angle1, angle2, angle3, tmpQ[3], theta5A, res);
		addRotationSolutions(baseRend, state, angle1, angle2, angle3, tmpQ[3], theta5B, res);
	}
}

void ClosedFormIKSolverKukaIIWA::addRotationSolutions(const Rotation3D<>& baseRend, State& state, double angle1, double angle2, double angle3, double angle4, double angle5, std::vector<Q>& res) const {
	Q tmpQ = Q(7,angle1,angle2,angle3,angle4,angle5,0,0);
	_device->setQ(tmpQ, state);

	const Vector3D<> tcpZ = baseRend.getCol(2);
	const Rotation3D<> j5Rbase = _fkRange5_0.get(state).R();
	const Vector3D<> j5z6 = j5Rbase*tcpZ;
	tmpQ[5] = std::atan2(j5z6[0],j5z6[2]);

	if (_checkJointLimits) { // Abort if outside limits
		if (tmpQ[5] < _qLow[5] || tmpQ[5] > _qHigh[5])
			return;
	}

	_device->setQ(tmpQ, state);

	const Vector3D<> tcpX = baseRend.getCol(0);
	const Rotation3D<> j6Rbase = _fkRange6_0.get(state).R();
	const Vector3D<> j6x6 = j6Rbase*tcpX;
	tmpQ[6] = std::atan2(j6x6[2],j6x6[0]);

	if (_checkJointLimits) { // Abort if outside limits
		if (tmpQ[6] < _qLow[6] || tmpQ[6] > _qHigh[6])
			return;
	}

	res.push_back(tmpQ);
}

Vector3D<> ClosedFormIKSolverKukaIIWA::randomPerpendicularVector(const Vector3D<>& v) {
	Vector3D<> perp;
	if (v[0] < v[1]) {
		if (v[0] < v[2])
			perp = normalize(cross(v,Vector3D<>::x()));
		else
			perp = normalize(cross(v,Vector3D<>::z()));
	} else {
		if (v[1] < v[2])
			perp = normalize(cross(v,Vector3D<>::y()));
		else
			perp = normalize(cross(v,Vector3D<>::z()));
	}
	return EAA<>(v,Random::ran(0,2*Pi)).toRotation3D()*perp;
}
