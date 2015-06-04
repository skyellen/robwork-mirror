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

#include "ClosedFormIKSolverUR.hpp"
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Models.hpp>
#include <rw/models/SerialDevice.hpp>

#include <iomanip>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;

ClosedFormIKSolverUR::ClosedFormIKSolverUR(const rw::common::Ptr<const SerialDevice> device, const State& state):
	_device(device),
	_checkJointLimits(true)
{
	_frames.push_back(_device->getBase());
	BOOST_FOREACH(const Joint* const joint, _device->getJoints()) {
		_frames.push_back(joint);
	}
	_frames.push_back(_device->getEnd());

	// The dimensions of the robot is now found.
	// Note that these are independent of the configuration of the robot.

	// The position of the last joint always lies in a plane perpendicular to the z axis of the second joint.
	// This plane can be determined from the angle of the base joint only.
	// The distance from the base to the plane is determined as:
	const Transform3D<> trans = Kinematics::frameTframe(_frames[0],_frames[6],state);
	const Vector3D<> direction = Kinematics::frameTframe(_frames[0],_frames[2],state).R()*Vector3D<>::z();
	_baseRadius = dot(trans.P(),direction);
	_baseRadiusSqr = Math::sqr(_baseRadius);

	// The position of the last joint (Joint 5) lies on a circle around the second last joint (Joint 4).
	// The radius of this circle is:
	const Transform3D<> endTrans = Kinematics::frameTframe(_frames[4],_frames[6],state);
	_endCircleRadius = std::fabs(endTrans.P()[1]);



	// The length from Joint0 to Joint1 to Joint2 is found:
	const Transform3D<> trJ0J1 = Kinematics::frameTframe(_frames[1],_frames[2],state);
	_lJ0J1 = trJ0J1.P().norm2();
	const Transform3D<> tr1 = Kinematics::frameTframe(_frames[2],_frames[3],state);	
	_l1 = Vector2D<>(tr1.P()(0), tr1.P()(1)).norm2();
	const Transform3D<> tr2 = Kinematics::frameTframe(_frames[3],_frames[4],state);
	_l2 = Vector2D<>(tr2.P()(0), tr2.P()(1)).norm2();

	// The length from the last joint to the TCP frame is:
	const Transform3D<> trTcp = Kinematics::frameTframe(_frames[6],_frames[7],state);
	_lTcp = trTcp.P().norm2();

	State tmpState = state;
	_device->setQ(Q::zero(6), tmpState);

	//Find the transformation needed to align the elbow joint
	const Transform3D<> shoulder2wrist = Kinematics::frameTframe(_frames[2], _frames[4], tmpState);
	Vector3D<> wristPos(shoulder2wrist.P()(0), shoulder2wrist.P()(1), 0);
	
	_rotAlignElbowJoint = Rotation3D<>(EAA<>(normalize(wristPos), Vector3D<>(1, 0 ,0)).toRotation3D());


	Vector3D<> wristOffset = Kinematics::frameTframe(_frames[4], _frames[6], tmpState).P();		
	wristOffset(2) = 0; //Only interested in the part not in the direction of the rotational axis
	_rotAlignDH4 = EAA<>(normalize(wristOffset), Vector3D<>(1, 0 ,0)).toRotation3D();

	_zaxisJoint6In5 = Kinematics::frameTframe(_frames[5], _frames[6], tmpState).R().getCol(2);

	_fkRange0_2 = FKRange(_frames[0], _frames[2], tmpState);
	_fkRange2_0 = FKRange(_frames[2], _frames[0], tmpState);
	_fkRange3_0 = FKRange(_frames[3], _frames[0], tmpState);
	_fkRange4_3 = FKRange(_frames[4], _frames[3], tmpState);
	_fkRange5_4 = FKRange(_frames[5], _frames[4], tmpState);
	_fkRange6_5 = FKRange(_frames[6], _frames[5], tmpState);



	_qMin = _device->getBounds().first;
	_qMax = _device->getBounds().second;

}

ClosedFormIKSolverUR::~ClosedFormIKSolverUR() {
}

std::vector<Q> ClosedFormIKSolverUR::solve(const Transform3D<>& baseTend, const State& stateArg) const {
	State state = stateArg;
	std::vector<Q> res;
	

	// First find the position of the last joint, and use the xy coordinate to determine the
	// two possible base angles.
	const Vector3D<> tcpZ = baseTend.R().getCol(2);//*Vector3D<>::z();
	const Vector3D<> baseTdh5 = baseTend.P()-tcpZ*_lTcp;

	const std::pair<double,double> baseAngles = findBaseAngle(Vector2D<>(baseTdh5[0],baseTdh5[1]), state);

	// Now add up to 4 solutions for the first base angle
	addBaseAngleSolutions(baseTend,baseTdh5,state,baseAngles.first,res);

	// ... and up to 4 solutions for the second base angle
	addBaseAngleSolutions(baseTend,baseTdh5,state,baseAngles.second,res);

	return res;
}

Q ClosedFormIKSolverUR::adjustJoints(const Q& q) const {
	Q qRes;
	qRes = q;
	for (std::size_t i = 0; i < _device->getDOF(); i++) {
		while(qRes[i] < _qMin[i])	qRes[i] += 2.*Pi;
		while(qRes[i] > _qMax[i])	qRes[i] -= 2.*Pi;
	}
	return qRes;
}

void ClosedFormIKSolverUR::addBaseAngleSolutions(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, State& state, double angle, std::vector<Q>& res) const {
	Q tmpQ = Q(6,angle,0,0,0,0,0);
	_device->setQ(tmpQ, state);
	const Vector3D<> tcpZ = baseTend.R().getCol(2);

	// Determine the two possible positions of joint 4 for the current base angle.
	const std::pair<Vector3D<>,Vector3D<> > j4Pos = getJoint4Positions(baseTdh5,tcpZ, state);

	// For each possible position of joint 4, find the elbow up and elbow down solutions
	const std::pair<std::pair<double,double>,std::pair<double,double> > elbowPos1 = getElbowJoints(j4Pos.first, state);

	const std::pair<std::pair<double,double>,std::pair<double,double> > elbowPos2 = getElbowJoints(j4Pos.second,state);

	// Add the four potential solutions for the current base angle
	addElbowSolutions(baseTend, baseTdh5, state, angle, elbowPos1.first, res);
	addElbowSolutions(baseTend, baseTdh5, state, angle, elbowPos1.second, res);
	addElbowSolutions(baseTend, baseTdh5, state, angle, elbowPos2.first, res);
	addElbowSolutions(baseTend, baseTdh5, state, angle, elbowPos2.second, res);
	
}

void ClosedFormIKSolverUR::addElbowSolutions(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, State& state, double baseAngle, std::pair<double,double> elbow, std::vector<Q>& res) const {
	if (!Math::isNaN(elbow.first) && !Math::isNaN(elbow.second)) {
		Q tmpQ = Q(6,baseAngle,elbow.first,elbow.second,0,0,0);
		_device->setQ(tmpQ, state);
		// The three first joints has been determined - find the last three joints.
		Q q = getOrientationJoints(baseTend,baseTdh5, state);
		
		const Q adjustedQ = adjustJoints(q);
		_device->setQ(adjustedQ, state);

		if (!_checkJointLimits) {
			res.push_back(adjustedQ);
		}
		else {
			if (Models::inBounds(adjustedQ,*_device))
				res.push_back(adjustedQ);
		}
	}
}

std::pair<Vector3D<>,Vector3D<> > ClosedFormIKSolverUR::getJoint4Positions(const Vector3D<>& baseTdh5, const Vector3D<>& tcpZ, const State& state) const {
	// Determine the two possible positions of joint 4.
	const Vector3D<> xDir = normalize(getPerpendicularVector(tcpZ));
	const Vector3D<> yDir = normalize(cross(tcpZ,xDir));
	const Vector3D<> dir = _fkRange0_2.get(state).R().getCol(2);
	return findCirclePlaneIntersection(baseTdh5, _endCircleRadius, xDir, yDir, dir);
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormIKSolverUR::getElbowJoints(const Vector3D<>& intersection, State& state) const {
	// Only use the base joint angle, and set remaining joints to zero.
	Q tmpQ = _device->getQ(state);
	tmpQ[1] = 0;
	tmpQ[2] = 0;
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ, state);

	// Determine the 2 possible configurations for joint 1 and joint 2, such that joint 4 reaches the given point
	const Transform3D<> dh1Tbase = _fkRange2_0.get(state);// Kinematics::frameTframe(_frames[2],_frames[0],tmpState);
	Vector3D<> target = dh1Tbase*intersection;
	target = _rotAlignElbowJoint*target;
	std::pair<std::pair<double,double>,std::pair<double,double> > a1a2 = findTwoBarAngles(Vector2D<>(target[0],target[1]),_l1,_l2);

	return a1a2;
}

Q ClosedFormIKSolverUR::getOrientationJoints(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, State& state) const {
	// Set last three joints to zero.
	//State tmpState = state;
	Q tmpQ = _device->getQ(state);
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ, state);
	std::vector<Q> res;


	// Joint 3
	Transform3D<> j3Tj0 = _fkRange3_0.get(state);
	Transform3D<> j4Tj3 = _fkRange4_3.get(state);
	Transform3D<> j4Tj0 = j4Tj3*j3Tj0;
	Vector3D<> dh4Tcenter = j4Tj0*baseTdh5;
	dh4Tcenter = _rotAlignDH4 * dh4Tcenter;
	tmpQ(3) = std::atan2(dh4Tcenter[1], dh4Tcenter[0]);
	_device->setQ(tmpQ, state);


	// Joint 4	
	j4Tj3 = _fkRange4_3.get(state);
	Rotation3D<> j5Rj4 = _fkRange5_4.get(state).R();
	Rotation3D<> j4Rj0 = j4Tj3.R()*j3Tj0.R();
	Rotation3D<> j5Rj0 = j5Rj4*j4Rj0;
	Vector3D<> zdes = (j5Rj0*baseTend.R().getCol(2));
	EAA<> eaa2(_zaxisJoint6In5 , zdes);
	tmpQ(4) = eaa2(2);
	_device->setQ(tmpQ, state);

	//Joint 5

	j5Rj4 = _fkRange5_4.get(state).R();
	Rotation3D<> j6Rj5 = _fkRange6_5.get(state).R();
	j5Rj0 = j5Rj4 * j4Rj0;
	Rotation3D<> j6Rj0 = j6Rj5*j5Rj0;

	Vector3D<> xaxis = j6Rj0*baseTend.R().getCol(0);
	tmpQ(5) = std::atan2(xaxis(1), xaxis(0));


	return tmpQ;
	
}

void ClosedFormIKSolverUR::setCheckJointLimits(bool check) {
	_checkJointLimits = check;
}

std::pair<double,double> ClosedFormIKSolverUR::findBaseAngle(const Vector2D<> &pos, const State& state) const {
	std::pair<double,double> res;
	const double arg = Math::sqr(pos.norm2())-_baseRadiusSqr;
	if ( arg > 0) {
		const double D = std::sqrt(arg);
	
		res.first	= std::atan2(-D*pos[0]+_baseRadius*pos[1],+D*pos[1]+_baseRadius*pos[0]);
		res.second	= std::atan2(+D*pos[0]+_baseRadius*pos[1],-D*pos[1]+_baseRadius*pos[0]);
		res.first  += Pi/2;
		res.second  += Pi/2;
		if (res.first > Pi)
			res.first -= 2*Pi;
		if (res.second > Pi)
			res.second -= 2*Pi;
	} else {
		const Q qcurrent = _device->getQ(state);
		res.first = qcurrent(0);
		if (qcurrent(0) > 0)
			res.second = qcurrent(0)-Pi;
		else
			res.second = qcurrent(0)+Pi;
	}
	return res;
}

std::pair<Vector3D<>,Vector3D<> > ClosedFormIKSolverUR::findCirclePlaneIntersection(const Vector3D<>& circleCenter, double radius, const Vector3D<>& circleDir1, const Vector3D<>& circleDir2, const Vector3D<>& planeNormal) {
	std::pair<Vector3D<>,Vector3D<> > res;
	const double t = atan2(-dot(circleDir1,planeNormal),dot(circleDir2,planeNormal));
	res.first = circleCenter + radius*(circleDir1*cos(t)+circleDir2*sin(t));
	res.second = circleCenter + radius*(circleDir1*cos(t+Pi)+circleDir2*sin(t+Pi));
	return res;
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormIKSolverUR::findTwoBarAngles(const Vector2D<>& pos, double L1, double L2) {
	std::pair<std::pair<double,double>,std::pair<double,double> > res;
	std::pair<double,double> res1, res2;
	res1.second = 2*std::atan(std::sqrt((Math::sqr(L1+L2)-Math::sqr(pos[0])-Math::sqr(pos[1]))/(Math::sqr(pos[0])+Math::sqr(pos[1])-Math::sqr(L1-L2))));
	res2.second = -res1.second;
	res1.first = std::atan2(pos[1]*(L1+L2*cos(res1.second))-pos[0]*(L2*sin(res1.second)),pos[0]*(L1+L2*cos(res1.second))+pos[1]*(L2*sin(res1.second)));
	res2.first = std::atan2(pos[1]*(L1+L2*cos(res2.second))-pos[0]*(L2*sin(res2.second)),pos[0]*(L1+L2*cos(res2.second))+pos[1]*(L2*sin(res2.second)));
	res.first = res1;
	res.second = res2;
	return res;
}

Vector3D<> ClosedFormIKSolverUR::getPerpendicularVector(const Vector3D<> &vec) {
	if (abs(vec[0]) < abs(vec[1]) && abs(vec[0]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(1,0,0));
	} else
	if (abs(vec[1]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(0,1,0));
	} else {
		return cross(vec,Vector3D<>(0,0,1));
	}
}
