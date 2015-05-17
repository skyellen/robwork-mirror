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

#include "ClosedFormURSolver.hpp"
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Models.hpp>
#include <rw/models/SerialDevice.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;

ClosedFormURSolver::ClosedFormURSolver(const rw::common::Ptr<const SerialDevice> device, const State& state):
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

	// The position of the last joint (Joint 5) lies on a circle around the second last joint (Joint 4).
	// The radius of this circle is:
	const Transform3D<> endTrans = Kinematics::frameTframe(_frames[5],_frames[6],state);
	_endCircleRadius = endTrans.P()[2];

	// The length from Joint0 to Joint1 to Joint2 is found:
	const Transform3D<> trJ0J1 = Kinematics::frameTframe(_frames[1],_frames[2],state);
	_lJ0J1 = trJ0J1.P().norm2();
	const Transform3D<> tr1 = Kinematics::frameTframe(_frames[2],_frames[3],state);
	_l1 = tr1.P().norm2();
	const Transform3D<> tr2 = Kinematics::frameTframe(_frames[3],_frames[4],state);
	_l2 = tr2.P().norm2();

	// The length from the last joint to the TCP frame is:
	const Transform3D<> trTcp = Kinematics::frameTframe(_frames[6],_frames[7],state);
	_lTcp = trTcp.P().norm2();

}

ClosedFormURSolver::~ClosedFormURSolver() {
}

std::vector<Q> ClosedFormURSolver::solve(const Transform3D<>& baseTend, const State& state) const {
	std::vector<Q> res;

	// First find the position of the last joint, and use the xy coordinate to determine the
	// two possible base angles.
	const Vector3D<> tcpZ = baseTend.R()*Vector3D<>::z();
	const Vector3D<> baseTdh5 = baseTend.P()-tcpZ*_lTcp;
	const std::pair<double,double> baseAngles = findBaseAngle(Vector2D<>(baseTdh5[0],baseTdh5[1]));

	// Now add up to 4 solutions for the first base angle
	addBaseAngleSolutions(baseTend,baseTdh5,state,baseAngles.first,res);
	// ... and up to 4 solutions for the second base angle
	addBaseAngleSolutions(baseTend,baseTdh5,state,baseAngles.second,res);

	return res;
}

Q ClosedFormURSolver::adjustJoints(const Q& q) const {
	const Q qMin = _device->getBounds().first;
	const Q qMax = _device->getBounds().second;
	Q qRes;
	qRes = q;
	for (std::size_t i = 0; i < _device->getDOF(); i++) {
		while(qRes[i] < qMin[i])	qRes[i] += 2.*Pi;
		while(qRes[i] > qMax[i])	qRes[i] -= 2.*Pi;
	}
	return qRes;
}

void ClosedFormURSolver::addBaseAngleSolutions(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, const State& state, double angle, std::vector<Q>& res) const {
	State tmpState = state;
	Q tmpQ = Q(6,angle,0,0,0,0,0);
	_device->setQ(tmpQ,tmpState);
	const Vector3D<> tcpZ = baseTend.R().getCol(2);

	// Determine the two possible positions of joint 4 for the current base angle.
	const std::pair<Vector3D<>,Vector3D<> > j4Pos = getJoint4Positions(baseTdh5,tcpZ,tmpState);

	// For each possible position of joint 4, find the elbow up and elbow down solutions
	const std::pair<std::pair<double,double>,std::pair<double,double> > elbowPos1 = getElbowJoints(j4Pos.first,tmpState);
	const std::pair<std::pair<double,double>,std::pair<double,double> > elbowPos2 = getElbowJoints(j4Pos.second,tmpState);

	// Add the four potential solutions for the current base angle
	addElbowSolutions(baseTend, baseTdh5, tmpState, angle, elbowPos1.first, res);
	addElbowSolutions(baseTend, baseTdh5, tmpState, angle, elbowPos1.second, res);
	addElbowSolutions(baseTend, baseTdh5, tmpState, angle, elbowPos2.first, res);
	addElbowSolutions(baseTend, baseTdh5, tmpState, angle, elbowPos2.second, res);
}

void ClosedFormURSolver::addElbowSolutions(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, const State& state, double baseAngle, std::pair<double,double> elbow, std::vector<Q>& res) const {
	// Elbow 1
	if (!Math::isNaN(elbow.first) && !Math::isNaN(elbow.second)) {
		State tmpState = state;
		Q tmpQ = Q(6,baseAngle,elbow.first,elbow.second,0,0,0);
		_device->setQ(tmpQ,tmpState);
		// The three first joints has been determined - find the last three joints.
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		const Q q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
	}
}

std::pair<Vector3D<>,Vector3D<> > ClosedFormURSolver::getJoint4Positions(const Vector3D<>& baseTdh5, const Vector3D<>& tcpZ, const State& state) const {
	// Determine the two possible positions of joint 4.
	const Vector3D<> xDir = normalize(getPerpendicularVector(tcpZ));
	const Vector3D<> yDir = normalize(cross(tcpZ,xDir));
	const Vector3D<> dir = Kinematics::frameTframe(_frames[0],_frames[2],state).R()*Vector3D<>::z();
	return findCirclePlaneIntersection(baseTdh5, _endCircleRadius, xDir, yDir, dir);
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormURSolver::getElbowJoints(const Vector3D<>& intersection, const State& state) const {
	// Only use the base joint angle, and set remaining joints to zero.
	State tmpState = state;
	Q tmpQ = _device->getQ(state);
	tmpQ[1] = 0;
	tmpQ[2] = 0;
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ,tmpState);

	// Determine the 2 possible configurations for joint 1 and joint 2, such that joint 4 reaches the given point
	const Transform3D<> dh0Tbase = Kinematics::frameTframe(_frames[1],_frames[0],tmpState);
	Vector3D<> target = dh0Tbase*intersection;
	target[2] -= _lJ0J1;
	const EAA<> eaa(0,Pi/2.,0);
	target = Transform3D<>(Vector3D<>::zero(),eaa.toRotation3D())*target;
	std::pair<std::pair<double,double>,std::pair<double,double> > a1a2 = findTwoBarAngles(Vector2D<>(target[2],target[0]),_l1,_l2);
	a1a2.first.first *= -1;
	a1a2.first.second *= -1;
	a1a2.second.first *= -1;
	a1a2.second.second *= -1;
	return a1a2;
}

Q ClosedFormURSolver::getOrientationJoints(const Transform3D<>& baseTend, const Vector3D<>& baseTdh5, const State& state) const {
	// Set last three joints to zero.
	State tmpState = state;
	Q tmpQ = _device->getQ(state);
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ,tmpState);

	// Joint 3
	const Vector3D<> dh4Tcenter = Kinematics::frameTframe(_frames[5],_frames[0],tmpState)*baseTdh5;
	tmpQ[3] = std::atan2(dh4Tcenter[0],dh4Tcenter[2]);
	_device->setQ(tmpQ,tmpState);

	// Joint 4
	const Vector3D<> dh5Ttcp = Kinematics::frameTframe(_frames[6],_frames[0],tmpState)*baseTend.P();
	tmpQ[4] = -std::atan2(dh5Ttcp[0],dh5Ttcp[2]);
	_device->setQ(tmpQ,tmpState);

	// Joint 5
	const Vector3D<> tcpX = (Kinematics::frameTframe(_frames[7],_frames[0],tmpState)*baseTend).R()*Vector3D<>::x();
	tmpQ[5] = std::atan2(tcpX[1],tcpX[0]);

	return tmpQ;
}

void ClosedFormURSolver::setCheckJointLimits(bool check) {
	_checkJointLimits = check;
}

std::pair<double,double> ClosedFormURSolver::findBaseAngle(const Vector2D<> &pos) const {
	std::pair<double,double> res;
	const double D = std::sqrt(std::pow(pos.norm2(),2)-std::pow(_baseRadius,2));
	res.first	= std::atan2(-D*pos[0]+_baseRadius*pos[1],+D*pos[1]+_baseRadius*pos[0]);
	res.second	= std::atan2(+D*pos[0]+_baseRadius*pos[1],-D*pos[1]+_baseRadius*pos[0]);
	res.first  += Pi/2;
	res.second  += Pi/2;
	if (res.first > Pi)
		res.first -= 2*Pi;
	if (res.second > Pi)
		res.second -= 2*Pi;
	return res;
}

std::pair<Vector3D<>,Vector3D<> > ClosedFormURSolver::findCirclePlaneIntersection(const Vector3D<>& circleCenter, double radius, const Vector3D<>& circleDir1, const Vector3D<>& circleDir2, const Vector3D<>& planeNormal) {
	std::pair<Vector3D<>,Vector3D<> > res;
	const double t = atan2(-dot(circleDir1,planeNormal),dot(circleDir2,planeNormal));
	res.first = circleCenter + radius*(circleDir1*cos(t)+circleDir2*sin(t));
	res.second = circleCenter + radius*(circleDir1*cos(t+Pi)+circleDir2*sin(t+Pi));
	return res;
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormURSolver::findTwoBarAngles(const Vector2D<>& pos, double L1, double L2) {
	std::pair<std::pair<double,double>,std::pair<double,double> > res;
	std::pair<double,double> res1, res2;
	res1.second = 2*std::atan(std::sqrt((std::pow(L1+L2,2)-std::pow(pos[0],2)-std::pow(pos[1],2))/(std::pow(pos[0],2)+std::pow(pos[1],2)-std::pow(L1-L2,2))));
	res2.second = -res1.second;
	res1.first = std::atan2(pos[1]*(L1+L2*cos(res1.second))-pos[0]*(L2*sin(res1.second)),pos[0]*(L1+L2*cos(res1.second))+pos[1]*(L2*sin(res1.second)));
	res2.first = std::atan2(pos[1]*(L1+L2*cos(res2.second))-pos[0]*(L2*sin(res2.second)),pos[0]*(L1+L2*cos(res2.second))+pos[1]*(L2*sin(res2.second)));
	res.first = res1;
	res.second = res2;
	return res;
}

Vector3D<> ClosedFormURSolver::getPerpendicularVector(const Vector3D<> &vec) {
	if (abs(vec[0]) < abs(vec[1]) && abs(vec[0]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(1,0,0));
	} else
	if (abs(vec[1]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(0,1,0));
	} else {
		return cross(vec,Vector3D<>(0,0,1));
	}
}
