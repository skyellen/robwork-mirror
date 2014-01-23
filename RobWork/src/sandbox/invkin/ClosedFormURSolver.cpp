/*
 * ClosedFormURSolver.cpp
 *
 *  Created on: 26/07/2012
 *      Author: thomas
 */

#include "ClosedFormURSolver.hpp"
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Models.hpp>
#include <iostream>
#include <string>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;

ClosedFormURSolver::ClosedFormURSolver(const rw::models::Device::Ptr device, const rw::kinematics::State& state):
	_checkJointLimits(true),
	_device(device)
{
	Frame* frame = device->getEnd();
	std::string ss = device->getName();
	std::string str(".Joint1");
	ss.append(str);
	while (frame->getName().compare(ss)) {
		_frames.push_back(frame);
		frame = frame->getParent();
	}
	_frames.push_back(frame);
	_frames.push_back(frame->getParent());
	std::reverse(_frames.begin(),_frames.end());
	Transform3D<> trans = Kinematics::frameTframe(_frames[0],_frames[6],state);
	Vector3D<> direction = Kinematics::frameTframe(_frames[0],_frames[2],state).R()*Vector3D<>::z();
	_baseRadius = dot(trans.P(),direction);
	Transform3D<> endTrans = Kinematics::frameTframe(_frames[5],_frames[6],state);
	_endCircleRadius = endTrans.P()[2];
	Transform3D<> tr1 = Kinematics::frameTframe(_frames[2],_frames[3],state);
	_l1 = tr1.P().norm2();
	Transform3D<> tr2 = Kinematics::frameTframe(_frames[3],_frames[4],state);
	_l2 = tr2.P().norm2();
	Transform3D<> trTcp = Kinematics::frameTframe(_frames[6],_frames[7],state);
	_lTcp = trTcp.P().norm2();
	Transform3D<> trJ0J1 = Kinematics::frameTframe(_frames[1],_frames[2],state);
	_l2 = tr2.P().norm2();
	_lJ0J1 = trJ0J1.P().norm2();
}

ClosedFormURSolver::~ClosedFormURSolver() {
}

std::vector<Q> ClosedFormURSolver::solve(const Transform3D<>& baseTend, const State& state) const {
	std::vector<Q> res;
    Vector3D<> tcpZ = baseTend.R()*Vector3D<>::z();
	Vector3D<> baseTdh5 = baseTend.P()-tcpZ*_lTcp;
	Vector2D<> pos(baseTdh5[0],baseTdh5[1]);
	std::pair<double,double> baseAngles = findBaseAngle(pos);

	Vector3D<> rot;
	std::pair<Vector3D<>,Vector3D<> > j4Pos;
	std::pair<std::pair<double,double>,std::pair<double,double> > elbow;
	std::pair<double,double> elbow1, elbow2;
	Q tmpQ, q;
	State tmpState;

	// Base Angle 1
	tmpState = state;
	tmpQ = Q(6,baseAngles.first,0,0,0,0,0);
	_device->setQ(tmpQ,tmpState);
	j4Pos = getJoint4Positions(baseTdh5,tcpZ,tmpState);

	// Position 1
	elbow = getElbowJoints(j4Pos.first,tmpState);
	elbow1 = elbow.first;
	elbow2 = elbow.second;
	// Elbow 1
	if (!Math::isNaN(elbow1.first) && !Math::isNaN(elbow1.second)) {
		tmpQ[1] = elbow1.first;
		tmpQ[2] = elbow1.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
		// Elbow 2
		tmpQ[1] = elbow2.first;
		tmpQ[2] = elbow2.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
	}

	// Position 2
	elbow = getElbowJoints(j4Pos.second,tmpState);
	elbow1 = elbow.first;
	elbow2 = elbow.second;
	// Elbow 1
	if (!Math::isNaN(elbow2.first) && !Math::isNaN(elbow2.second)) {
		tmpQ[1] = elbow1.first;
		tmpQ[2] = elbow1.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
		// Elbow 2
		tmpQ[1] = elbow2.first;
		tmpQ[2] = elbow2.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
	}

	// Base Angle 2
	tmpState = state;
	tmpQ = Q(6,baseAngles.second,0,0,0,0,0);
	_device->setQ(tmpQ,tmpState);
	j4Pos = getJoint4Positions(baseTdh5,tcpZ,tmpState);

	// Position 1
	elbow = getElbowJoints(j4Pos.first,tmpState);
	elbow1 = elbow.first;
	elbow2 = elbow.second;
	// Elbow 1
	if (!Math::isNaN(elbow1.first) && !Math::isNaN(elbow1.second)) {
		tmpQ[1] = elbow1.first;
		tmpQ[2] = elbow1.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
		// Elbow 2
		tmpQ[1] = elbow2.first;
		tmpQ[2] = elbow2.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
	}

	// Position 2
	elbow = getElbowJoints(j4Pos.second,tmpState);
	elbow1 = elbow.first;
	elbow2 = elbow.second;
	// Elbow 1
	if (!Math::isNaN(elbow2.first) && !Math::isNaN(elbow2.second)) {
		tmpQ[1] = elbow1.first;
		tmpQ[2] = elbow1.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
		// Elbow 2
		tmpQ[1] = elbow2.first;
		tmpQ[2] = elbow2.second;
		_device->setQ(tmpQ,tmpState);
		tmpQ = getOrientationJoints(baseTend,baseTdh5,tmpState);
		q = adjustJoints(tmpQ);
		if (!_checkJointLimits)
			res.push_back(q);
		else {
			if (Models::inBounds(q,*_device))
				res.push_back(q);
		}
	}

	return res;
}

Q ClosedFormURSolver::adjustJoints(const Q &q) const {
	Q qMin = _device->getBounds().first;
	Q qMax = _device->getBounds().second;
	Q qRes;
	qRes = q;
	for (std::size_t i = 0; i < _device->getDOF(); i++) {
		while(qRes[i] < qMin[i])	qRes[i] += 2.*Pi;
		while(qRes[i] > qMax[i])	qRes[i] -= 2.*Pi;
	}
	return qRes;
}

std::pair<Vector3D<>,Vector3D<> > ClosedFormURSolver::getJoint4Positions(const Vector3D<> &baseTdh5, const Vector3D<> &tcpZ, const State &state) const {
	Vector3D<> xDir = normalize(getPerpendicularVector(tcpZ));
	Vector3D<> yDir = normalize(cross(tcpZ,xDir));
	Vector3D<> dir = Kinematics::frameTframe(_frames[0],_frames[2],state).R()*Vector3D<>::z();
	std::pair<Vector3D<>,Vector3D<> > intersections = findCirclePlaneIntersection(baseTdh5, _endCircleRadius, xDir, yDir, dir);
	return intersections;
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormURSolver::getElbowJoints(const Vector3D<> &intersection, const State &state) const {
	State tmpState = state;
	Q tmpQ = _device->getQ(state);
	tmpQ[1] = 0;
	tmpQ[2] = 0;
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ,tmpState);
	Transform3D<> dh0Tbase = Kinematics::frameTframe(_frames[1],_frames[0],tmpState);
	Vector3D<> target = dh0Tbase*intersection;
	target[2] -= _lJ0J1;
	EAA<> eaa(0,Pi/2.,0);
	target = Transform3D<>(Vector3D<>::zero(),eaa.toRotation3D())*target;
	std::pair<std::pair<double,double>,std::pair<double,double> > a1a2 = findTwoBarAngles(Vector2D<>(target[2],target[0]),_l1,_l2);
	std::pair<double,double> a1 = a1a2.first;
	std::pair<double,double> a2 = a1a2.second;
	a1.first *= -1;
	a1.second *= -1;
	a2.first *= -1;
	a2.second *= -1;
	std::pair<std::pair<double,double>,std::pair<double,double> > res;
	res.first = a1;
	res.second = a2;
	return res;
}

Q ClosedFormURSolver::getOrientationJoints(const Transform3D<> &baseTend, const Vector3D<> &baseTdh5, const State &state) const {
	State tmpState = state;
	Q tmpQ = _device->getQ(state);
	tmpQ[3] = 0;
	tmpQ[4] = 0;
	tmpQ[5] = 0;
	_device->setQ(tmpQ,tmpState);
	Vector3D<> dh4Tcenter = Kinematics::frameTframe(_frames[5],_frames[0],tmpState)*baseTdh5;
	tmpQ[3] = std::atan2(dh4Tcenter[0],dh4Tcenter[2]);
	_device->setQ(tmpQ,tmpState);
	Vector3D<> dh5Ttcp = Kinematics::frameTframe(_frames[6],_frames[0],tmpState)*baseTend.P();
	tmpQ[4] = -std::atan2(dh5Ttcp[0],dh5Ttcp[2]);
	_device->setQ(tmpQ,tmpState);
	Vector3D<> tcpX = (Kinematics::frameTframe(_frames[7],_frames[0],tmpState)*baseTend).R()*Vector3D<>::x();
	tmpQ[5] = std::atan2(tcpX[1],tcpX[0]);
	return tmpQ;
}

void ClosedFormURSolver::setCheckJointLimits(bool check) {
	_checkJointLimits = check;
}

std::pair<double,double> ClosedFormURSolver::findBaseAngle(const Vector2D<> &pos) const {
	std::pair<double,double> res;
	double D = std::sqrt(std::pow(pos.norm2(),2)-std::pow(_baseRadius,2));
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

std::pair<Vector3D<>,Vector3D<> > ClosedFormURSolver::findCirclePlaneIntersection(const Vector3D<> &circleCenter, double radius, const Vector3D<> &circleDir1, const Vector3D<> &circleDir2, const Vector3D<> &planeNormal) {
	std::pair<Vector3D<>,Vector3D<> > res;
	double t = atan2(-dot(circleDir1,planeNormal),dot(circleDir2,planeNormal));
	res.first = circleCenter + radius*(circleDir1*cos(t)+circleDir2*sin(t));
	res.second = circleCenter + radius*(circleDir1*cos(t+Pi)+circleDir2*sin(t+Pi));
	return res;
}

std::pair<std::pair<double,double>,std::pair<double,double> > ClosedFormURSolver::findTwoBarAngles(const rw::math::Vector2D<> &pos, double L1, double L2) {
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

rw::math::Vector3D<> ClosedFormURSolver::getPerpendicularVector(const rw::math::Vector3D<> &vec) {
	if (abs(vec[0]) < abs(vec[1]) && abs(vec[0]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(1,0,0));
	} else
	if (abs(vec[1]) < abs(vec[2])) {
		return cross(vec,Vector3D<>(0,1,0));
	} else {
		return cross(vec,Vector3D<>(0,0,1));
	}
}
