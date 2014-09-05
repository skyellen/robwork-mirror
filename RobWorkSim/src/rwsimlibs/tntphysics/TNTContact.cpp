/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTContact.hpp"
#include "TNTBody.hpp"

#include <rwsim/dynamics/Body.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::tntphysics;

TNTContact::TNTContact(const TNTBody* parent, const TNTBody* child):
	TNTConstraint(parent,child),
	_leaving(false),
	_linearType(Sticking),
	_angularType(Sticking),
	_muLin(0),
	_muAng(0),
	_muLinViscuous(0),
	_muAngViscuous(0)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
}

TNTContact::TNTContact(const TNTBody* parent, const TNTBody* child, const Contact &contact, const State &state):
	TNTConstraint(parent,child),
	_leaving(false),
	_linearType(Sticking),
	_angularType(Sticking),
	_contact(contact),
	_muLin(0),
	_muAng(0),
	_muLinViscuous(0),
	_muAngViscuous(0)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
	setContact(contact,state);
}

TNTContact::~TNTContact() {
}

bool TNTContact::isLeaving() const {
	return _leaving;
}

TNTContact::Type TNTContact::getTypeLinear() const {
	return _linearType;
}

TNTContact::Type TNTContact::getTypeAngular() const {
	return _angularType;
}

void TNTContact::setTypeLeaving() {
	_leaving = true;
}

void TNTContact::setType(Type linear, Type angular) {
	_leaving = false;
	_linearType = linear;
	_angularType = angular;
}

const Contact& TNTContact::getContact() const {
	return _contact;
}

void TNTContact::setContact(const Contact &contact, const State &state) {
	// Construct transforms for the constraint
	const Vector3D<> pointA = contact.getPointA();
	const Vector3D<> pointB = contact.getPointB();
	const Vector3D<> n = contact.getNormal();
	const Rotation3D<> R = EAA<>(Vector3D<>::z(),n).toRotation3D();
	const Transform3D<> wTa(pointA,R);
	const Transform3D<> wTb(pointB,R);

	// Find frames and construct body local transforms relative to com
	const Frame* const frameP = getParent()->get()->getBodyFrame();
	const Frame* const frameC = getChild()->get()->getBodyFrame();
	Transform3D<> pTc = inverse(Kinematics::worldTframe(frameP,state))*wTa;
	Transform3D<> cTc = inverse(Kinematics::worldTframe(frameC,state))*wTb;
	const Vector3D<> comP = getParent()->get()->getInfo().masscenter;
	const Vector3D<> comC = getChild()->get()->getInfo().masscenter;
	pTc.P() -= comP;
	cTc.P() -= comC;
	_posParent = pTc.P();
	_posChild = cTc.P();
	_rotLinParent = pTc.R();
	_rotLinChild = cTc.R();
	_rotAngParent = pTc.R();
	_rotAngChild = cTc.R();
	_contact = contact;
}

void TNTContact::update(TNTIslandState &tntstate, const State &rwstate) {
}

void TNTContact::reset(TNTIslandState &tntstate, const State &rwstate) {
}

std::vector<TNTConstraint::Mode> TNTContact::getConstraintModes() const {
	std::vector<TNTConstraint::Mode> modes;
	if (_leaving || _linearType == None) {
		modes.push_back(Free);
		modes.push_back(Free);
	} else if (_linearType == Sticking) {
		modes.push_back(Velocity);
		modes.push_back(Velocity);
	} else if (_linearType == Sliding) {
		modes.push_back(Wrench);
		modes.push_back(Free);
	}
	if (_leaving)
		modes.push_back(Free);
	else
		modes.push_back(Velocity);
	modes.push_back(Free);
	modes.push_back(Free);
	if (_leaving || _angularType == None) {
		modes.push_back(Free);
	} else if (_angularType == Sticking) {
		modes.push_back(Velocity);
	} else if (_angularType == Sliding) {
		modes.push_back(Wrench);
	}
	return modes;
}

std::size_t TNTContact::getDimVelocity() const {
	std::size_t dim = 0;
	if (!_leaving) {
		dim++;
		if (_linearType == Sticking)
			dim += 2;
		if (_angularType == Sticking)
			dim++;
	}
	return dim;
}

std::size_t TNTContact::getDimWrench() const {
	std::size_t dim = 0;
	if (!_leaving) {
		if (_linearType == Sliding)
			dim++;
		if (_angularType == Sliding)
			dim++;
	}
	return dim;
}

std::size_t TNTContact::getDimFree() const {
	std::size_t dim = 0;
	if (_leaving) {
		dim = 6;
	} else {
		dim = 2;
		if (_linearType == None)
			dim += 2;
		else if (_linearType == Sliding)
			dim++;
		if (_angularType == None)
			dim++;
	}
	return dim;
}

Vector3D<> TNTContact::getNormalW(const TNTIslandState &tntstate) const {
	return getLinearRotationParentW(tntstate).getCol(2);
}

Vector3D<> TNTContact::getFrictionDirW(const TNTIslandState &tntstate) const {
	return getLinearRotationParentW(tntstate).getCol(0);
}

void TNTContact::setFrictionDir(const Vector3D<>& frictionDir) {
	const Vector3D<> x = _rotLinParent.getCol(0);
	const Vector3D<> normalP = _rotLinParent.getCol(2);
	const Vector3D<> normalC = _rotLinParent.getCol(2);
	const Vector3D<> frictionDir_proj = normalize(frictionDir-dot(normalP,frictionDir)*normalP);
	const Vector3D<> sinN = cross(x,frictionDir_proj);
	const double cos = dot(x,frictionDir_proj);
	const double angle = Math::sign(dot(sinN,normalP))*atan2(sinN.norm2(),cos);
	const Rotation3D<> Rp = EAA<>(normalP,angle).toRotation3D();
	const Rotation3D<> Rc = EAA<>(normalC,angle).toRotation3D();
	_rotLinParent = Rp*_rotLinParent;
	_rotLinChild = Rc*_rotLinChild;
	_rotAngParent = Rp*_rotAngParent;
	_rotAngChild = Rc*_rotAngChild;
}

void TNTContact::setFrictionDirW(const Vector3D<>& frictionDir, const State &state) {
	const Frame* const frameP = getParent()->get()->getBodyFrame();
	const Transform3D<> wTp = Kinematics::worldTframe(frameP,state);
	const Vector3D<> dir = inverse(wTp.R())*frictionDir;
	setFrictionDir(dir);
}

void TNTContact::setFriction(double linearCoefficient, double angularCoefficient, double absoluteLinear, double absoluteAngular) {
	_muLin = linearCoefficient;
	_muAng = angularCoefficient;
	_muLinViscuous = absoluteLinear;
	_muAngViscuous = absoluteAngular;
}

std::string TNTContact::toString(Type type) {
	switch (type) {
	case None:
		return "None";
	case Sliding:
		return "Sliding";
	case Sticking:
		return "Sticking";
	}
	return "ERROR";
}

Eigen::MatrixXd TNTContact::getWrenchModelLHS(const TNTConstraint* constraint) const {
	Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(getDimWrench(),6);
	if (constraint == this) {
		RW_ASSERT(!_leaving);
		RW_ASSERT(_linearType == Sliding || _angularType == Sliding);
		if (_linearType == Sliding && _angularType != Sliding) {
			matrix(0,0) = 1;
			matrix(0,2) = _muLin;
		} else if (_linearType != Sliding && _angularType == Sliding) {
			matrix(0,2) = _muAng;
			matrix(0,5) = 1;
		} else {
			matrix(0,0) = 1;
			matrix(0,2) = _muLin;
			matrix(1,2) = _muAng;
			matrix(1,5) = 1;
		}
	}
	return matrix;
}

Eigen::VectorXd TNTContact::getWrenchModelRHS() const {
	Eigen::VectorXd vector = Eigen::VectorXd::Zero(getDimWrench());
	if (_linearType == Sliding && _angularType != Sliding)
		vector[0] = _muLinViscuous;
	else if (_linearType != Sliding && _angularType == Sliding)
		vector[0] = _muAngViscuous;
	else if (_linearType == Sliding && _angularType == Sliding) {
		vector[0] = _muLinViscuous;
		vector[1] = _muAngViscuous;
	}
	return vector;
}
