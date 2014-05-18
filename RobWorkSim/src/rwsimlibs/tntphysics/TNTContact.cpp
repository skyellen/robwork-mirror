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
	_angularType(Sticking)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
}

TNTContact::TNTContact(const TNTBody* parent, const TNTBody* child, const Contact &contact, const State &state):
	TNTConstraint(parent,child),
	_leaving(false),
	_linearType(Sticking),
	_angularType(Sticking),
	_contact(contact)
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
	if (_leaving) {
		modes.push_back(Wrench);
		modes.push_back(Wrench);
		modes.push_back(Wrench);
	} else if (_linearType == Sticking) {
		modes.push_back(Velocity);
		modes.push_back(Velocity);
		modes.push_back(Velocity);
	} else if (_linearType == Sliding) {
		modes.push_back(Wrench);
		modes.push_back(Wrench);
		modes.push_back(Velocity);
	}
	modes.push_back(Wrench);
	modes.push_back(Wrench);
	if (_leaving || _angularType == Sliding) {
		modes.push_back(Wrench);
	} else if (_angularType == Sticking) {
		modes.push_back(Velocity);
	}
	return modes;
}

std::size_t TNTContact::getDimVelocity() const {
	if (_leaving)
		return 0;
	else if (_linearType == Sticking && _angularType == Sticking)
		return 4;
	else if (_linearType == Sticking && _angularType == Sliding)
		return 3;
	else if (_linearType == Sliding && _angularType == Sticking)
		return 2;
	else if (_linearType == Sliding && _angularType == Sliding)
		return 1;
	return 0;
}

std::size_t TNTContact::getDimWrench() const {
	return 6-getDimVelocity();
}

Vector3D<> TNTContact::getNormalW(const TNTIslandState &tntstate) const {
	return getLinearRotationParentW(tntstate).getCol(2);
}
