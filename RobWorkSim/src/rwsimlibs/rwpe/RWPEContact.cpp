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

#include <rwsim/dynamics/Body.hpp>
#include "RWPEBody.hpp"
#include "RWPEContact.hpp"
#include "RWPEFrictionModel.hpp"
#include "RWPEFrictionModelData.hpp"
#include "RWPEIslandState.hpp"

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::rwpe;

RWPEContact::RWPEContact(const RWPEBody* parent, const RWPEBody* child, const RWPEFrictionModel& friction):
	RWPEConstraint(parent,child),
	_leaving(false),
	_linearType(Sticking),
	_angularType(Sticking),
	_friction(friction)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
}

RWPEContact::RWPEContact(const RWPEBody* parent, const RWPEBody* child, const RWPEFrictionModel& friction, const Contact &contact, const State &state):
	RWPEConstraint(parent,child),
	_leaving(false),
	_linearType(Sticking),
	_angularType(Sticking),
	_contact(contact),
	_friction(friction)
{
	RW_ASSERT(parent != NULL);
	RW_ASSERT(child != NULL);
	setContact(contact,state);
}

RWPEContact::~RWPEContact() {
}

bool RWPEContact::isLeaving() const {
	return _leaving;
}

RWPEContact::Type RWPEContact::getTypeLinear() const {
	return _linearType;
}

RWPEContact::Type RWPEContact::getTypeAngular() const {
	return _angularType;
}

void RWPEContact::setTypeLeaving() {
	_leaving = true;
}

void RWPEContact::setType(Type linear, Type angular) {
	_leaving = false;
	_linearType = linear;
	_angularType = angular;
}

const Contact& RWPEContact::getContact() const {
	return _contact;
}

void RWPEContact::setContact(const Contact &contact, const State &state) {
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

void RWPEContact::update(RWPEIslandState &islandState, const State &rwstate) {
	RWPEFrictionModelData* data = islandState.getFrictionData(this);
	if (data == NULL) {
		data = _friction.makeDataStructure();
		islandState.setFrictionData(this,data);
	}
	const Wrench6D<> friction = _friction.getViscuousFriction(*this,islandState,rwstate,data);
	islandState.setWrenchApplied(this,friction);
}

void RWPEContact::reset(RWPEIslandState &islandState, const State &rwstate) {
	clearWrenchApplied(islandState);
	islandState.clearFrictionData(this);
}

void RWPEContact::step(RWPEIslandState &islandState, const State &rwstate, double h) {
	RWPEFrictionModelData* data = islandState.getFrictionData(this);
	if (data == NULL) {
		data = _friction.makeDataStructure();
		islandState.setFrictionData(this,data);
	}
	RWPEFrictionModel::DryFriction friction;
	_friction.updateData(*this,islandState,rwstate,h,data);
	friction = _friction.getDryFriction(*this,islandState,rwstate,data);
	if (friction.enableTangent) {
		const Frame* const frameP = getParent()->get()->getBodyFrame();
		const Transform3D<> wTp = Kinematics::worldTframe(frameP,rwstate);
		const Vector3D<> dir = inverse(wTp.R())*friction.tangentDirection;
		const Vector3D<> normalP = _rotLinParent.getCol(2);
		const Vector3D<> frictionDir_proj = normalize(dir-dot(normalP,dir)*normalP);

		const Vector3D<> constraintDir = normalize(normalP+friction.tangent*frictionDir_proj);
		const Vector3D<> rotDir = normalize(cross(normalP,constraintDir));
		const Vector3D<> sinN = cross(normalP,constraintDir);
		const double cos = dot(normalP,constraintDir);
		const double angle = Math::sign(dot(sinN,rotDir))*atan2(sinN.norm2(),cos);
		const Rotation3D<> Rp = EAA<>(rotDir,angle).toRotation3D();
		const Rotation3D<> Rc = EAA<>(rotDir,angle).toRotation3D();
		_offsetParent = Rp;
		_offsetChild = Rc;
	}
}

std::vector<RWPEConstraint::Mode> RWPEContact::getConstraintModes() const {
	std::vector<RWPEConstraint::Mode> modes;
	if (_leaving || _linearType == None || _linearType == Sliding) {
		modes.push_back(Free);
		modes.push_back(Free);
	} else if (_linearType == Sticking) {
		modes.push_back(Velocity);
		modes.push_back(Velocity);
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

std::size_t RWPEContact::getDimVelocity() const {
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

std::size_t RWPEContact::getDimWrench() const {
	std::size_t dim = 0;
	if (!_leaving) {
		if (_angularType == Sliding)
			dim++;
	}
	return dim;
}

std::size_t RWPEContact::getDimFree() const {
	std::size_t dim = 0;
	if (_leaving) {
		dim = 6;
	} else {
		dim = 2;
		if (_linearType == None || _linearType == Sliding)
			dim += 2;
		if (_angularType == None)
			dim++;
	}
	return dim;
}

Rotation3D<> RWPEContact::getLinearRotationParentForceW(const RWPEIslandState &state) const {
	return getParent()->getWorldTcom(state).R()*_offsetParent*_rotLinParent;
}

Rotation3D<> RWPEContact::getLinearRotationChildForceW(const RWPEIslandState &state) const {
	return getChild()->getWorldTcom(state).R()*_offsetChild*_rotLinChild;
}

Vector3D<> RWPEContact::getNormalW(const RWPEIslandState &islandState) const {
	return getLinearRotationParentW(islandState).getCol(2);
}

Vector3D<> RWPEContact::getFrictionDirW(const RWPEIslandState &islandState) const {
	return getLinearRotationParentW(islandState).getCol(0);
}

std::string RWPEContact::toString(Type type) {
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
