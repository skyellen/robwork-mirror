/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "Contact.hpp"
#include "ContactModel.hpp"

#include <rw/common/InputArchive.hpp>
#include <rw/common/OutputArchive.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rwsim::contacts;

#define UNKNOWN_NAME "Unknown"

Contact::Contact():
	_frameA(NULL),
	_frameB(NULL),
	_nameA(UNKNOWN_NAME),
	_nameB(UNKNOWN_NAME),
	_depth(0)
{
}

Contact::~Contact() {
}

ContactModel::Ptr Contact::getModelA() const {
	return _a;
}

ContactModel::Ptr Contact::getModelB() const {
	return _b;
}

const Frame* Contact::getFrameA() const {
	return _frameA;
}

const Frame* Contact::getFrameB() const {
	return _frameB;
}

const std::string& Contact::getNameA() const {
	return _nameA;
}

const std::string& Contact::getNameB() const {
	return _nameB;
}

Transform3D<> Contact::aTb() const {
	return _aTb;
}

Vector3D<> Contact::getPointA() const {
	return _pointA;
}

Vector3D<> Contact::getPointB() const {
	return _pointB;
}

Vector3D<> Contact::getNormal() const {
	return _normal;
}

double Contact::getDepth() const {
	return _depth;
}

void Contact::setModelA(ContactModel::Ptr modelA) {
	_a = modelA;
}

void Contact::setModelB(ContactModel::Ptr modelB) {
	_b = modelB;
}

void Contact::setFrameA(const Frame* frame) {
	if (frame == NULL)
		_nameA = UNKNOWN_NAME;
	else
		_nameA = frame->getName();
	_frameA = frame;
}

void Contact::setFrameB(const Frame* frame) {
	if (frame == NULL)
		_nameB = UNKNOWN_NAME;
	else
		_nameB = frame->getName();
	_frameB = frame;
}

void Contact::setNameA(const std::string& name) {
	_frameA = NULL;
	_nameA = name;
}

void Contact::setNameB(const std::string& name) {
	_frameB = NULL;
	_nameB = name;
}

bool Contact::setFrames(const WorkCell& wc) {
	const Frame* const frameA = wc.findFrame(_nameA);
	const Frame* const frameB = wc.findFrame(_nameB);
	if (frameA != NULL && frameB != NULL) {
		_frameA = frameA;
		_frameB = frameB;
		return true;
	} else {
		return false;
	}
}

void Contact::setTransform(Transform3D<> aTb) {
	_aTb = aTb;
}

void Contact::setPointA(Vector3D<> pointA) {
	_pointA = pointA;
}

void Contact::setPointB(Vector3D<> pointB) {
	_pointB = pointB;
}

void Contact::setPoints(rw::math::Vector3D<> pointA, rw::math::Vector3D<> pointB) {
	_pointA = pointA;
	_pointB = pointB;
}

void Contact::setNormal(Vector3D<> normal) {
	_normal = normal;
}

void Contact::setDepth() {
	//double dist = (_pointA-_pointB).norm2();
	//if (dot(_pointA-_pointB,_normal) < 0)
	//	_depth = -dist;
	//else
	//	_depth = dist;
	_depth = dot(_pointA-_pointB,_normal);
}

void Contact::setDepth(double depth) {
	_depth = depth;
}

bool Contact::operator==(const Contact& b) const {
	if (!(b.getModelA() == getModelA()))
		return false;
	if (!(b.getModelB() == getModelB()))
		return false;
	if (b.getFrameA() != getFrameA())
		return false;
	if (b.getFrameB() != getFrameB())
		return false;
	if (!(b.aTb() == aTb()))
		return false;
	if (!(b.getPointA() == getPointA()))
		return false;
	if (!(b.getPointB() == getPointB()))
		return false;
	if (!(b.getNormal() == getNormal()))
		return false;
	if (b.getDepth() != getDepth())
		return false;
	return true;
}

bool Contact::operator!=(const Contact& b) const {
	return !operator==(b);
}

void Contact::read(class InputArchive& iarchive, const std::string& id) {
	iarchive.read(_pointA,"PointA");
	iarchive.read(_pointB,"PointB");
	iarchive.read(_normal,"Normal");
	iarchive.read(_depth,"Depth");
	iarchive.read(_aTb,"aTb");
	_frameA = NULL;
	_frameB = NULL;
	_nameA = iarchive.readString("NameA");
	_nameB = iarchive.readString("NameB");
	// Contact models can not be deserialized (ignored)
}

void Contact::write(class OutputArchive& oarchive, const std::string& id) const {
	oarchive.write(_pointA,"PointA");
	oarchive.write(_pointB,"PointB");
	oarchive.write(_normal,"Normal");
	oarchive.write(_depth,"Depth");
	oarchive.write(_aTb,"aTb");
	oarchive.write(_nameA,"NameA");
	oarchive.write(_nameB,"NameB");
	// Contact models can not be deserialized (ignored)
}

namespace rwsim {
namespace contacts {
std::ostream& operator<<(std::ostream& out, const Contact& contact) {
	out << "Frames: " << contact.getNameA() << " <-> " << contact.getNameB();
	out << " Points: " << contact.getPointA() << " <-> " << contact.getPointB();
	out << " Normal: " << contact.getNormal() << " Depth: " << contact.getDepth();
	return out;
}
}
}
