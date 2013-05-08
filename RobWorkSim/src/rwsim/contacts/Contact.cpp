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

using namespace rw::math;
using namespace rwsim::contacts;

Contact::Contact():
	_depth(0)
{
}

Contact::~Contact() {
	clear();
}

ContactModel::Ptr Contact::getModelA() const {
	return _a;
}

ContactModel::Ptr Contact::getModelB() const {
	return _b;
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

void Contact::setTransform(Transform3D<> aTb) {
	_aTb = aTb;
}

void Contact::setPointA(Vector3D<> pointA) {
	_pointA = pointA;
}

void Contact::setPointB(Vector3D<> pointB) {
	_pointB = pointB;
}

void Contact::setNormal(Vector3D<> normal) {
	_normal = normal;
}

void Contact::setDepth(double depth) {
	_depth = depth;
}

void Contact::clear(){
}
