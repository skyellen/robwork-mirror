/*
 * Contact.cpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#include "Contact.hpp"

using namespace rw::math;
using namespace rwsim::contacts;

Contact::Contact():
	_depth(0)
{
}

virtual Contact::~Contact() {
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

void Contact::getPointB(Vector3D<> pointB) {
	_pointB = pointB;
}

void Contact::getNormal(Vector3D<> normal) {
	_normal = normal;
}

void Contact::getDepth(double depth) {
	_depth = depth;
}

void Contact::clear(){
}
