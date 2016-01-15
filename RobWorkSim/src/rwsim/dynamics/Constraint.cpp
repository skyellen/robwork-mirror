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

#include "Constraint.hpp"

#include <boost/algorithm/string.hpp>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::dynamics;

Constraint::Constraint(const std::string& name, const ConstraintType &type, Body* b1, Body* b2):
	StateData(0,name),
	_type(type),
	_body1(b1),
	_body2(b2),
	_limits(getDOF(type))
{
}

Constraint::~Constraint() {
}

Constraint::ConstraintType Constraint::getType() const {
	return _type;
}

Body* Constraint::getBody1() const {
	return _body1;
}

Body* Constraint::getBody2() const {
	return _body2;
}

size_t Constraint::getDOF() const {
	return getDOFLinear(_type) + getDOFAngular(_type);
}

size_t Constraint::getDOFLinear() const {
	return getDOFLinear(_type);
}

size_t Constraint::getDOFAngular() const {
	return getDOFAngular(_type);
}

Transform3D<> Constraint::getTransform() const {
	boost::mutex::scoped_lock lock(_parentTconstraint_mutex);
	return _parentTconstraint;
}

void Constraint::setTransform(const Transform3D<> &parentTconstraint) {
	boost::mutex::scoped_lock lock(_parentTconstraint_mutex);
	_parentTconstraint = parentTconstraint;
}

Constraint::SpringParams Constraint::getSpringParams() const {
	boost::mutex::scoped_lock lock(_springParams_mutex);
	return _springParams;
}

void Constraint::setSpringParams(const SpringParams &params) {
	boost::mutex::scoped_lock lock(_springParams_mutex);
	_springParams = params;
}

Constraint::Limit Constraint::getLimit(std::size_t i) const {
	boost::mutex::scoped_lock lock(_limits_mutex);
	RW_ASSERT(i < _limits.size());
	return _limits[i];
}

void Constraint::setLimit(std::size_t i, const Limit& limit) {
	boost::mutex::scoped_lock lock(_limits_mutex);
	RW_ASSERT(i < _limits.size());
	_limits[i] = limit;
}

size_t Constraint::getDOF(ConstraintType type) {
	return getDOFLinear(type) + getDOFAngular(type);
}

size_t Constraint::getDOFLinear(ConstraintType type) {
	switch(type) {
	case Fixed:
		return 0;
	case Prismatic:
		return 1;
	case Revolute:
		return 0;
	case Universal:
		return 0;
	case Spherical:
		return 0;
	case Piston:
		return 1;
	case PrismaticRotoid:
		return 1;
	case PrismaticUniversal:
		return 1;
	case Free:
		return 3;
	default:
		RW_THROW("Constraint (getDOFLinear): the given constraint type is unknown!");
		return 0;
	}
	return 0;
}

size_t Constraint::getDOFAngular(ConstraintType type) {
	switch(type) {
	case Fixed:
		return 0;
	case Prismatic:
		return 0;
	case Revolute:
		return 1;
	case Universal:
		return 2;
	case Spherical:
		return 3;
	case Piston:
		return 1;
	case PrismaticRotoid:
		return 1;
	case PrismaticUniversal:
		return 2;
	case Free:
		return 3;
	default:
		RW_THROW("Constraint (getDOFLinear): the given constraint type is unknown!");
		return 0;
	}
	return 0;
}

bool Constraint::toConstraintType(const std::string &string, ConstraintType &type) {
    const std::string typeUpper = boost::to_upper_copy(string);
	if (typeUpper == "FIXED")
		type = Constraint::Fixed;
	else if (typeUpper == "PRISMATIC")
		type = Constraint::Prismatic;
	else if (typeUpper == "REVOLUTE")
		type = Constraint::Revolute;
	else if (typeUpper == "UNIVERSAL")
		type = Constraint::Universal;
	else if (typeUpper == "SPHERICAL")
		type = Constraint::Spherical;
	else if (typeUpper == "PISTON")
		type = Constraint::Piston;
	else if (typeUpper == "PRISMATICROTOID")
		type = Constraint::PrismaticRotoid;
	else if (typeUpper == "PRISMATICUNIVERSAL")
		type = Constraint::PrismaticUniversal;
	else if (typeUpper == "FREE")
		type = Constraint::Free;
	else
		return false;
	return true;
}
