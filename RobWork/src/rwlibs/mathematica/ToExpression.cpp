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

#include "ToExpression.hpp"

using namespace rw::common;
using namespace rwlibs::mathematica;

ToExpression::ToExpression(const char* expression):
	FunctionBase("ToExpression"),
	_expression(ownedPtr(new Mathematica::String(expression)))
{
}

ToExpression::ToExpression(const std::string& expression):
	FunctionBase("ToExpression"),
	_expression(ownedPtr(new Mathematica::String(expression)))
{
}

ToExpression::ToExpression(const Mathematica::String& expression):
	FunctionBase("ToExpression"),
	_expression(expression.clone().cast<Mathematica::String>())
{
}

ToExpression::~ToExpression() {
}

std::list<rw::common::Ptr<const Mathematica::Expression> > ToExpression::getArguments() const {
	return std::list<rw::common::Ptr<const Mathematica::Expression> >(1,_expression);
}

Mathematica::Expression::Ptr ToExpression::clone() const {
	return ownedPtr(new ToExpression(*_expression));
}
