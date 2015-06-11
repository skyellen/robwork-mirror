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

#include "List.hpp"

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

List::List():
	FunctionBase("List")
{
}

List::List(const std::vector<Expression::Ptr>& args):
	FunctionBase("List")
{
	BOOST_FOREACH(const Expression::Ptr arg, args) {
		const rw::common::Ptr<const FunctionBase> fct = arg.cast<const FunctionBase>();
		if (!fct.isNull()) {
			if (fct->getName() == "List")
				_data.push_back(List::fromExpression(*fct));
			else
				_data.push_back(arg);
		} else {
			_data.push_back(arg);
		}
	}
}

List::List(const std::list<rw::common::Ptr<const Expression> >& args):
	FunctionBase("List")
{
	BOOST_FOREACH(const rw::common::Ptr<const Expression> arg, args) {
		const rw::common::Ptr<const FunctionBase> fct = arg.cast<const FunctionBase>();
		if (!fct.isNull()) {
			if (fct->getName() == "List")
				_data.push_back(List::fromExpression(*fct));
			else
				_data.push_back(arg->clone());
		} else {
			_data.push_back(arg->clone());
		}
	}
}

List::~List() {
}

std::list<rw::common::Ptr<const Mathematica::Expression> > List::getArguments() const {
	std::list<rw::common::Ptr<const Mathematica::Expression> > res;
	BOOST_FOREACH(const Expression::Ptr e, _data) {
		res.push_back(e);
	}
	return res;
}

Mathematica::Expression::Ptr List::operator[](std::size_t i) {
	return _data[i];
}

rw::common::Ptr<const Mathematica::Expression> List::operator[](std::size_t i) const {
	return _data[i];
}

List& List::add(Expression::Ptr expression) {
	_data.push_back(expression);
	return *this;
}

List& List::add(const Mathematica::AutoExpression& expression) {
	_data.push_back(expression.expression());
	return *this;
}

void List::set(std::size_t i, Expression::Ptr expression) {
	_data[i] = expression;
}

Mathematica::Expression::Ptr List::clone() const {
	return ownedPtr(new List(_data));
}

List::Ptr List::fromExpression(const Expression& exp) {
	try {
		const FunctionBase& function = dynamic_cast<const FunctionBase&>(exp);
		if (function.getName() != "List")
			RW_THROW("Expression does not appear to be a List.");
		return ownedPtr(new List(function.getArguments()));
	} catch(const std::bad_cast&) {
		RW_THROW("Expression does not appear to be a function (hence it can not be a List).");
	}
}
