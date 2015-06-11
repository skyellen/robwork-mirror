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

#include "ListPlot.hpp"
#include "List.hpp"
#include "Rule.hpp"

#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

ListPlot::ListPlot():
	Mathematica::FunctionBase("ListPlot")
{
}

ListPlot::ListPlot(const Expression& data):
	Mathematica::FunctionBase("ListPlot")
{
	_data = data.clone();
}

ListPlot::ListPlot(const Expression& data, const std::list<Rule::Ptr>& options):
	Mathematica::FunctionBase("ListPlot")
{
	_data = data.clone();
	_options = options;
}

ListPlot::ListPlot(const std::vector<double>& x, const std::vector<double>& y, const std::list<Rule::Ptr>& options):
	Mathematica::FunctionBase("ListPlot")
{
	RW_ASSERT(x.size() == y.size());
	const List::Ptr list = ownedPtr(new List());
	for (std::size_t i = 0; i < x.size(); i++) {
		const List::Ptr innerList = ownedPtr(new List());
		innerList->add(x[i]);
		innerList->add(y[i]);
		list->add(innerList);
	}
	_data = list;
	_options = options;
}

ListPlot::~ListPlot() {
}

std::list<rw::common::Ptr<const Mathematica::Expression> > ListPlot::getArguments() const {
	std::list<rw::common::Ptr<const Expression> > res;
	res.push_back(_data);
	BOOST_FOREACH(const Expression::Ptr option, _options) {
		res.push_back(option);
	}
	return res;
}

Mathematica::Expression::Ptr ListPlot::clone() const {
	ListPlot::Ptr clone = ownedPtr(new ListPlot(*_data));
	clone->_options = _options;
	return clone;
}

void ListPlot::option(const std::string& name, const Mathematica::Expression& value) {
	Rule::Ptr option = NULL;
	BOOST_FOREACH(const Rule::Ptr o, _options) {
		if (o->getId() == name) {
			option = o;
			break;
		}
	}
	if (!option.isNull())
		option->setValue(value);
	else
		_options.push_back(ownedPtr(new Rule(name,value)));
}

void ListPlot::setImageSize(int width, int height) {
	List size;
	size.add(width);
	size.add(height);
	Rule::Ptr option = NULL;
	BOOST_FOREACH(const Rule::Ptr o, _options) {
		if (o->getId() == "ImageSize") {
			option = o;
			break;
		}
	}
	if (!option.isNull())
		option->setValue(size);
	else
		_options.push_back(ownedPtr(new Rule("ImageSize",size)));
}

ListPlot ListPlot::fromExpression(const Mathematica::Expression& expression) {
	ListPlot plot;
	try {
		const Mathematica::FunctionBase& fct = dynamic_cast<const Mathematica::FunctionBase&>(expression);
		if (fct.getName() != "ListPlot")
			RW_THROW("ListPlot could not be constructed from function of name \"" << fct.getName() << "\"!");
		RW_THROW("ListPlot could not be constructed - not yet implemented!");
	} catch(const std::bad_cast& e) {
		RW_THROW("ListPlot could not be constructed from expression as it is not a Function expression!");
	}
	return plot;
}
