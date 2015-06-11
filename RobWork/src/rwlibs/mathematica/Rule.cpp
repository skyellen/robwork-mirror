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

#include "Rule.hpp"
#include "List.hpp"

#include <rw/common/PropertyBase.hpp>
#include <rw/common/PropertyMap.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::mathematica;

Rule::Rule(const Mathematica::Symbol& name, const Mathematica::AutoExpression& value):
	FunctionBase("Rule"),
	_name(ownedPtr(new Mathematica::Symbol(name))),
	_value(value.expression())
{
}

Rule::Rule(const PropertyBase& property):
	FunctionBase("Rule"),
	_name(ownedPtr(new Mathematica::Symbol(property.getIdentifier()))),
	_value(NULL)
{
	if (property.getType().getId() == PropertyType::String) {
		const Property<std::string>& p = dynamic_cast<const Property<std::string>& >(property);
		_value = ownedPtr(new Mathematica::String(p.getValue()));
	} else if (property.getType().getId() == PropertyType::Float) {
		const Property<float>& p = dynamic_cast<const Property<float>& >(property);
		_value = ownedPtr(new Mathematica::Real(p.getValue()));
	} else if (property.getType().getId() == PropertyType::Double) {
		const Property<double>& p = dynamic_cast<const Property<double>& >(property);
		_value = ownedPtr(new Mathematica::Real(p.getValue()));
	} else if (property.getType().getId() == PropertyType::Int) {
		const Property<int>& p = dynamic_cast<const Property<int>& >(property);
		_value = ownedPtr(new Mathematica::Integer(p.getValue()));
	} else if (property.getType().getId() == PropertyType::Bool) {
		const Property<bool>& p = dynamic_cast<const Property<bool>& >(property);
		if (p.getValue())
			_value = ownedPtr(new Mathematica::Symbol("True"));
		else
			_value = ownedPtr(new Mathematica::Symbol("False"));
	} else if (property.getType().getId() == PropertyType::Vector3D) {
		const Property<Vector3D<> >& p = dynamic_cast<const Property<Vector3D<> >& >(property);
		_value = List::toList(p.getValue());
	} else if (property.getType().getId() == PropertyType::Vector2D) {
		const Property<Vector2D<> >& p = dynamic_cast<const Property<Vector2D<> >& >(property);
		_value = List::toList(p.getValue());
	} else if (property.getType().getId() == PropertyType::Q) {
		const Property<Q>& p = dynamic_cast<const Property<Q>& >(property);
		_value = List::toList(p.getValue());
	} else if (property.getType().getId() == PropertyType::StringList) {
		const Property<std::vector<std::string> >& p = dynamic_cast<const Property<std::vector<std::string> >& >(property);
		_value = List::toList(p.getValue());
	} else if (property.getType().getId() == PropertyType::IntList) {
		const Property<std::vector<int> >& p = dynamic_cast<const Property<std::vector<int> >& >(property);
		_value = List::toList(p.getValue());
	} else if (property.getType().getId() == PropertyType::DoubleList) {
		const Property<std::vector<double> >& p = dynamic_cast<const Property<std::vector<double> >& >(property);
		_value = List::toList(p.getValue());
	} else {
		RW_THROW("Option \"" << property.getIdentifier() << "\" is not supported!");
	}
}

Rule::~Rule() {
}

std::list<rw::common::Ptr<const Mathematica::Expression> > Rule::getArguments() const {
	std::list<rw::common::Ptr<const Mathematica::Expression> > res;
	res.push_back(_name);
	res.push_back(_value);
	return res;
}

Mathematica::Expression::Ptr Rule::clone() const {
	return ownedPtr(new Rule(*_name,*_value));
}

std::string Rule::getId() const {
	return _name->getName();
}

void Rule::setValue(const Mathematica::Expression& value) {
	_value = value.clone();
}

std::list<Rule::Ptr> Rule::toRules(const PropertyMap& options) {
	std::list<Rule::Ptr> rules;
	const PropertyMap::Range range = options.getProperties();
	for (PropertyMap::iterator it = range.first; it != range.second; it++) {
		const PropertyBase::Ptr option = *it;
		rules.push_back(ownedPtr(new Rule(*option)));
	}
	return rules;
}

PropertyMap::Ptr Rule::toPropertyMap(const std::list<rw::common::Ptr<const Mathematica::Expression> >& rules) {
	const PropertyMap::Ptr map = ownedPtr(new PropertyMap());
	BOOST_FOREACH(rw::common::Ptr<const Mathematica::Expression> rule, rules) {
		map->add(toProperty(*rule));
	}
	return map;
}

PropertyBase::Ptr Rule::toProperty(const Mathematica::Expression& rule) {
	if (rule.getType() != Mathematica::Expression::Function)
		RW_THROW("Expected function.");
	const Mathematica::FunctionBase& fct = dynamic_cast<const Mathematica::FunctionBase&>(rule);
	if (fct.getName() != "Rule")
		RW_THROW("Expected function with name Rule, not " << fct.getName() << ".");
	const std::list<rw::common::Ptr<const Mathematica::Expression> >& args = fct.getArguments();
	if (args.size() != 2)
		RW_THROW("Expected two arguments for Rule function expression (" << args.size() << " found).");

	// Find name of rule
	rw::common::Ptr<const Mathematica::Expression> nameArg = args.front();
	if (nameArg->getType() != Mathematica::Expression::Symbol)
		RW_THROW("First argument of Rule expression was not a Symbol.");
	const std::string name = nameArg.cast<const Mathematica::Symbol>()->getName();

	// Find value of rule
	rw::common::Ptr<const Mathematica::Expression> valueArg = args.back();
	switch(valueArg->getType()) {
	case Mathematica::Expression::Function:
	{
		RW_THROW("Rules with Function values are only supported for List.");
		break;
	}
	case Mathematica::Expression::Integer:
	{
		const int value = valueArg.cast<const Mathematica::Integer>()->value();
		return ownedPtr(new Property<int>(name, "", value));
	}
	break;
	case Mathematica::Expression::Real:
	{
		const double value = valueArg.cast<const Mathematica::Real>()->value();
		return ownedPtr(new Property<double>(name, "", value));
	}
	break;
	case Mathematica::Expression::String:
	{
		const std::string& value = valueArg.cast<const Mathematica::String>()->value();
		return ownedPtr(new Property<std::string>(name, "", value));
	}
	break;
	case Mathematica::Expression::Symbol:
	{
		const std::string& value = valueArg.cast<const Mathematica::Symbol>()->getName();
		return ownedPtr(new Property<std::string>(name, "", value));
	}
	}

	return NULL;
}
