#include "Target.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;
using namespace rw::common;
using namespace rw::math;

Target::Target(
    const boost::variant<Q, ToolLocation> &value,
    const PropertyMap& properties,
    const std::string& name)
    :
    _value(value),
    _name(name),
    _properties(properties)
{
	_next = _prev = NULL;
}

const Q &Target::getQ() const
{
	assert(isQ());
	return boost::get<Q>(_value);
}

const ToolLocation &Target::getToolLocation() const
{
	assert(isToolLocation());
	return boost::get<ToolLocation >(_value);
}

void Target::setData(const Target &target) 
{
	_value = target._value;
	_name = target._name;
	_properties = target._properties;
}

