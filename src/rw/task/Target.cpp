#include "Target.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;

Target::Target(
    const boost::variant<rw::math::Q, ToolLocation> &value,
    const std::string &name)
    :
    _value(value),
    _name(name)
{
	_next = _prev = NULL;
}

Target::~Target() {}

const rw::math::Q &Target::getQ() const
{
	assert(isQ());
	return boost::get<rw::math::Q>(_value);
}

const ToolLocation &Target::getToolLocation() const
{
	assert(isToolLocation());
	return boost::get<ToolLocation >(_value);
}
