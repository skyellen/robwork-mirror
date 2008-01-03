#include "Target.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;


Target::Target(const boost::variant<rw::math::Q, ToolLocation > &value, const std::string &name) 
  : _name(name), _value(value)
{

	_next = _prev = NULL;
}


Target::~Target()
{

}

rw::math::Q &Target::getQ()
{
	assert(isQ());
	return boost::get<rw::math::Q>(_value);
}


ToolLocation &Target::getToolLocation()
{
	assert(isToolLocation());
	return boost::get<ToolLocation >(_value);
}

