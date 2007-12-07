#include "Target.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;


Target::Target(rw::math::Transform3D<> T, std::string frame)
{
	_value =  std::pair<rw::math::Transform3D<>, std::string>( T,frame);

	_next = _prev = NULL;
}


Target::Target(rw::math::Q q)
{
	_value = q;

}


Target::~Target()
{

}

rw::math::Q Target::Joint()
{
	assert(isJoint());
	return boost::get<JointLocation>(_value);
}


rw::math::Vector3D<> Target::P()
{
	assert(isToolFrame());
	ToolLocation tool_loc = boost::get<ToolLocation >(_value);

	return tool_loc.first.P();

}

rw::math::Transform3D<> Target::Transform3D()
{
	assert(isToolFrame());
	ToolLocation tool_loc = boost::get<ToolLocation >(_value);

	return tool_loc.first;
}

std::string Target::Frame()
{
	assert(isToolFrame());
	ToolLocation tool_loc = boost::get<ToolLocation >(_value);

	return tool_loc.second;
}
