#include "Link.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;

Link::Link(double tool_speed, SpeedType speed_type)
{
	_tool_speed = tool_speed;
	_speed_type = speed_type;

	_next = _prev = NULL;
}

