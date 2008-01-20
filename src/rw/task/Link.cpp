#include "Link.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;

Link::Link(const std::string &name)
    :
    _motion_constraint(NoConstraint()),
    _name(name)
{
	_next = _prev = NULL;
}

Link::Link(const MotionConstraint &motion_constraint, const std::string &name)
   :
    _motion_constraint(motion_constraint),
    _name(name)
{
	_next = _prev = NULL;
}

Link::~Link()
{}

void Link::setData(const Link &link) 
{
	_motion_constraint = link._motion_constraint;
	_name = link._name;
	_properties = link._properties;

	_solved_path = link._solved_path;
}

