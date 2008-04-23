#include "Link.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;
using namespace rw::common;

Link::Link(
    const Entity& entity,
    const value_type& motion_constraint)
   :
    Entity(entity),
    _value(motion_constraint),
    _prev(NULL),
    _next(NULL)
{}
