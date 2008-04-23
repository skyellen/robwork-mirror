#include "Target.hpp"

#include <stddef.h>
#include <assert.h>

using namespace rw::task;
using namespace rw::common;
using namespace rw::math;

Target::Target(
    const Entity& entity,
    const Target::value_type& value)
    :
    Entity(entity),
    _value(value)
{
	_next = _prev = NULL;
}
