#include "StateData.hpp"

#include "State.hpp"
#include <rw/common/macros.hpp>

using namespace rw::kinematics;

StateData::StateData(int size, const std::string& name):
    _id(-1),
    _size(size),
    _name(name)
{
    RW_ASSERT(0 <= size);
}
