#include "PropertyType.hpp"

using namespace rw::common;

int PropertyType::_NextID = (int)BOOL + 1;

int PropertyType::GetNewID()
{
    return _NextID++;
}
