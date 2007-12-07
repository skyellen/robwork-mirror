#include "PropertyType.hpp"


using namespace rw::common;

int PropertyType::_NextID = (int)BOOL+1;

//PropertyType PropertyType::STRING(



int PropertyType::GetNewID() {
    return _NextID++;
}
