#include "Schema.hpp"

void Schema::executeMotorProgram( rw::common::PropertyMap::Ptr parameters, const BrainState& bstate  ){
    _mp->execute( parameters, bstate);
}

