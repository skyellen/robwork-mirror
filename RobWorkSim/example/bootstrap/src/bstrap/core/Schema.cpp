#include "Schema.hpp"

void Schema::executeMotorProgram( rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate  ){
    _mp->execute( parameters, bstate);
}

