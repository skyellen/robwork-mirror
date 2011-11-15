

#include "ScriptTypes.hpp"


rw::common::Ptr<rws::swig::RobWorkStudio> rwstudio_internal;

rws::swig::RobWorkStudio* rws::swig::getRobWorkStudio(){
    return rwstudio_internal.get();
}

void rws::swig::setRobWorkStudio(rws::swig::RobWorkStudio* rwstudio){
    rwstudio_internal = rwstudio;
}
