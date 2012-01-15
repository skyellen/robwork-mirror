

#include "ScriptTypes.hpp"


rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc_internal;

rwsim::dynamics::DynamicWorkCell* rwsim::swig::getDynamicWorkCell(){
    return dwc_internal.get();
}

void rwsim::swig::setDynamicWorkCell(rwsim::swig::DynamicWorkCell* dwc){
    dwc_internal = dwc;
}

