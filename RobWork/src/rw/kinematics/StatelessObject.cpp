/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "StatelessObject.hpp"

using namespace rw::kinematics;

void StatelessObject::registerStateData(rw::kinematics::StateStructure::Ptr statestructure){
    if(statestructure==NULL)
        RW_THROW("Statestructure is null, please specify a valid statestructure");

    if(_statestructure!=NULL && (_statestructure!=statestructure) )
        RW_THROW("Data has allready been added to another statestructure...");
    _statestructure = statestructure;

    for(size_t i=0;i<_stateDatas.size();i++){
        // check if the statedata is allready registrered
        if( !_statestructure->has(_stateDatas[i]) )
            _statestructure->addData(_stateDatas[i]);
    }
}


void StatelessObject::resetStateData(){
    for(size_t i=0;i<_stateDatas.size();i++){
        // check if the statedata is allready registrered
        if( _statestructure->has(_stateDatas[i]) )
            _statestructure->remove(_stateDatas[i]);
        // TODO when we delete this then the state structure will delete them forever...
        // we need to add states to the statestructure without loosing ownership.
    }
}

