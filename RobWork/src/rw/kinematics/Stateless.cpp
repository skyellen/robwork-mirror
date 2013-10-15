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


#include "Stateless.hpp"

using namespace rw::kinematics;

void Stateless::registerIn(State& state){
	registerIn( state.getStateStructure() );
	state.upgrade();
}

void Stateless::registerIn(StateStructure::Ptr state){
	if(_registered)
		RW_THROW("Stateless Object hass allready been initialized to another state!");
	std::cout << "Registering all state datas.... "  << _datas.size() << std::endl;
	for(int i=0;i< (int) _datas.size(); i++){
	    std::cout << "Reg: " << _datas[i]->getName() << " " << _datas[i]->getID() << std::endl;
		state->addData( _datas[i] );
	}
	_stateStruct = state;
	_registered = true;
}

void Stateless::unregister(){
	if(!_registered)
		return;
	for(size_t i=0;i<_datas.size(); i++){
		_stateStruct->remove( _datas[i].get() );
	}
	_registered = false;
	_stateStruct = NULL;
}
