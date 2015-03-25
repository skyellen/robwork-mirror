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

#include "Simulator.hpp"

using namespace rwlibs::simulation;

rw::sensor::Sensor::Ptr Simulator::getSensorHandle(SimulatedSensor* ssensor, const std::string& type){
	if(_simsensorToHandle.find(std::make_pair(ssensor,type))==_simsensorToHandle.end()){
		return NULL;
	}
	return _simsensorToHandle[std::make_pair(ssensor,type)];
}

bool Simulator::hasHandle(SimulatedSensor* ssensor, const std::string& type){
	if(_simsensorToHandle.find(std::make_pair(ssensor,type))==_simsensorToHandle.end()){
		return false;
	}
	return true;
}

void Simulator::addHandle(SimulatedSensor* ssensor, const std::string& type, rw::sensor::Sensor::Ptr sensor){
	_simsensorToHandle[std::make_pair(ssensor,type)] = sensor;
}
