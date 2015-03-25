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

#include "Scanner2DModel.hpp"

using namespace rw::sensor;
using namespace rw::kinematics;

Scanner2DModel::Scanner2DModel(const std::string& name, double angleRange, int width, rw::kinematics::Frame* frame )
	: SensorModel(name, frame),_sstate(1, rw::common::ownedPtr( new Scanner2DModelCache(width)).cast<StateCache>() ),
	  _width(width),_angleRange(angleRange)
{
	add(_sstate);
}


Scanner2DModel::~Scanner2DModel(){
}

rw::geometry::PointCloud& Scanner2DModel::getScan(const rw::kinematics::State& state){
	return _sstate.getStateCache<Scanner2DModelCache>(state)->_cloud;
}

void Scanner2DModel::setScan(const rw::geometry::PointCloud& data, const rw::kinematics::State& state){
	_sstate.getStateCache<Scanner2DModelCache>(state)->_cloud = data;
}
