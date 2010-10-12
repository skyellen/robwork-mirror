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

#include "LuaRWStudio.hpp"

#include <rw/common.hpp>

using namespace rws::lua::rwstudio;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

RobWorkStudio::RobWorkStudio(rws::RobWorkStudio* rws):_rws(rws)
{}

void RobWorkStudio::openFile(const std::string& filename){
	_rws->openFile(filename);
}

rw::common::PropertyMap& RobWorkStudio::getPropertyMap(){
	return _rws->getPropertyMap();
}

void RobWorkStudio::setWorkcell(rw::models::WorkCellPtr workcell){
	_rws->setWorkcell( workcell );
}

rwlibs::lua::WorkCell RobWorkStudio::getWorkcell(){
    return rwlibs::lua::WorkCell(_rws->getWorkcell());
}

rw::proximity::CollisionDetector* RobWorkStudio::getCollisionDetector(){
	return _rws->getCollisionDetector();
}

rwlibs::drawable::WorkCellGLDrawer* RobWorkStudio::getWorkCellGLDrawer(){
	return _rws->getWorkCellGLDrawer();
}

const rwlibs::lua::TimedStatePath RobWorkStudio::getTimedStatePath(){
	return _rws->getTimedStatePath();
}

void RobWorkStudio::setTimedStatePath(const rwlibs::lua::TimedStatePath& path){
    _rws->postTimedStatePath(path);
}

void RobWorkStudio::setState(const rwlibs::lua::State& state){
	_rws->postState(state);
}

rwlibs::lua::State RobWorkStudio::getState(){
	return _rws->getState();
}

rw::common::Log& RobWorkStudio::log(){
	return _rws->log();
}

void RobWorkStudio::saveViewGL(const std::string& filename){
	_rws->postSaveViewGL( filename );
}

void RobWorkStudio::updateAndRepaint(){
	_rws->postUpdateAndRepaint();
}

rws::ViewGL* RobWorkStudio::getView(){
	return _rws->getView();
}

RobWorkStudio *rwstudio_internal = NULL;

RobWorkStudio* rws::lua::rwstudio::getRobWorkStudio(){
	return rwstudio_internal;
}

void rws::lua::rwstudio::setRobWorkStudio(rws::RobWorkStudio* rwstudio){
	rwstudio_internal = new RobWorkStudio(rwstudio);
}

