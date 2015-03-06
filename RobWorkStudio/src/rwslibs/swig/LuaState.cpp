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

#include "LuaState.hpp"

#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwlibs/swig/lua/Lua.hpp>
#include "ScriptTypes.hpp"
#include "Lua.hpp"

using namespace rws;

LuaState::LuaState():_lua(NULL),_rws(NULL)
{}

LuaState::~LuaState(){
    if(_lua!=NULL)
        lua_close(_lua);
    _lua = NULL;
}

int LuaState::runCmd(const std::string& cmd){
    int error = luaL_loadbuffer(_lua, cmd.c_str(), cmd.size(), "");
    if (!error)
        error = lua_pcall(_lua, 0, 0, 0);
    return error;
}

void LuaState::addLibrary(LuaLibrary::Ptr lib){
    _libraryCBs.push_back( lib );
}

void LuaState::removeLibrary(const std::string& id){
    int idx=-1;
	for(std::size_t i=0;i<_libraryCBs.size();i++){
		if(_libraryCBs[i]->getId()==id){
			idx = (int)i;
		}
	}
	if(idx<0){
		return;
	}
	_libraryCBs.erase(_libraryCBs.begin()+idx);
	reset();
}


void LuaState::reset(){
    if (_lua!=NULL)
        lua_close(_lua);

    // Open the Lua state.
    _lua = lua_open();
    luaL_openlibs(_lua);

    rwlibs::swig::openLuaLibRW( _lua );

    rwslibs::swig::openLuaLibRWS( _lua );

    rws::swig::setRobWorkStudio( _rws );

    BOOST_FOREACH(LuaLibrary::Ptr cb, _libraryCBs){
        cb->initLibrary( *this );
    }

    // add rw and rws namespaces
    runCmd("rw = rwlua.rw");
    runCmd("rws = rws.lua.rwstudio");
    runCmd("rwstudio = rws.getRobWorkStudio()");
}
