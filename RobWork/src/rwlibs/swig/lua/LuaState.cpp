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

#include "Lua.hpp"

using namespace rwlibs::swig;

LuaState::LuaState():_lua(NULL)
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

    BOOST_FOREACH(LuaLibrary::Ptr cb, _libraryCBs){
        cb->initLibrary( *this );
    }

    // get extension point libs
    std::vector<LuaLibrary::Ptr> libs = LuaState::Factory::getLuaLibraries();
    BOOST_FOREACH(LuaLibrary::Ptr cb, libs){
        cb->initLibrary( *this );
    }


    // add rw and rws namespaces
    runCmd("rw = rwlua.rw");
}


std::vector<LuaState::LuaLibrary::Ptr> LuaState::Factory::getLuaLibraries(){
	using namespace rw::common;
	LuaState::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	std::vector<LuaState::LuaLibrary::Ptr> libs;
	BOOST_FOREACH(Extension::Ptr ext, exts){
		// else try casting to ImageLoader
		LuaState::LuaLibrary::Ptr lib = ext->getObject().cast<LuaState::LuaLibrary>();
		libs.push_back(lib);
	}
	return libs;
}

std::vector<std::string> LuaState::Factory::getLuaLibraryIDs(){
	using namespace rw::common;
	LuaState::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	std::vector<std::string> libs;
	BOOST_FOREACH(Extension::Ptr ext, exts){
		// else try casting to ImageLoader
		//LuaState::LuaLibrary::Ptr lib = ext->getObject().cast<LuaState::LuaLibrary>();
		libs.push_back( ext->getId() );
	}
	return libs;
}

LuaState::LuaLibrary::Ptr LuaState::Factory::getLuaLibrary(const std::string& id){
	using namespace rw::common;
	LuaState::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(ext->getId()!=id)
			continue;
		LuaState::LuaLibrary::Ptr lib = ext->getObject().cast<LuaState::LuaLibrary>();
		return lib;
	}
	return NULL;
}

