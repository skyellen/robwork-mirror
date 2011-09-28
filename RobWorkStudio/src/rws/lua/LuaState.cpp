/*
 * LuaState.cpp
 *
 *  Created on: 04/10/2010
 *      Author: jimali
 */

#include "LuaState.hpp"

#include <rwlibs/lua/LuaRobWork.hpp>
#include <rws/lua/LuaRWStudioStub.hpp>
#include <rws/lua/LuaRWStudio.hpp>

extern "C" {
    #include <lua.h>
    #include <lualib.h>
    #include <lauxlib.h>
}

LuaState::LuaState():_rws(NULL),_lua(NULL)
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

void LuaState::addLibrary(AddLibraryCB cb){
    _libraryCBs.push_back(cb);
}

void LuaState::reset(){
    if (_lua!=NULL)
        lua_close(_lua);

    // Open the Lua state.
    _lua = lua_open();
    luaL_openlibs(_lua);

    rwlua::rw::luaRobWork_open(_lua);
    tolua_LuaRWStudio_open(_lua);
    rws::lua::rwstudio::setRobWorkStudio( _rws );
    BOOST_FOREACH(AddLibraryCB &cb, _libraryCBs){
        cb(_lua);
    }

    // add rw and rws namespaces
    runCmd("rw = rwlua.rw");
    runCmd("rws = rws.lua.rwstudio");
    runCmd("rwstudio = rws.getRobWorkStudio()");
}
