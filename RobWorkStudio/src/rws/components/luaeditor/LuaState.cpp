/*
 * LuaState.cpp
 *
 *  Created on: 04/10/2010
 *      Author: jimali
 */

#include "LuaState.hpp"

#include <rws/lua/LuaRWStudioStub.hpp>
#include <rws/lua/LuaRWStudio.hpp>

extern "C" {
    #include <lua.h>
    #include <lualib.h>
    #include <lauxlib.h>
}

int LuaState::runCmd(const std::string& cmd){
    int error = luaL_loadbuffer(_lua.get(), cmd.c_str(), cmd.size(), "");
    if (!error)
        error = lua_pcall(_lua.get(), 0, 0, 0);
    return error;
}

void LuaState::reset(){
    if (_lua!=NULL)
        lua_close(_lua.get());

    // Open the Lua state.
    _lua = lua_open();
    luaL_openlibs(_lua.get());
    rwlibs::lua::luaRobWork_open(_lua.get());
    tolua_LuaRWStudio_open(_lua.get());
    rws::lua::rwstudio::setRobWorkStudio( _rws );

    // add rw and rws namespaces
    runCmd("rw = rwlibs.lua");
    runCmd("rws = rws.lua.rwstudio");
    runCmd("rwstudio = rws.getRobWorkStudio()");
}
