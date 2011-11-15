/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef LUA_HPP_
#define LUA_HPP_

#ifdef __cplusplus
extern "C" {
#endif
    #include <lua.h>
    #include <lualib.h>
    #include <lauxlib.h>

    int luaopen_rw(lua_State* L); // declare the wrapped module

#ifdef __cplusplus
}
#endif


namespace rwlibs {
namespace swig {

    int openLuaLibRW(lua_State* L);

}
}
#endif /* LUA_HPP_ */
