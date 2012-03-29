/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWLIBS_SWIG_LUA_HPP_
#define RWLIBS_SWIG_LUA_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>


#ifdef __cplusplus
}
#endif

namespace rwlibs {
namespace swig {

    /**
     * @brief initialize a lua state
     * @param L
     * @return
     */
    int openLuaLibRW(lua_State* L);

}
}
#endif /* LUA_HPP_ */
