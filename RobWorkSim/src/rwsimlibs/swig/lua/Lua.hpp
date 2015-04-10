/*
 * Lua.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: jimali
 */

#ifndef RWSIM_SWIG_LUA_HPP_
#define RWSIM_SWIG_LUA_HPP_


#ifdef __cplusplus
extern "C" {
#endif

#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#ifdef __cplusplus
}
#endif

namespace rwsim {
namespace swig {

    /**
     * @brief initialize a lua state
     * @param L
     * @return
     */
    int openLuaLibRWSim(lua_State* L);

}
}
#endif /* LUA_HPP_ */
