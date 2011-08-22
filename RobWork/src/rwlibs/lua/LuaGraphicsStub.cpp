/*
** Lua binding: LuaGraphics
** Generated automatically by tolua++-1.0.93 on Fri Aug 19 20:39:08 2011.
*/

#ifndef __cplusplus
#include "stdlib.h"
#endif
#include "string.h"

#include "tolua++.h"

/* Exported function */
TOLUA_API int  tolua_LuaGraphics_open (lua_State* tolua_S);

#include <rw/math.hpp>
#include <rw/graphics.hpp>
#include "LuaGraphics.hpp"

/* function to register type */
static void tolua_reg_types (lua_State* tolua_S)
{
}

/* Open function */
TOLUA_API int tolua_LuaGraphics_open (lua_State* tolua_S)
{
 tolua_open(tolua_S);
 tolua_reg_types(tolua_S);
 tolua_module(tolua_S,NULL,0);
 tolua_beginmodule(tolua_S,NULL);
  tolua_module(tolua_S,"rwlua",0);
  tolua_beginmodule(tolua_S,"rwlua");
   tolua_module(tolua_S,"rw",0);
   tolua_beginmodule(tolua_S,"rw");
   tolua_endmodule(tolua_S);
  tolua_endmodule(tolua_S);

  { /* begin embedded lua code */
   int top = lua_gettop(tolua_S);
   static const unsigned char B[] = {
    13, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45,
     45,32
   };
   tolua_dobuffer(tolua_S,(char*)B,sizeof(B),"tolua: embedded Lua code 1");
   lua_settop(tolua_S, top);
  } /* end of embedded lua code */

 tolua_endmodule(tolua_S);
 return 1;
}


#if defined(LUA_VERSION_NUM) && LUA_VERSION_NUM >= 501
 TOLUA_API int luaopen_LuaGraphics (lua_State* tolua_S) {
 return tolua_LuaGraphics_open(tolua_S);
};
#endif

