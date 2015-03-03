#include "Lua.hpp"

#ifdef __cplusplus
extern "C" {
#endif

    int luaopen_rw(lua_State* L); // declare the wrapped module

#ifdef __cplusplus
}
#endif


int rwlibs::swig::openLuaLibRW(lua_State* L){
    return luaopen_rw(L);
}
