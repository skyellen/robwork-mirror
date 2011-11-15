#include "Lua.hpp"

int rwlibs::swig::openLuaLibRW(lua_State* L){
    return luaopen_rw(L);
}
