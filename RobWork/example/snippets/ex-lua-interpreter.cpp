#include <rwlibs/swig/lua/Lua.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <lua-script>\n";
        return 1;
    }

    lua_State *L = lua_open();
    luaL_openlibs(L);


    rwlibs::swig::openLuaLibRW(L);

    const int error = luaL_dofile(L, argv[1]);
    if (error) std::cerr << lua_tostring(L, -1) << "\n";
    lua_close(L);

    return error;
}
