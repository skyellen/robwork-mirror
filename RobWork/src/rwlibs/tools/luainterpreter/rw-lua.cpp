#include <rwlibs/swig/lua/Lua.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <lua-script>\n";
        return 1;
    }

    lua_State *lstate = luaL_newstate();
    luaL_openlibs(lstate);

    rwlibs::swig::openLuaLibRW(lstate);

    const int error = luaL_dofile(lstate, argv[1]);
    if (error) std::cerr << lua_tostring(lstate, -1) << "\n";
    lua_close(lstate);

    return error;
}
