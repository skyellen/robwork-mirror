set ROOT=%1

%ROOT%\tolua -o ../../../../src/rws/lua/LuaRWStudioStub.cpp -H ../../../../src/rws/lua/LuaRWStudioStub.hpp -n LuaRWStudio ../../../../src/rws/lua/LuaRWStudio.pkg
