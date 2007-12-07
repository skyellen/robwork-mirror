
# CMake does not allow a library and executable to have the same name, so we use
# the name lualib for the library.
add_library(lualib
  src/lapi.c
  src/lauxlib.c
  src/lbaselib.c
  src/lcode.c
  src/ldblib.c
  src/ldebug.c
  src/ldo.c
  src/ldump.c
  src/lfunc.c
  src/lgc.c
  src/linit.c
  src/liolib.c
  src/llex.c
  src/lmathlib.c
  src/lmem.c
  src/loadlib.c
  src/lobject.c
  src/lopcodes.c
  src/loslib.c
  src/lparser.c
  src/lstate.c
  src/lstring.c
  src/lstrlib.c
  src/ltable.c
  src/ltablib.c
  src/ltm.c
  src/lundump.c
  src/lvm.c
  src/lzio.c
  src/print.c
  src/lua.c
  src/luac.c)

add_executable(lua src/lua.cpp)
target_link_libraries(lua lualib)

add_executable(luac src/luac.cpp)
target_link_libraries(luac lualib)
