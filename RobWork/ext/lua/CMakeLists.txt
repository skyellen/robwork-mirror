SET(LUA_HPP_FILES
  src/lapi.h
  src/lauxlib.h
  src/ldebug.h
  src/ldo.h
  src/lfunc.h
  src/lgc.h
  src/llex.h
  src/lmem.h
  src/lobject.h
  src/lopcodes.h
  src/lparser.h
  src/lstate.h
  src/lstring.h
  src/ltable.h
  src/ltm.h
  src/lundump.h
  src/lvm.h
  src/lzio.h
  src/lua.h
  src/llimits.h
  src/luaconf.h
  src/lualib.h
)


# CMake does not allow a library and executable to have the same name, so we use
# the name lualib for the library.
add_library(lua51
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

if(NOT WIN32)
  add_definitions("-DLUA_USE_POSIX")
  add_definitions("-DLUA_USE_DLOPEN")
elseif(BUILD_SHARED_LIBS)
  add_definitions("-DLUA_BUILD_AS_DLL")
endif()

add_executable(lua src/lua.cpp)
target_link_libraries(lua lua51 ${CMAKE_DL_LIBS})

add_executable(luac src/luac.cpp)
target_link_libraries(luac lua51 ${CMAKE_DL_LIBS})

INSTALL(TARGETS lua luac DESTINATION ${RW_INSTALL_DIR}/bin)
INSTALL(TARGETS lua51 DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${LUA_HPP_FILES} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/lua/")
