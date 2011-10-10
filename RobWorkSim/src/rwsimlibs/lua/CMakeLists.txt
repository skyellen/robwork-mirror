
set(Here ${CMAKE_CURRENT_SOURCE_DIR})




# With MSVC++ the program executables go to OutDir.
if (DEFINED MSVC)
  set(LuaCommand ${RW_ROOT}/bin/$(OutDir)/tolua)
else ()
  set(LuaCommand ${RW_ROOT}/bin/tolua)
endif ()

add_custom_command(
  # Because we are building in a temporary directory, we have to use absolute
  # paths here.
  OUTPUT
  ${Here}/RWSimLuaStub.cpp
  ${Here}/RWSimLuaStub.hpp

  COMMAND ${LuaCommand}
  -o ${Here}/RWSimLuaStub.cpp
  -H ${Here}/RWSimLuaStub.hpp
  ${Here}/RWSimLuaWrapper.pkg
  )

ADD_LIBRARY(rwsim_lua
  ${RW_LUA_USER_FILES}
  RWSimLua.cpp
  ${Here}/RWSimLuaStub.cpp)

INSTALL(TARGETS rwsim_lua DESTINATION ${RWSIM_LIB_INSTALL_DIR})