
set(Here ${CMAKE_CURRENT_SOURCE_DIR})

# With MSVC++ the program executables go to OutDir.
if (DEFINED MSVC)
  set(LuaCommand ${EXECUTABLE_OUTPUT_PATH}/$(OutDir)/tolua)
else ()
  set(LuaCommand ${EXECUTABLE_OUTPUT_PATH}/tolua)
endif ()

add_custom_command(
  # Because we are building in a temporary directory, we have to use absolute
  # paths here.
  OUTPUT
  ${Here}/robwork_stub.cpp
  ${Here}/robwork_stub.hpp

  COMMAND ${LuaCommand}
  -o ${Here}/robwork_stub.cpp
  -H ${Here}/robwork_stub.hpp
  ${Here}/robwork_wrapper.pkg
  DEPENDS ${Here}/robwork_wrapper.pkg)

add_library(rw_lua
  ${RW_LUA_USER_FILES}
  RobWork.cpp
  robwork_wrapper.cpp
  ${Here}/robwork_stub.cpp)
