
set(Here ${CMAKE_CURRENT_SOURCE_DIR})

# With MSVC++ the program executables go to OutDir.
if (DEFINED MSVC)
  set(LuaCommand ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$(OutDir)/tolua)
else ()
  set(LuaCommand ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/tolua)
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
  -n robwork_wrapper
  ${Here}/robwork_wrapper.pkg
  DEPENDS ${Here}/robwork_wrapper.pkg tolua)

SET(SRC_FILES
  ${RW_LUA_USER_FILES}
  RobWork.cpp
  robwork_wrapper.cpp
  ${Here}/robwork_stub.cpp
)

SET(SRC_FILES_HPP
  RobWork.hpp
  robwork_wrapper.hpp
  ${Here}/robwork_stub.hpp
)

ADD_LIBRARY(rw_lua ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_lua DESTINATION lib)
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/lua)
