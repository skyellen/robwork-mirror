
set(Here ${CMAKE_CURRENT_SOURCE_DIR})

if (DEFINED MSVC)
  add_custom_command(
    # Because we are building in a temporary directory, we have to use absolute
    # paths here.
    OUTPUT
    ${Here}/robwork_stub.cpp
    ${Here}/robwork_stub.hpp

    COMMAND ${EXECUTABLE_OUTPUT_PATH}/$(OutDir)/tolua
    -o ${Here}/robwork_stub.cpp
    -H ${Here}/robwork_stub.hpp
    ${Here}/robwork_wrapper.pkg
    DEPENDS ${Here}/robwork_wrapper.pkg)
else ()
  add_custom_command(
    # Because we are building in a temporary directory, we have to use absolute
    # paths here.
    OUTPUT
    ${Here}/robwork_stub.cpp
    ${Here}/robwork_stub.hpp

    COMMAND ${EXECUTABLE_OUTPUT_PATH}/tolua
    -o ${Here}/robwork_stub.cpp
    -H ${Here}/robwork_stub.hpp
    ${Here}/robwork_wrapper.pkg
    DEPENDS ${Here}/robwork_wrapper.pkg)
endif ()

add_library(rw_lua
 ${RW_LUA_USER_FILES} 
 RobWork.cpp 
 robwork_wrapper.cpp 
 ${Here}/robwork_stub.cpp)
