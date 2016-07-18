#####################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source 
# files and when configuring your project.  
#####################################################


# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. (EDIT THIS)
PROJECT(ExampleSnippets)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
SET(USER_LIBRARIES )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/)

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

ADD_LIBRARY(ex-snippets 
ex-collisions.cpp
ex-constraints.cpp
ex-frame-to-frame-transform.cpp
ex-frame-to-frame-transforms.cpp
ex-get-path-planner.cpp
ex-get-state-path.cpp
ex-grip-frame.cpp
ex-ik.cpp
ex-is-daf.cpp
ex-metrics.cpp
ex-owned-ptr.cpp
ex-path-planning.cpp
ex-print-devices.cpp
ex-print-kinematic-tree.cpp
ex-qsampler.cpp
ex-world-transforms.cpp
)

MESSAGE("linking: ${ROBWORK_LIBRARIES}")

# And now we add any targets that we want
add_executable(ex-main ex-main.cpp)
target_link_libraries(ex-main ex-snippets ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})        

add_executable(ex-ik-reachable ex-ik-reachable.cpp)
target_link_libraries(ex-ik-reachable ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-load-task ex-load-task.cpp)
target_link_libraries(ex-load-task ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-load-workcell ex-load-workcell.cpp)
target_link_libraries(ex-load-workcell ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-lua-interpreter ex-lua-interpreter.cpp)
target_link_libraries(ex-lua-interpreter ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-task ex-task.cpp)
target_link_libraries(ex-task ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-threadpool ex-threadpool.cpp)
target_link_libraries(ex-threadpool ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})

add_executable(ex-threadtask ex-threadtask.cpp)
target_link_libraries(ex-threadtask ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})