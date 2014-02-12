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

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
SET(USER_LIBRARIES )
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src/)

ADD_LIBRARY(ex-snippets 
ex-collisions.cpp
ex-constraints.cpp
ex-frame-to-frame-transform.cpp
ex-frame-to-frame-transforms.cpp
ex-get-path-planner.cpp
ex-get-state-path.cpp
ex-grip-frame.cpp
ex-ik.cpp
ex-ik-reachable.cpp
ex-is-daf.cpp
ex-load-task.cpp
ex-load-workcell.cpp
ex-lua-interpreter.cpp
ex-main.cpp
ex-metrics.cpp
ex-owned-ptr.cpp
ex-path-planning.cpp
ex-print-devices.cpp
ex-print-kinematic-tree.cpp
ex-qsampler.cpp
ex-task.cpp
ex-world-transforms.cpp
)

# And now we add any targets that we want
add_executable(ex-main ex-main.cpp)
target_link_libraries(ex-main ex-snippets ${ROBWORK_LIBRARIES} ${USER_LIBRARIES})        

