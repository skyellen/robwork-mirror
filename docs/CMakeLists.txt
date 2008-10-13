cmake_minimum_required(VERSION 2.4.4)

# The name of the project.
project(RobWorkDocs)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Enable new linker path policy.
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif ()

set(RobWorkDir ${CMAKE_CURRENT_SOURCE_DIR}/..)

include(${RobWorkDir}/RobWork.cmake OPTIONAL)

link_directories(
  ${RobWorkDir}/libs)

include_directories(
  ${RobWorkDir}/ext
  ${RobWorkDir}/ext/boostbindings
  ${RobWorkDir}/src
  ${RobWorkDir}/ext/lua/src
  ${RobWorkDir}/ext/tolua/include)

if (DEFINED MSVC)
  set(MathLibs lapack_win32 blas_win32)
else ()
  set(MathLibs lapack blas g2c)
endif ()

set(AllLibraries
  rw_algorithms
  rw_drawable
  rw_lua
  rw_pathoptimization
  rw_pathplanners
  rw_proximitystrategies
  rw
  tolualib
  lualib
  yaobi
  ${MathLibs})

add_library(rw_manual_library
  ex-get-path-planner.cpp
  ex-path-planning.cpp
  ex-qsampler.cpp
  ex-constraints.cpp
  ex-collisions.cpp
  ex-metrics.cpp
  ex-grip-frame.cpp
  ex-is-daf.cpp
  ex-get-state-path.cpp
  ex-world-transforms.cpp
  ex-frame-to-frame-transform.cpp
  ex-frame-to-frame-transforms.cpp
  ex-print-devices.cpp
  ex-print-kinematic-tree.cpp
  ex-traverse-task.cpp
  )

add_executable(rw_lua_interpreter ex-lua-interpreter.cpp)
target_link_libraries(rw_lua_interpreter rw_manual_library ${AllLibraries})

add_executable(rw_manual_test main.cpp)
target_link_libraries(rw_manual_test rw_manual_library ${AllLibraries})

add_executable(rw_load_task ex-load-task.cpp)
target_link_libraries(rw_load_task ${AllLibraries})

add_executable(rw_load_workcell ex-load-workcell.cpp)
target_link_libraries(rw_load_workcell ${AllLibraries})

add_executable(rw_ik_reachable ex-ik-reachable.cpp)
target_link_libraries(rw_ik_reachable ${AllLibraries})
