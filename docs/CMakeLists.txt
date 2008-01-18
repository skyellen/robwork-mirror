
set(RobWorkDir ${CMAKE_CURRENT_SOURCE_DIR}/..)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

include(${RobWorkDir}/RobWork.cmake OPTIONAL)

link_directories(${RobWorkDir}/libs)

include_directories(
  ${RobWorkDir}/ext
  ${RobWorkDir}/ext/boostbindings
  ${RobWorkDir}/src
  ${RobWorkDir}/ext/lua/src
  ${RobWorkDir}/ext/tolua/include)

set(AllLibraries
  rw_lua
  tolualib
  lualib
  rw_proximitystrategies
  rw_pathplanners
  rw
  pqp
  opcode
  lapack
  blas
  g2c
  # rw_algorithms
  # rw_devices
  # rw_distancestrategies
  # rw_drawable
  # rw_io
  # rw_sensors
  )

add_library(rw_manual_library
  ex-print-devices.cpp
  ex-print-kinematic-tree
  )

add_executable(rw_manual_test main.cpp)
target_link_libraries(rw_manual_test rw_manual_library ${AllLibraries})

add_executable(rw_load_workcell ex-load-workcell.cpp)
target_link_libraries(rw_load_workcell ${AllLibraries})
