set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

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

set(AllLibraries
  rw_algorithms
  rw_devices
  rw_drawable
  rw_io
  rw_lua
  rw_pathoptimization
  rw_pathplanners
  rw_proximitystrategies
  rw_sensors
  rw
  tolualib
  lualib
  pqp
  opcode
  lapack
  blas
  g2c)

add_library(rw_manual_library
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

add_executable(rw_manual_test main.cpp)
target_link_libraries(rw_manual_test rw_manual_library ${AllLibraries})

add_executable(rw_load_task ex-load-task.cpp)
target_link_libraries(rw_load_task ${AllLibraries})

add_executable(rw_load_workcell ex-load-workcell.cpp)
target_link_libraries(rw_load_workcell ${AllLibraries})
