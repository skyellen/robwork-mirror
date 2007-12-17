
add_library(rw_examples ex-load-workcell.cpp)

set(RobWorkDir ${CMAKE_CURRENT_SOURCE_DIR}/..)

include_directories(
  ${RobWorkDir}/ext
  ${RobWorkDir}/ext/boostbindings
  ${RobWorkDir}/src
  ${RobWorkDir}/ext/lua/src
  ${RobWorkDir}/ext/tolua/include)
