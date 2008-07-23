
add_library(
  rw_pathplanners
  ${RW_PATHPLANNERS_USER_FILES}
  ./prm/PartialIndexTable.cpp
  ./prm/PRMPlanner.cpp
  ./rrt/RRTQToQPlanner.cpp
  ./sbl/SBLInternal.cpp
  ./sbl/SBLSetup.cpp
  ./sbl/SBLOptions.cpp
  ./sbl/SBLPlanner.cpp)
