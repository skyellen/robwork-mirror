
add_library(
  rw_pathplanners
  ${RW_PATHPLANNERS_USER_FILES}
  ./lazyprm/LazyPRMPathPlanner.cpp
  ./prm/PartialIndexTable.cpp
  ./prm/PRMPlanner.cpp
  ./rrt/RRTPathPlanner.cpp
  ./sbl/SBLInternal.cpp
  ./sbl/SBLPathPlanner.cpp)
