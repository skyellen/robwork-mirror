
add_library(
  rw_pathplanners
  ${RW_PATHPLANNERS_USER_FILES}
  ./lazyprm/LazyPRMPathPlanner.cpp
  ./rrt/RRTPathPlanner.cpp)
