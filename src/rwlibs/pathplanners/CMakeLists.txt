
SET(SRC_FILES
  ./z3/Z3QToQPlanner.cpp
  ./z3/Z3Planner.cpp
  ./prm/PartialIndexTable.cpp
  ./prm/PRMPlanner.cpp
  ./rrt/RRTQToQPlanner.cpp
  ./rrt/RRTPlanner.cpp
  ./arw/ARWExpand.cpp
  ./arw/ARWPlanner.cpp
  ./arw/ARWQToQPlanner.cpp
  ./sbl/SBLExpand.cpp
  ./sbl/SBLInternal.cpp
  ./sbl/SBLSetup.cpp
  ./sbl/SBLOptions.cpp
  ./sbl/SBLPlanner.cpp
)

SET(SRC_FILES_HPP
  ./z3/Z3QToQPlanner.hpp
  ./z3/Z3Planner.hpp
  ./prm/PartialIndexTable.hpp
  ./prm/PRMPlanner.hpp
  ./rrt/RRTQToQPlanner.hpp
  ./rrt/RRTPlanner.hpp
  ./arw/ARWExpand.hpp
  ./arw/ARWPlanner.hpp
  ./arw/ARWQToQPlanner.hpp
  ./sbl/SBLExpand.hpp
  ./sbl/SBLInternal.hpp
  ./sbl/SBLSetup.hpp
  ./sbl/SBLOptions.hpp
  ./sbl/SBLPlanner.hpp
)

ADD_LIBRARY(rw_pathplanners ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_pathplanners DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/pathplanners)
