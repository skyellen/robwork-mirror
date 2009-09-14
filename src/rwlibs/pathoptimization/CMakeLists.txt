
SET(SRC_FILES
  ./clearance/ClearanceOptimizer.cpp
  ./clearance/ClearanceCalculator.cpp
  ./clearance/MinimumClearanceCalculator.cpp
  ./pathlength/PathLengthOptimizer.cpp
)

SET(SRC_FILES_HPP
  ./clearance/ClearanceOptimizer.hpp
  ./clearance/ClearanceCalculator.hpp
  ./clearance/MinimumClearanceCalculator.hpp
  ./pathlength/PathLengthOptimizer.hpp
)

ADD_LIBRARY(rw_pathoptimization ${SRC_FILES} ${SRC_FILES_HPP})
 
INSTALL(TARGETS rw_pathoptimization DESTINATION ${RW_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_HPP} DESTINATION include/rwlibs/pathoptimization)
  