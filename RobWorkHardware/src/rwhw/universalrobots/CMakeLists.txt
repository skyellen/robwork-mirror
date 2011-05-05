
INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} ${ESDCAN_INCLUDE_DIR})
ADD_LIBRARY(rwhw_universalrobots UniversalRobots.cpp UnivesalRobotsData.hpp)
INSTALL(TARGETS rwhw_universalrobots DESTINATION ${RWHW_LIB_INSTALL_DIR})

