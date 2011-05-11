SET(SRC_CPP UniversalRobots.cpp UniversalRobotsData.cpp)
SET(SRC_HPP UniversalRobots.hpp UniversalRobotsData.hpp)

ADD_LIBRARY(rwhw_universalrobots ${SRC_CPP} ${SRC_HPP})
INSTALL(TARGETS rwhw_universalrobots DESTINATION ${RWHW_LIB_INSTALL_DIR})
set( ENV{ROBWORKHARDWARE_LIBRARIES} "$ENV{ROBWORKHARDWARE_LIBRARIES}rwhw_universalrobots;")
