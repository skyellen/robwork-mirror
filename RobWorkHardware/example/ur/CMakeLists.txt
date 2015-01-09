cmake_minimum_required(VERSION 2.6.0)
PROJECT(UR)

#SET(CMAKE_BUILD_TYPE Release)
#UNSET(Boost_USE_STATIC_LIBS)
#UNSET(Boost_FIND_QUIETLY)
#find_package(Boost 1.40.0 REQUIRED COMPONENTS asio)

#Include default settings for constructing a robwork dependent project
MESSAGE("$ENV{RWHW_ROOT}/cmake")
SET(RobWorkHardware_DIR "$ENV{RWHW_ROOT}/cmake")

FIND_PACKAGE(RobWorkHardware COMPONENTS universalrobots REQUIRED HINTS "../../cmake")

find_package(Threads REQUIRED)

INCLUDE_DIRECTORIES(${ROBWORKHARDWARE_INCLUDE_DIRS})
LINK_DIRECTORIES( ${ROBWORKHARDWARE_LIBRARY_DIRS})

add_executable(ur simpleTest.cpp )
#message("testmessage: ${ROBWORKHARDWARE_LIBRARY_DIRS}")
#message("RWHW_ROOT: ${RWHW_ROOT}")
#message("LIBS: ${ROBWORK_LIBRARIES}")
message("${ROBWORKHARDWARE_LIBRARIES}")
TARGET_LINK_LIBRARIES(ur ${ROBWORKHARDWARE_LIBRARIES} ${ROBWORK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

