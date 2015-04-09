################################################
#
# IMPORTANT: THIS FILE IS DEPRECATED, USE cmake/RobWorkHardwareConfig instead
#
#################################################

SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

MESSAGE(STATUS "")
MESSAGE(STATUS "DEPRECATION NOTICE: \nThis is the deprecated FindRobWorkHardware script. Please instead use \n SET(RobWorkHardware_DIR RWHWPATH/cmake) \n FIND_PACKAGE(RobWorkHardware ... ) ")
MESSAGE(STATUS "")

# now include the correct setup stuff
SET(RobWork_DIR ${CMAKE_CURRENT_LIST_DIR}/../cmake)
#MESSAGE("RobWork_DIR ${RobWork_DIR}")
FIND_PACKAGE(RobWorkHardware ${RobWorkHardware_FIND_VERSION} REQUIRED NO_MODULE)