#########################################################################################
######################## THIS FILE IS DEPRECATED ########################################
#
# Allow the syntax else (), endif (), etc.
#
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

MESSAGE(STATUS "")
MESSAGE(STATUS "DEPRECATION NOTICE: \nThis is the deprecated FindRobWork script. Please instead use \n SET(RobWorkSim_DIR RWSIMPATH/cmake) \n FIND_PACKAGE(RobWorkSim ... ) ")
MESSAGE(STATUS "")
# Get the compiler architecture
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(AMD64 1)
ELSE()
	SET(AMD64 0)
ENDIF()

IF(DEFINED RWSIM_ROOT)
	FILE(TO_CMAKE_PATH ${RWSIM_ROOT} RWSIM_ROOT)
ENDIF()
IF(DEFINED ROBWORKSIM_ROOT)
	FILE(TO_CMAKE_PATH ${ROBWORKSIM_ROOT} ROBWORKSIM_ROOT)
	IF(NOT DEFINED RWSIM_ROOT)
		SET(RWSIM_ROOT "${ROBWORKSIM_ROOT}")
	ENDIF()
ENDIF()

# now include the correct setup stuff
SET(RobWorkSim_DIR ${CMAKE_CURRENT_LIST_DIR}/../cmake)
MESSAGE(" RobWorkSim_DIR ${RobWorkSim_DIR}")
FIND_PACKAGE(RobWorkSim ${RobWorkSim_FIND_VERSION} REQUIRED NO_MODULE)
