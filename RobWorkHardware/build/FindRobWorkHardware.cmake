# Find and sets up RobWorkHardware
#
#  Output variables:
#    ROBWORKHARDWARE_INCLUDE_DIR  - Where to find robwork include sub-directory.
#    ROBWORKHARDWARE_LIBRARIES    - List of libraries when using RobWork (includes all libraries that RobWork depends on).
#    ROBWORKHARDWARE_LIBRARY_DIRS - List of directories where libraries of RobWork are located. 
#    ROBWORKHARDWARE_FOUND        - True if RobWork was found. (not fully implemented yet)
#
#    ROBWORKHARDWARE_ROOT         - If set this defines the root of RobWorkHardware if not set then it
#                                   if possible be autodetected.
#
#  Input variables:
#    RobWorkHardware_FIND_COMPONENTS - List of required RWHW components to search for (see list below).
#                                      If REQUIRED is set then a fatal error is reported if these are not found. 
#                                       else only a warning is issues. 
#
#  Components:
#    camera
#	 can
#	 crsa465
#	 dockwelder
#	 fanucdevice
#	 katana
#	 motomanIA20
#    netft
#	 pa10
#	 pcube
#    schunkpg70
#	 sdh
#	 serialport
#	 sick
#	 swissranger
#	 tactile
#    trakstar
#    universalrobots

# Allow the syntax else (), endif (), etc.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Indicate whether a fatal error happened that should render ROBWORKHARDWARE_FOUND to be false
SET(ROBWORKHARDWARE_EXPERIENCED_FATAL_PROBLEMS FALSE)

# Check if RW_ROOT path are setup correctly
#FIND_FILE(ROBWORKHARDWARE_FOUND FindRobWorkHardware.cmake ${ROBWORKHARDWARE_ROOT}/build NO_DEFAULT_PATH)

FIND_FILE(RWHW_ROOT_PATH_TEST FindRobWorkHardware.cmake
	"${RWHW_ROOT}/build/"
	"${ROBWORKHARDWARE_ROOT}/build/"
	"../build/"
	"../RobWorkHardware/build/"
	"../../RobWorkHardware/build/"
	"c:/program files/RobWork/build/"
	"c:/programmer/RobWork/build/"
)

IF(NOT RWHW_ROOT_PATH_TEST)
  SET(ROBWORKHARDWARE_EXPERIENCED_FATAL_PROBLEMS TRUE)
    MESSAGE(FATAL_ERROR "Path to RobWorkHardware root (ROBWORKHARDWARE_ROOT) is incorrectly setup! \nROBWORKHARDWARE_ROOT  ==${ROBWORKHARDWARE_ROOT}")
ENDIF()

GET_FILENAME_COMPONENT(ROBWORKHARDWARE_ROOT_TMP ${RWHW_ROOT_PATH_TEST} PATH)
SET(ROBWORKHARDWARE_ROOT "${ROBWORKHARDWARE_ROOT_TMP}/../")
SET(RWHW_ROOT "${ROBWORKHARDWARE_ROOT_TMP}/../")

LIST(APPEND CMAKE_MODULE_PATH ${ROOT}/build ${RW_ROOT}/build ${RWHW_ROOT}/build)

INCLUDE(${RWHW_ROOT}/build/RobWorkHardwareBuildConfig${CMAKE_BUILD_TYPE}.cmake)
MESSAGE("components ${RobWorkHardware_FIND_COMPONENTS}")

SET(LIBRARIES_TO_INCLUDE ) # Libraries that must be included
SET(OPTIONAL_LIBRARIES_TO_INCLUDE ) # libraries that are included if they have been build
IF(RobWorkHardware_FIND_COMPONENTS)
    # FIRST check if all required components are installed/build
    FOREACH(component IN LISTS RobWorkHardware_FIND_COMPONENTS)
        
        LIST(FIND RWH_BUILD_WITH_LIBRARIES "rwhw_${component}" COMP_FOUND)
        IF(${COMP_FOUND} GREATER -1)
            LIST(APPEND LIBRARIES_TO_INCLUDE "rwhw_${component}")
        ELSE()
            IF( RobWorkHardware_REQUIRED )
	      SET(ROBWORKHARDWARE_EXPERIENCED_FATAL_PROBLEMS TRUE)
              MESSAGE(FATAL_ERROR "The component: rwhw_${component} has not been built with RobWorkHardware. Reconfigure RobWorkHardware installation or check component spelling!")
            ELSE ()
              MESSAGE(WARNING "The component: rwhw_${component} has not been built with RobWorkHardware. Reconfigure RobWorkHardware installation or check component spelling!")
            ENDIF()
        ENDIF()
    ENDFOREACH()
    MESSAGE(" ${LIBRARIES_TO_INCLUDE} ")
ELSE()
    SET(OPTIONAL_LIBRARIES_TO_INCLUDE ${RWH_BUILD_WITH_LIBRARIES})
ENDIF()    

FOREACH(component IN LISTS LIBRARIES_TO_INCLUDE)
    LIST(APPEND OPTIONAL_LIBRARIES_TO_INCLUDE ${component})
ENDFOREACH()

# Setup the libraries
IF (RWHARDWARE_BUILD_WITH_SANDBOX)
#  SET(ROBWORKHARDWARE_SANDBOX_LIB rwhw_sandbox)
  SET(ROBWORKHARDWARE_HAVE_SANDBOX ON)
ELSE ()
  SET(ROBWORKHARDWARE_HAVE_SANDBOX OFF)
ENDIF ()

SET(ROBWORKHARDWARE_INCLUDE_DIRS ${ROBWORKHARDWARE_ROOT}/src/)
SET(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_SANDBOX_LIB} )

#SDH
LIST(FIND OPTIONAL_LIBRARIES_TO_INCLUDE "rwhw_sdh" USE_SDH)
IF(${USE_SDH} GREATER -1)
    INCLUDE(${ROBWORKHARDWARE_ROOT}/build/FindSDH.cmake)
    IF(SDH_FOUND) #AND RAW1394_FOUND)
    	MESSAGE(STATUS "RobWork Hardware: component rwhw_sdh is included!")
    	LIST(APPEND ROBWORKHARDWARE_LIBRARIES rwhw_sdh ${SDH_LIBRARY})
    	LIST(APPEND ROBWORKHARDWARE_INCLUDE_DIRS ${SDH_INCLUDE_DIR}) 
    ELSE()
        MESSAGE(STATUS "RobWork Hardware: component rwhw_sdh is NOT included!")
        LIST(FIND LIBRARIES_TO_INCLUDE "rwhw_sdh" FORCE_SDH)
        IF(FORCE_SDH)
            MESSAGE(SEND_ERROR "SDH requirements could not be resolved!")
        ENDIF()
    ENDIF()
ENDIF()

MACRO (RWHW_ADD_INTERNAL_LIBRARY library)
    LIST(FIND OPTIONAL_LIBRARIES_TO_INCLUDE ${library} USE_LIB)
    #MESSAGE("${OPTIONAL_LIBRARIES_TO_INCLUDE} --- ${library}")
    #MESSAGE("${USE_LIB}") 
    IF(${USE_LIB} GREATER -1)
	#MESSAGE("   ${RWH_BUILD_WITH_LIBRARIES}")
        LIST(FIND RWH_BUILD_WITH_LIBRARIES ${library} HAS_LIB)
        IF(${HAS_LIB} GREATER -1) #AND RAW1394_FOUND)
        	MESSAGE(STATUS "RobWork Hardware: component ${library} is included!")
        	LIST(APPEND ROBWORKHARDWARE_LIBRARIES ${library})
        	SET(${library}_FOUND TRUE)
        ELSE()
            MESSAGE(STATUS "RobWork Hardware: component ${library} is NOT included!")
            SET(${library}_FOUND FALSE)
        ENDIF()
    ENDIF()
ENDMACRO (RWHW_ADD_INTERNAL_LIBRARY)

MACRO (RWHW_REMOVE_INTERNAL_LIBRARY library)
    LIST(REMOVE_ITEM ROBWORKHARDWARE_LIBRARIES ${library})
    SET(${library}_FOUND FALSE)
ENDMACRO (RWHW_REMOVE_INTERNAL_LIBRARY)


RWHW_ADD_INTERNAL_LIBRARY("rwhw_schunkpg70")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_pcube")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_serialport")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_dockwelder")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_fanucdevice")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_motomanIA20")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_pa10")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_sick")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_swissranger")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_tatile")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_can")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_universalrobots")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_netft")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_trakstar")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_robolabFT")
RWHW_ADD_INTERNAL_LIBRARY("rwhw_robotiq")


IF( rwhw_trakstar_FOUND )
    FIND_PACKAGE(TrakStar)
    IF( TRAKSTAR_FOUND )
        LIST(APPEND ROBWORKHARDWARE_INCLUDE_DIRS ${TRAKSTAR_INCLUDE_DIR})
    ELSE()
        RWHW_REMOVE_INTERNAL_LIBRARY("rwhw_trakstar")
    ENDIF()
ENDIF()

#IEICAN
LIST(FIND OPTIONAL_LIBRARIES_TO_INCLUDE "rwhw_can" USE_CAN)
IF(${USE_CAN} GREATER -1)
    IF (CMAKE_COMPILER_IS_GNUCXX)
        IF (DEFINED MINGW)
            # TODO mingw32 libraries
    		INCLUDE(${ROBWORKHARDWARE_ROOT}/build/FindIEICAN.cmake)
    		IF(IEICAN_FOUND)
    			MESSAGE(STATUS "RobWork Hardware: component rwhw_can - IEICAN is included!")
    			ADD_DEFINITIONS(-DRWHW_HAS_IEICAN)
    			SET(ROBWORKHARDWARE_INCLUDE_DIRS  ${ROBWORKHARDWARE_INCLUDE_DIRS} ${IEICAN_INCLUDE_DIR})
    			SET(ROBWORKHARDWARE_LIBRARIES     ${ROBWORKHARDWARE_LIBRARIES} rwhw_can )
    		ELSE()
    			MESSAGE(STATUS "RobWork Hardware: component rwhw_can - IEICAN is NOT included!")
    		ENDIF()
        ELSE()
            MESSAGE(STATUS "RobWork Hardware: component rwhw_can - IEICAN is NOT included!")
        ENDIF()
    ELSEIF (DEFINED MSVC)
        # TODO MSVC
    	INCLUDE(${ROBWORKHARDWARE_ROOT}/build/FindIEICAN.cmake)
    	IF(IEICAN_FOUND)
    		MESSAGE(STATUS "RobWork Hardware: component rwhw_can - IEICAN is NOT included!")
    		ADD_DEFINITIONS(-DRWHW_HAS_IEICAN)
    		SET(ROBWORKHARDWARE_INCLUDE_DIRS  ${ROBWORKHARDWARE_INCLUDE_DIRS} ${IEICAN_INCLUDE_DIR})
    		SET(ROBWORKHARDWARE_LIBRARIES     ${ROBWORKHARDWARE_LIBRARIES} rwhw_can )
    	ELSE()
    		MESSAGE(STATUS "RobWork Hardware: component rwhw_can - IEICAN is NOT included!")
    	ENDIF()
    ENDIF()
ENDIF()

LIST(FIND OPTIONAL_LIBRARIES_TO_INCLUDE "rwhw_can" USE_CAN)
IF(${USE_CAN} GREATER -1)
    #ESDCAN
    INCLUDE(${ROBWORKHARDWARE_ROOT}/build/FindESDCAN.cmake)
    IF(ESDCAN_FOUND)
        MESSAGE(STATUS "RobWork Hardware: component rwhw_can - ESDCAN is included!")
        ADD_DEFINITIONS(-DRWHW_HAS_ESDCAN)
        LIST(APPEND ROBWORKHARDWARE_LIBRARIES ${ESDCAN_LIBRARY})
        LIST(APPEND ROBWORKHARDWARE_INCLUDE_DIRS ${ESDCAN_INCLUDE_DIR})
    ELSE()
        MESSAGE(STATUS "RobWork Hardware: component rwhw_can - ESDCAN is NOT included!") 
    ENDIF()   
ENDIF()



RWHW_ADD_INTERNAL_LIBRARY("rwhw_camera")

#DC1394
LIST(FIND OPTIONAL_LIBRARIES_TO_INCLUDE "rwhw_camera" USE_CAMERA)
IF(${USE_CAMERA} GREATER -1)
    IF (CMAKE_COMPILER_IS_GNUCXX)
        IF (DEFINED MINGW)
            # TODO mingw32 libraries
        ELSE()
            INCLUDE(${ROBWORKHARDWARE_ROOT}/build/FindDC1394.cmake)
            IF(DC1394_FOUND)
                MESSAGE(STATUS "RobWork Hardware: component rwhw_camera - DC1394 is included!")
                SET(ROBWORKHARDWARE_LIBRARIES 
                ${ROBWORKHARDWARE_LIBRARIES} rwhw_camera ${DC1394_LIBRARY} )
                SET(ROBWORKHARDWARE_INCLUDE_DIRS ${ROBWORKHARDWARE_INCLUDE_DIRS} ${DC1394_INCLUDE_DIR})
            ELSE()
                MESSAGE(STATUS "RobWork Hardware: component rwhw_camera - DC1394 is NOT included!")
            ENDIF()
        ENDIF()
    ELSEIF (DEFINED MSVC)
        # TODO MSVC AND CMU1394
    ENDIF()
ENDIF()


#rwhw_serialport
IF(DEFINED MSVC)
  SET(ROBWORKHARDWARE_LIBS_DIR "${ROBWORKHARDWARE_ROOT}/libs/")
ELSE()
  SET(ROBWORKHARDWARE_LIBS_DIR "${ROBWORKHARDWARE_ROOT}/libs/${CMAKE_BUILD_TYPE}/")
ENDIF()

# Setup RobWorkHardware include and link directories
SET(ROBWORKHARDWARE_INCLUDE_DIR ${ROBWORKHARDWARE_INCLUDE_DIRS})
SET(ROBWORKHARDWARE_LIBRARY_DIRS ${ROBWORKHARDWARE_LIBS_DIR})

#MESSAGE(STATUS "Path to RobWorkHardware root (ROBWORKHARDWARE_ROOT) = ${ROBWORKHARDWARE_ROOT} \n" 
#               "Path to RobWorkHardware includes dir (ROBWORKHARDWARE_INCLUDE_DIR) = ${ROBWORKHARDWARE_INCLUDE_DIR} \n"
#               "Path to RobWorkHardware libraries dir (ROBWORKHARDWARE_LIBRARY_DIRS) = ${ROBWORKHARDWARE_LIBRARY_DIRS}\n"
#               "RobWork Hardware libraties: ${ROBWORKHARDWARE_LIBRARIES}")


# Find and add full path information for the RobWorkHardware libraries
SET(ROBWORKHARDWARE_LIBRARIES_TMP ${ROBWORKHARDWARE_LIBRARIES})
SET(ROBWORKHARDWARE_LIBRARIES)
FOREACH(l ${ROBWORKHARDWARE_LIBRARIES_TMP})
  UNSET(tmp CACHE)
  FIND_LIBRARY(tmp ${l} PATHS ${ROBWORKHARDWARE_LIBRARY_DIRS} NO_DEFAULT_PATH)
  IF(tmp)
    LIST(APPEND ROBWORKHARDWARE_LIBRARIES ${tmp})
  ELSE()
    LIST(APPEND ROBWORKHARDWARE_LIBRARIES ${l})
  ENDIF()
ENDFOREACH(l)

IF (ROBWORKHARDWARE_EXPERIENCED_FATAL_PROBLEMS)
  SET(ROBWORKHARDWARE_FOUND FALSE)
ELSE()
  SET(ROBWORKHARDWARE_FOUND TRUE)
ENDIF()
