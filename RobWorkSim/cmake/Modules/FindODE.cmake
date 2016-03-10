# Locate ode
# This module defines
# ODE_LIBRARY
# ODE_FOUND, if false, do not try to link to ode 
# ODE_INCLUDE_DIR, where to find the headers
# ODE_BUILD_WITH, set to DOUBLE or SINGLE
#
# this module optionally use followin vars to guide the search for ODE:
# ODE_DIR - path to ODE root dir
# ODE_LIBRARY_DIR - specify to guide the search in library
# ODE_LIBRARY_NAME - specify to guide library search ad selection
# ODE_USE_SINGLE - set if force using double
# ODE_USE_DOUBLE - set if force using double
# ODE_USE_DEBUG - set if force using debug
# created by RobWork, based on code by David Guthrie.  Based on code by Robert Osfield 

FIND_PATH(ODE_INCLUDE_DIR ode/ode.h
    HINTS
    ${ODE_DIR}/include
    $ENV{ODE_DIR}/include
    $ENV{ODE_DIR}
    ${DELTA3D_EXT_DIR}/inc
    $ENV{DELTA_ROOT}/ext/inc
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/include
    /usr/include
    /usr/include/cal3d
    /sw/include # Fink
    /opt/local/include # DarwinPorts
    /opt/csw/include # Blastwave
    /opt/include
    [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/include
    /usr/freeware/include
)


MACRO(FIND_ODE_LIBRARY MYLIBRARY MYLIBRARYNAME)
	set(CMAKE_FIND_LIBRARY_SUFFIXES .so .a .lib)
    FIND_LIBRARY(${MYLIBRARY}
        NAMES ${ODE_LIBRARY_NAME} ${MYLIBRARYNAME}
        HINTS
        ${ODE_LIBRARY_DIR}
        ${ODE_DIR}/lib
        $ENV{ODE_DIR}/lib
        $ENV{ODE_DIR}
        ${DELTA3D_EXT_DIR}/lib
        $ENV{DELTA_ROOT}/ext/lib
        ~/Library/Frameworks
        /Library/Frameworks
        /usr/local/lib
        /usr/lib
        /sw/lib
        /opt/local/lib
        /opt/csw/lib
        /opt/lib
        [HKEY_LOCAL_MACHINE\\SYSTEM\\CurrentControlSet\\Control\\Session\ Manager\\Environment;OSG_ROOT]/lib
        /usr/freeware/lib64
        PATH_SUFFIXES
        lib/ReleaseSingleLib
        lib/ReleaseSingleDLL
        lib/ReleaseDoubleLib
        lib/ReleaseDoubleDLL
        lib/DebugSingleLib
        lib/DebugSingleDLL
        lib/DebugDoubleLib
        lib/DebugDoubleDLL
        lib/Release
        lib/Debug
        ReleaseSingleLib
        ReleaseSingleDLL
        ReleaseDoubleLib
        ReleaseDoubleDLL        
        DebugSingleLib
        DebugSingleDLL
        DebugDoubleLib
        DebugDoubleDLL
        Release
        Debug
    )
    #MESSAGE("Find ode: ${${MYLIBRARY}}")
ENDMACRO(FIND_ODE_LIBRARY MYLIBRARY MYLIBRARYNAME)

UNSET(ODE_BUILD_WITH )
UNSET(ODE_LIBRARY)

IF( ODE_USE_DEBUG )
    IF(ODE_USE_SINGLE)
        SET(DEBUG_LIST ode_singled oded )
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
        SET(ODE_BUILD_WITH "SINGLE")    
    ELSEIF(ODE_USE_DOUBLE)
        SET(DEBUG_LIST ode_doubled oded )
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
        SET(ODE_BUILD_WITH "DOUBLE")
    ELSE()
        # else try first with single then with double
        SET(DEBUG_LIST ode_singled oded )
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
        SET(ODE_BUILD_WITH "SINGLE")
        IF(NOT ODE_LIBARY_TMP)
            SET(DEBUG_LIST ode_doubled oded )
            FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            SET(ODE_BUILD_WITH "DOUBLE")
        ENDIF()
    ENDIF()    
    
ELSE()
    
    IF(ODE_USE_SINGLE)
        SET(RELEASE_LIST ode_single ode )
        SET(DEBUG_LIST ode_singled oded )
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${RELEASE_LIST};${DEBUG_LIST}" )
        SET(ODE_BUILD_WITH "SINGLE")
    ELSEIF(ODE_USE_DOUBLE)
        SET(RELEASE_LIST ode_double ode )
        SET(DEBUG_LIST ode_doubled oded )
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${RELEASE_LIST};${DEBUG_LIST}")
        SET(ODE_BUILD_WITH "DOUBLE")
    ELSE()
        # first try single
        #MESSAGE("${ODE_LIBRARY_TMP} ___ ${RELEASE_LIST}")
        SET(RELEASE_LIST ode_single ode ode_singled oded)
        FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${RELEASE_LIST}")
        SET(ODE_BUILD_WITH "SINGLE")
        #MESSAGE("${ODE_LIBRARY_TMP} ___ ${RELEASE_LIST}")
        IF(NOT ODE_LIBRARY)
            SET(RELEASE_LIST ode_double ode)
            FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${RELEASE_LIST}")
            SET(ODE_BUILD_WITH "DOUBLE")
        ENDIF()
        # try debug
        IF(NOT ODE_LIBRARY)
            SET(DEBUG_LIST ode_singled oded )
            FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            SET(ODE_BUILD_WITH "SINGLE")
        ENDIF()
        IF(NOT ODE_LIBRARY)
            SET(DEBUG_LIST ode_doubled oded )
            FIND_ODE_LIBRARY(ODE_LIBRARY_TMP "${DEBUG_LIST}")
            SET(ODE_BUILD_WITH "DOUBLE")
        ENDIF()        
    ENDIF()        
ENDIF()

#MESSAGE("${ODE_LIBRARY_TMP}")
#MESSAGE("${ODE_INCLUDE_DIR}")
#MESSAGE("${ODE_LIBRARY_DEBUG}")

SET(ODE_FOUND NO)
IF(ODE_LIBRARY_TMP AND ODE_INCLUDE_DIR)
    SET(ODE_FOUND YES)
    IF( ${ODE_BUILD_WITH} STREQUAL "DOUBLE" )
        ADD_DEFINITIONS(-DdDOUBLE)
    ELSE()
        ADD_DEFINITIONS(-DdSINGLE)
    ENDIF()
    SET(ODE_LIBRARY ${ODE_LIBRARY_TMP})
    SET(ODE_LIBRARIES ${ODE_LIBRARY})
ENDIF(ODE_LIBRARY_TMP AND ODE_INCLUDE_DIR)

#MESSAGE("${CMAKE_BINARY_DIR}/odeTestStuff.c")

#file(WRITE "${CMAKE_BINARY_DIR}/odeTestStuff.c" 
#    "
#    #define dDOUBLE
#    #include <ode/ode.h>
#    int main(const char** args, int nargs ){
#        int res = dCheckConfiguration (\"ODE_double_precision\");
#        
#        return 0;
#    }
#    ")
#TRY_RUN(ODE_LIB_STUFF COMPILE_RES 
#        "${CMAKE_BINARY_DIR}" 
#        "${CMAKE_BINARY_DIR}/odeTestStuff.c" 
#        CMAKE_FLAGS -DINCLUDE_DIRECTORIES:STRING=${ODE_INCLUDE_DIR}
#        -DLINK_LIBRARIES:STRING=${ODE_LIBRARIES}
#        COMPILE_OUTPUT_VARIABLE comp)

#MESSAGE("ODE_LIB_STUFF ${ODE_LIB_STUFF}")
#MESSAGE("COMPILE_RES ${COMPILE_RES}")
#MESSAGE("comp: ${comp}") 
