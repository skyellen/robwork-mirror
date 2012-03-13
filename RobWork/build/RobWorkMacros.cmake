#
# This is a collection of macros used throughout the robwork project
# 

######################################################################
# Converts a standard cmake list to a python string list
#
MACRO(RW_TO_PYTHON_STR_LIST ITEMS OUTPUT)

SET(RESULT_STR "'")
FOREACH (item ${ITEMS})
    SET(RESULT_STR "${RESULT_STR}${item}','")
ENDFOREACH()
SET(${OUTPUT} "${RESULT_STR}'")

ENDMACRO()

######################################################################
# Converts a standard VERSION 0.1.2 to three version numbers
#
MACRO(RW_SPLIT_VERSION VERSION MAJOR MINOR PATCH)
STRING( REGEX MATCHALL "[0-9]+" VERSIONS ${VERSION} )
LIST( GET VERSIONS 0 ${MAJOR})
LIST( GET VERSIONS 1 ${MINOR})
LIST( GET VERSIONS 2 ${PATCH})
ENDMACRO()

######################################################################
# Get a string describing the current system, e.g. windows-mingw-x64, mac-x64 or ubuntu-11.04-x64
#
MACRO(RW_SYS_INFO INFO)
    IF(CMAKE_SIZEOF_VOID_P EQUAL 4)
    	SET(ARCH "x86")
    ELSE()
    	SET(ARCH "amd64")
    ENDIF()
    
    IF(UNIX)
        IF(MAC)
            SET(SUFFIX "mac-${ARCH}")
        ELSE()
            #EXECUTE_PROCESS(COMMAND cat /etc/issue OUTPUT_VARIABLE SUFFIX)
            EXECUTE_PROCESS(COMMAND cat /etc/lsb-release OUTPUT_VARIABLE SUFFIX)
            STRING( REGEX MATCHALL "\".+\"" SUFFIX ${SUFFIX} )
            
            # this will add kernel version eg Linux_3.0....
            #EXECUTE_PROCESS(COMMAND uname  -r OUTPUT_VARIABLE KERNEL_VERSION)
            #SET(SUFFIX "${SUFFIX}_${KERNEL_VERSION}")
            SET(SUFFIX "${SUFFIX}_${ARCH}_${CMAKE_BUILD_TYPE}")
            STRING( REPLACE "\"" "" SUFFIX ${SUFFIX} )
            STRING( REPLACE " " "_" SUFFIX ${SUFFIX} )
        
            # this will make it lowercase        
            #STRING(TOLOWER "${SUFFIX}" SUFFIX)
            #MESSAGE("${SUFFIX}")
        ENDIF()
    ELSEIF(MINGW)
    	SET(SUFFIX "windows-mingw-${ARCH}")
    ELSEIF(MSVC)
    	IF(MSVC80)
    		SET(SUFFIX "windows-msvc2005-${ARCH}")
    	ELSEIF(MSVC90)
    		SET(SUFFIX "windows-msvc2008-${ARCH}")
    	ELSEIF(MSVC10)
    		SET(SUFFIX "windows-msvc2010-${ARCH}")
    	ENDIF()
    ELSE()
    	# Trouble
    	
    ENDIF()
    SET(${INFO} ${SUFFIX})
ENDMACRO()
