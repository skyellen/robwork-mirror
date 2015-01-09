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
    
#rehat: /etc/redhat-release
#Slackware: /etc/slackware-version
#Slamd64:   /etc/slamd64-version
#Fedora:    /etc/fedora-
    
    IF(UNIX)
        IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
            SET(SUFFIX "mac-${ARCH}_${RW_BUILD_TYPE}")
        ELSE()
            IF(EXISTS "/etc/lsb-release")
                EXECUTE_PROCESS(COMMAND cat /etc/lsb-release OUTPUT_VARIABLE SUFFIX)
                STRING( REGEX MATCHALL "\".+\"" SUFFIX ${SUFFIX} )            
                # this will add kernel version eg Linux_3.0....
                SET(SUFFIX "${SUFFIX}_${ARCH}_${RW_BUILD_TYPE}")
                STRING( REPLACE "\"" "" SUFFIX ${SUFFIX} )
                STRING( REPLACE " " "_" SUFFIX ${SUFFIX} )
            
            ELSEIF(EXISTS "/etc/os-release")
                EXECUTE_PROCESS(COMMAND cat /etc/os-release OUTPUT_VARIABLE SUFFIX)
                STRING( REGEX MATCHALL "ID=\".+\"" SUFFIX1 ${SUFFIX} )
                STRING( REGEX MATCHALL "VERSION_ID=\".+\"" SUFFIX2 ${SUFFIX} )
                # this will add kernel version eg Linux_3.0....
                SET(SUFFIX "${SUFFIX1}-${SUFFIX2}-${ARCH}_${RW_BUILD_TYPE}")
                STRING( REPLACE "\"" "" SUFFIX ${SUFFIX} )
                STRING( REPLACE " " "_" SUFFIX ${SUFFIX} )
            ELSEIF(EXISTS "/etc/redhat-release")
                SET(SUFFIX "redhat-${ARCH}_${RW_BUILD_TYPE}")
            ELSEIF(EXISTS "/etc/slackware-version")
                SET(SUFFIX "slackware-${ARCH}_${RW_BUILD_TYPE}")
            ELSEIF(EXISTS "/etc/fedora-release")
                SET(SUFFIX "fedora-${ARCH}_${RW_BUILD_TYPE}")
            ELSE( )
                # this will make it lowercase
                SET(SUFFIX "linux-${ARCH}")
            ENDIF()
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




#############################################################################
# This is a default project setup. It enables multiple build trees for 
# multiple configuration eg. CMAKE_BUILD_TYPE 
# 
# input:
# ROOT : root of the project folder. if not defined then it will be defined as
# PROJECT_NAME: name of project, something like RobWork or RobWorkStudio, MyProject, 
# PREFIX: for RobWork its RW, for RobWorkStudio its RWS. It will be used as suffix to project specific paths
#
# defines :
# ${PREFIX}_CMAKE_RUNTIME_OUTPUT_DIRECTORY 
# ${PREFIX}_CMAKE_ARCHIVE_OUTPUT_DIRECTORY
# ${PREFIX}_CMAKE_LIBRARY_OUTPUT_DIRECTORY
# and sets up cmake variables
MACRO(RW_INIT_PROJECT ROOT PROJECT_NAME PREFIX VERSION)
    #MESSAGE("ROOOT, ${ROOT} ${PROJECT_NAME} ${PREFIX}")
    # Allow the syntax else (), endif (), etc.
    SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)
    
    # Enable new linker path policy.
    IF (COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    ENDIF ()
    
    #OPTION(RW_VERBOSE "Set to true if cmake build information should be printet!" False)
        
    # Specify wether to default compile in Release, Debug, MinSizeRel, RelWithDebInfo mode
    IF (NOT CMAKE_BUILD_TYPE)
        SET(${PREFIX}_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
    else ()
	# we need to force the right configuration
	STRING(TOLOWER ${CMAKE_BUILD_TYPE} TMP_BUILD_TYPE)
	IF (${TMP_BUILD_TYPE} STREQUAL "release")
		SET(${PREFIX}_BUILD_TYPE "Release" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
	ELSEIF ( ${TMP_BUILD_TYPE} STREQUAL "debug")
		SET(${PREFIX}_BUILD_TYPE "Debug" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
	ELSEIF ( ${TMP_BUILD_TYPE} STREQUAL "relwithdebinfo")
		SET(${PREFIX}_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
	ELSEIF ( ${TMP_BUILD_TYPE} STREQUAL "minsizerel")
		SET(${PREFIX}_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type: Release, Debug, RelWithDebInfo, MinSizeRel." FORCE)
	ELSE ()
		MESSAGE(FATAL_ERROR "Build type: ${CMAKE_BUILD_TYPE} not supported! please select one of: Release, Debug, RelWithDebInfo, MinSizeRel")
	ENDIF ()
        
    ENDIF ()

    STRING(TOLOWER ${${PREFIX}_BUILD_TYPE} ${PREFIX}_BUILD_TYPE)
    MESSAGE(STATUS "${PROJECT_NAME}: Build configuration: ${${PREFIX}_BUILD_TYPE}")
    
    # Load the optional Default.cmake file.
    INCLUDE(${ROOT}/config.cmake OPTIONAL)
    IF (NOT EXISTS ${ROOT}/config.cmake)
      IF (EXISTS ${ROOT}/config.cmake.template)
          # Setup the default settings in case no RobWork.cmake exist.
          INCLUDE(${ROOT}/config.cmake.template)
          #MESSAGE(STATUS "Using default settings from config.cmake.template")
      ENDIF()
    ENDIF ()
    
    SET(${PREFIX}_CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${${PREFIX}_BUILD_TYPE}" CACHE PATH "Runtime directory"  FORCE )
    SET(${PREFIX}_CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}" CACHE PATH "Library directory"  FORCE )
    SET(${PREFIX}_CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${${PREFIX}_BUILD_TYPE}" CACHE PATH "Archive directory"  FORCE )
    
    # Output goes to bin/<CONFIG> and libs/<CONFIG> unless specified otherwise by the user.
    IF (DEFINED MSVC)
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
    ELSE ()
        SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${RW_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
        SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${RW_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
        SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${RW_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
    ENDIF ()
    
    STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UP)
    #MESSAGE("uppercase ${PROJECT_NAME_UP}_VERSION")
    SET(${PROJECT_NAME_UP}_VERSION ${VERSION} CACHE STRING "Project Version Nr" FORCE)
    STRING( REGEX MATCHALL "[0-9]+" ${PROJECT_NAME_UP}_VERSIONS ${VERSION})
    LIST( GET ${PROJECT_NAME_UP}_VERSIONS 0 ${PROJECT_NAME_UP}_VERSION_MAJOR)
    LIST( GET ${PROJECT_NAME_UP}_VERSIONS 1 ${PROJECT_NAME_UP}_VERSION_MINOR)
    LIST( GET ${PROJECT_NAME_UP}_VERSIONS 2 ${PROJECT_NAME_UP}_VERSION_PATCH)
    SET(PROJECT_VERSION ${${PROJECT_NAME_UP}_VERSION})
    SET(PROJECT_VERSION_MAJOR ${${PROJECT_NAME_UP}_VERSION_MAJOR})
    SET(PROJECT_VERSION_MINOR ${${PROJECT_NAME_UP}_VERSION_MINOR})
    SET(PROJECT_VERSION_PATCH ${${PROJECT_NAME_UP}_VERSION_PATCH})
    MESSAGE(STATUS "${PROJECT_NAME}: Version ${${PROJECT_NAME_UP}_VERSION}")
    set(RW_SUBSYSTEMS "" CACHE INTERNAL "Internal list of subsystems" FORCE)
    # setup install directories
ENDMACRO()

MACRO(RW_GET_OS_INFO)
    # Get the compiler architecture
    IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    	SET(AMD64 TRUE)
    ELSE()
    	SET(AMD64 FALSE)
    ENDIF()
ENDMACRO()


###############################################################################
# Set the destination directories for installing stuff.
# input: 
#   PREFIX: project prefix id
# Sets LIB_INSTALL_DIR. Install libraries here.
# Sets BIN_INSTALL_DIR. Install binaries here.
# Sets INCLUDE_INSTALL_DIR. Install include files here, preferably in a
macro(RW_SET_INSTALL_DIRS PROJECT_NAME PREFIX)
    STRING(TOLOWER ${PREFIX} PREFIX_LOWER)
    STRING(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
    STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
    IF (NOT DEFINED LIB_INSTALL_DIR)
        SET(LIB_INSTALL_DIR "lib")
    ENDIF (NOT DEFINED LIB_INSTALL_DIR)
    SET(INCLUDE_INSTALL_ROOT "include/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}")
    SET(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_ROOT}")
    SET(EXT_INSTALL_DIR ${INCLUDE_INSTALL_DIR}/ext/)
    SET(BIN_INSTALL_DIR "bin")
    SET(PKGCFG_INSTALL_DIR "${LIB_INSTALL_DIR}/pkgconfig")
    
    IF(WIN32)
        SET(${PREFIX}_INSTALL_DIR "${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}")
        SET(CONFIG_INSTALL_DIR "${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}/cmake")
    ELSE(WIN32)
        SET(${PREFIX}_INSTALL_DIR "share/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}")
        set(CONFIG_INSTALL_DIR "share/${PROJECT_NAME_LOWER}-${${PROJECT_NAME_UPPER}_VERSION_MAJOR}.${${PROJECT_NAME_UPPER}_VERSION_MINOR}")
    ENDIF(WIN32)
ENDMACRO(RW_SET_INSTALL_DIRS)

MACRO(RW_IS_RELEASE IS_RELEASE)
    IF(${RW_BUILD_TYPE} STREQUAL "release" OR ${RW_BUILD_TYPE} STREQUAL "relwithdebinfo" OR ${RW_BUILD_TYPE} STREQUAL "minsizerel")
        SET(${IS_RELEASE} TRUE)
    ELSE()
        SET(${IS_RELEASE} FALSE)
    ENDIF()
ENDMACRO(RW_IS_RELEASE)


MACRO(RW_OPTIONS)
    # Build shared libraries by default.
    option(PROJECT_SHARED_LIBS "Build shared libraries." OFF)
    if(PROJECT_SHARED_LIBS)
      set(PROJECT_LIB_PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
      set(PROJECT_LIB_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
      set(PROJECT_LIB_TYPE "SHARED")
    else(PROJECT_SHARED_LIBS)
      set(PROJECT_LIB_PREFIX ${CMAKE_STATIC_LIBRARY_PREFIX})
      set(PROJECT_LIB_SUFFIX ${CMAKE_STATIC_LIBRARY_SUFFIX})
      set(PROJECT_LIB_TYPE "STATIC")
    endif(PROJECT_SHARED_LIBS)
    mark_as_advanced(PROJECT_SHARED_LIBS)
ENDMACRO(RW_OPTIONS)



###############################################################################
# Add a set of include files to install.
# _component The part of RW that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
MACRO(RW_ADD_INCLUDES _component _subdir)
    INSTALL(FILES ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT ${_component})
ENDMACRO(RW_ADD_INCLUDES)

###############################################################################
# Add a set of include files to install.
# _component The part of RW that the install files belong to.
# _subdir The sub-directory for these include files.
# ARGN The include files.
MACRO(RW_ADD_INCLUDE_DIRS _component _subdir)
    INSTALL(DIRECTORY ${ARGN} DESTINATION ${INCLUDE_INSTALL_DIR}/${_subdir} COMPONENT ${_component}
        FILES_MATCHING 
            PATTERN "*.h" 
            PATTERN "*.hpp"
            PATTERN ".svn" EXCLUDE
    )
ENDMACRO(RW_ADD_INCLUDE_DIRS)


###############################################################################
# Add a library target.
# _name The library name.
# _component The part of RW that this library belongs to.
# ARGN The source files for the library.
MACRO(RW_ADD_LIBRARY _name _component)
    ADD_LIBRARY(${_name} ${PROJECT_LIB_TYPE} ${ARGN})
    # must link explicitly against boost.
    target_link_libraries(${_name} ${Boost_LIBRARIES})
    
    # Only link if needed
    if(WIN32 AND MSVC)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl)
    elseif(__COMPILER_PATHSCALE)
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -mp)
    else()
      set_target_properties(${_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
    endif()
    #
    set_target_properties(${_name} PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        #DEFINE_SYMBOL "RWAPI_EXPORTS"
        )
    #if(USE_PROJECT_FOLDERS)
    #  set_target_properties(${_name} PROPERTIES FOLDER "Libraries")
    #endif(USE_PROJECT_FOLDERS)

    install(TARGETS ${_name}
        RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ${_component}
        LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ${_component})

endmacro(RW_ADD_LIBRARY)


###############################################################################
# Set a value in a global, cached map.
# _map The map name.
# _key The key name.
# _value The value.
macro(SET_IN_GLOBAL_MAP _map _key _value)
    set("${_map}_${_key}" "${_value}" CACHE INTERNAL "Map value" FORCE)
endmacro(SET_IN_GLOBAL_MAP)


###############################################################################
# Get a value from a map.
# _dest The name of the variable to store the value in.
# _map The map name.
# _key The key name.
macro(GET_IN_MAP _dest _map _key)
    set(${_dest} ${${_map}_${_key}})
endmacro(GET_IN_MAP)


###############################################################################
# Make one subsystem depend on one or more other subsystems, and disable it if
# they are not being built.
# _var The cumulative build variable. This will be set to FALSE if the
#   dependencies are not met.
# _name The name of the subsystem.
# ARGN The subsystems and external libraries to depend on.
macro(RW_SUBSYS_DEPEND _var _name)
# at some point we might start using this.... 
# for now we are readying the infrastructure
    set(options)
    set(oneValueArgs)
    set(multiValueArgs DEPS EXT_DEPS OPT_DEPS)
#    cmake_parse_arguments(SUBSYS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    #if(SUBSYS_DEPS)
#        SET_IN_GLOBAL_MAP(RW_SUBSYS_DEPS ${_name} "${SUBSYS_DEPS}")
     ####   #ADD_DEPENDENCIES(${_name} ${SUBSYS_DEPS})
    #endif(SUBSYS_DEPS)
#    if(SUBSYS_EXT_DEPS)
#        SET_IN_GLOBAL_MAP(RW_SUBSYS_EXT_DEPS ${_name} "${SUBSYS_EXT_DEPS}")
#    endif(SUBSYS_EXT_DEPS)
#    if(SUBSYS_OPT_DEPS)
#        SET_IN_GLOBAL_MAP(RW_SUBSYS_OPT_DEPS ${_name} "${SUBSYS_OPT_DEPS}")
#    endif(SUBSYS_OPT_DEPS)
#    GET_IN_MAP(subsys_status RW_SUBSYS_HYPERSTATUS ${_name})
#    if(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
#        if(SUBSYS_DEPS)
#        foreach(_dep ${SUBSYS_DEPS})
#            RW_GET_SUBSYS_STATUS(_status ${_dep})
#            if(NOT _status)
#                set(${_var} FALSE)
#                RW_SET_SUBSYS_STATUS(${_name} FALSE "Requires ${_dep}.")
#            else(NOT _status)
#                RW_GET_SUBSYS_INCLUDE_DIR(_include_dir ${_dep})
#                include_directories(${PROJECT_SOURCE_DIR}/${_include_dir}/include)
#            endif(NOT _status)
#        endforeach(_dep)
#        endif(SUBSYS_DEPS)
#        if(SUBSYS_EXT_DEPS)
#        foreach(_dep ${SUBSYS_EXT_DEPS})
#            string(TOUPPER "${_dep}_found" EXT_DEP_FOUND)
#            if(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
#                set(${_var} FALSE)
#                RW_SET_SUBSYS_STATUS(${_name} FALSE "Requires external library ${_dep}.")
#            endif(NOT ${EXT_DEP_FOUND} OR (NOT ("${EXT_DEP_FOUND}" STREQUAL "TRUE")))
#        endforeach(_dep)
#        endif(SUBSYS_EXT_DEPS)
#    endif(${_var} AND (NOT ("${subsys_status}" STREQUAL "AUTO_OFF")))
endmacro(RW_SUBSYS_DEPEND)


###############################################################################
# Set the include directory name of a subsystem.
# _name Subsystem name.
# _includedir Name of subdirectory for includes 
# ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_INCLUDE_DIR _name _includedir)
    SET_IN_GLOBAL_MAP(RW_SUBSYS_INCLUDE ${_name} ${_includedir})
endmacro(RW_SET_SUBSYS_INCLUDE_DIR)


###############################################################################
# Get the include directory name of a subsystem - return _name if not set
# _var Destination variable.
# _name Name of the subsystem.
macro(RW_GET_SUBSYS_INCLUDE_DIR _var _name)
    GET_IN_MAP(${_var} RW_SUBSYS_INCLUDE ${_name})
    if(NOT ${_var})
      set (${_var} ${_name})
    endif(NOT ${_var})
endmacro(RW_GET_SUBSYS_INCLUDE_DIR)



###############################################################################
# Register a subsystem.
# _name Subsystem name.
# _desc Description of the subsystem
macro(RW_ADD_SUBSYSTEM _name _desc)
    set(_temp ${RW_SUBSYSTEMS})
    list(APPEND _temp ${_name})
    set(RW_SUBSYSTEMS ${_temp} CACHE INTERNAL "Internal list of subsystems"
        FORCE)
    SET_IN_GLOBAL_MAP(RW_SUBSYS_DESC ${_name} ${_desc})
endmacro(RW_ADD_SUBSYSTEM)

###############################################################################
# Add an option to build a subsystem or not.
# _var The name of the variable to store the option in.
# _name The name of the option's target subsystem.
# _desc The description of the subsystem.
# _default The default value (TRUE or FALSE)
# ARGV5 The reason for disabling if the default is FALSE.
macro(RW_SUBSYS_OPTION _var _name _desc _default)
    set(_opt_name "BUILD_${_name}")
    RW_GET_SUBSYS_HYPERSTATUS(subsys_status ${_name})
    if(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
      option(${_opt_name} ${_desc} ${_default})
      if(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} FALSE)
        if(${ARGC} GREATER 4)
          set(_reason ${ARGV4})
        else(${ARGC} GREATER 4)
          set(_reason "Disabled by default.")
        endif(${ARGC} GREATER 4)
        RW_SET_SUBSYS_STATUS(${_name} FALSE ${_reason})
        MESSAGE(STATUS "${_opt_name}  ${BUILD_${_name}} : ${_reason}")
        RW_DISABLE_DEPENDIES(${_name})
      elseif(NOT ${_opt_name})
        set(${_var} FALSE)
        RW_SET_SUBSYS_STATUS(${_name} FALSE "Disabled manually.")
        MESSAGE(STATUS "${_opt_name}  ${BUILD_${_name}} : Disabled manually.")
        RW_DISABLE_DEPENDIES(${_name})
      else(NOT ${_default} AND NOT ${_opt_name})
        set(${_var} TRUE)
        if(${ARGC} GREATER 4)
          set(_reason ${ARGV4})
          IF( "${ARGV4}" STREQUAL "" )
              set(_reason "Enabled by default.")
          ENDIF()
        else()
            set(_reason "Enabled by default.")
        endif(${ARGC} GREATER 4)
        
        RW_SET_SUBSYS_STATUS(${_name} TRUE ${_reason})
        MESSAGE(STATUS "${_opt_name}  ${BUILD_${_name}} : ${_reason}")
        RW_ENABLE_DEPENDIES(${_name})
      endif(NOT ${_default} AND NOT ${_opt_name})
    endif(NOT ("${subsys_status}" STREQUAL "AUTO_OFF"))
    
    
    RW_ADD_SUBSYSTEM(${_name} ${_desc})
endmacro(RW_SUBSYS_OPTION)


########################################################################################
# Macro to disable subsystem dependies
# _subsys IN subsystem name
macro(RW_DISABLE_DEPENDIES _subsys)
    string(TOUPPER "rw_${_subsys}_dependies" RW_SUBSYS_DEPENDIES)
    if(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${RW_SUBSYS_DEPENDIES}})
            RW_SET_SUBSYS_HYPERSTATUS(${_subsys} ${dep} AUTO_OFF "Automatically disabled.")
            set(BUILD_${dep} OFF CACHE BOOL "Automatically disabled ${dep}" FORCE)
        endforeach(dep)
    endif(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro(RW_DISABLE_DEPENDIES subsys)

########################################################################################
# Macro to enable subsystem dependies
# _subsys IN subsystem name
macro(RW_ENABLE_DEPENDIES _subsys)
    string(TOUPPER "rw_${_subsys}_dependies" RW_SUBSYS_DEPENDIES)
    if(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
        foreach(dep ${${RW_SUBSYS_DEPENDIES}})
            RW_GET_SUBSYS_HYPERSTATUS(dependee_status ${_subsys} ${dep})
            if("${dependee_status}" STREQUAL "AUTO_OFF")
                RW_SET_SUBSYS_HYPERSTATUS(${_subsys} ${dep} AUTO_ON)
                GET_IN_MAP(desc RW_SUBSYS_DESC ${dep})
                set(BUILD_${dep} ON CACHE BOOL "${desc}" FORCE)
            endif("${dependee_status}" STREQUAL "AUTO_OFF")
        endforeach(dep)
    endif(NOT ("${${RW_SUBSYS_DEPENDIES}}" STREQUAL ""))
endmacro(RW_ENABLE_DEPENDIES subsys)

###############################################################################
# Get the status of a subsystem
# _var Destination variable.
# _name Name of the subsystem.
macro(RW_GET_SUBSYS_STATUS _var _name)
    GET_IN_MAP(${_var} RW_SUBSYS_STATUS ${_name})
endmacro(RW_GET_SUBSYS_STATUS)


###############################################################################
# Set the status of a subsystem.
# _name Subsystem name.
# _status TRUE if being built, FALSE otherwise.
# ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_STATUS _name _status)
    if(${ARGC} EQUAL 3)
        set(_reason ${ARGV2})
    else(${ARGC} EQUAL 3)
        set(_reason "No reason")
    endif(${ARGC} EQUAL 3)
    SET_IN_GLOBAL_MAP(RW_SUBSYS_STATUS ${_name} ${_status})
    SET_IN_GLOBAL_MAP(RW_SUBSYS_REASONS ${_name} ${_reason})
endmacro(RW_SET_SUBSYS_STATUS)

###############################################################################
# Set the hyperstatus of a subsystem and its dependee
# _name Subsystem name.
# _dependee Dependant subsystem.
# _status AUTO_OFF to disable AUTO_ON to enable
# ARGN[0] Reason for not building.
macro(RW_SET_SUBSYS_HYPERSTATUS _name _dependee _status) 
    SET_IN_GLOBAL_MAP(RW_SUBSYS_HYPERSTATUS ${_name}_${_dependee} ${_status})
    if(${ARGC} EQUAL 4)
        SET_IN_GLOBAL_MAP(RW_SUBSYS_REASONS ${_dependee} ${ARGV3})
    endif(${ARGC} EQUAL 4)
endmacro(RW_SET_SUBSYS_HYPERSTATUS)

###############################################################################
# Get the hyperstatus of a subsystem and its dependee
# _name IN subsystem name.
# _dependee IN dependant subsystem.
# _var OUT hyperstatus
# ARGN[0] Reason for not building.
macro(RW_GET_SUBSYS_HYPERSTATUS _var _name)
    set(${_var} "AUTO_ON")
    if(${ARGC} EQUAL 3)
        GET_IN_MAP(${_var} RW_SUBSYS_HYPERSTATUS ${_name}_${ARGV2})
    else(${ARGC} EQUAL 3)
        foreach(subsys ${RW_SUBSYS_DEPS_${_name}})
            if("${RW_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
                set(${_var} "AUTO_OFF")
                break()
            endif("${RW_SUBSYS_HYPERSTATUS_${subsys}_${_name}}" STREQUAL "AUTO_OFF")
        endforeach(subsys)
    endif(${ARGC} EQUAL 3)
endmacro(RW_GET_SUBSYS_HYPERSTATUS)



########################################################################################
# Macro to build subsystem centric documentation
# _subsys IN the name of the subsystem to generate documentation for
macro (RW_ADD_DOC _subsys)
  string(TOUPPER "${_subsys}" SUBSYS)
  set(doc_subsys "doc_${_subsys}")
  GET_IN_MAP(dependencies RW_SUBSYS_DEPS ${_subsys})
  if(DOXYGEN_FOUND)
    if(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP YES)
    else(HTML_HELP_COMPILER)
      set(DOCUMENTATION_HTML_HELP NO)
    endif(HTML_HELP_COMPILER)
    if(DOXYGEN_DOT_EXECUTABLE)
      set(HAVE_DOT YES)
    else(DOXYGEN_DOT_EXECUTABLE)
      set(HAVE_DOT NO)
    endif(DOXYGEN_DOT_EXECUTABLE)
    if(NOT "${dependencies}" STREQUAL "")
      set(STRIPPED_HEADERS "${RW_SOURCE_DIR}/${dependencies}/include")
      string(REPLACE ";" "/include \\\n\t\t\t\t\t\t\t\t\t\t\t\t ${RW_SOURCE_DIR}/" 
             STRIPPED_HEADERS "${STRIPPED_HEADERS}")
    endif(NOT "${dependencies}" STREQUAL "")
    set(DOC_SOURCE_DIR "\"${CMAKE_CURRENT_SOURCE_DIR}\"\\")
    foreach(dep ${dependencies})
      set(DOC_SOURCE_DIR 
          "${DOC_SOURCE_DIR}\n\t\t\t\t\t\t\t\t\t\t\t\t \"${RW_SOURCE_DIR}/${dep}\"\\")
    endforeach(dep)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html")
    set(doxyfile "${CMAKE_CURRENT_BINARY_DIR}/doxyfile")
    configure_file("${RW_SOURCE_DIR}/doc/doxygen/doxyfile.in" ${doxyfile})
    add_custom_target(${doc_subsys} ${DOXYGEN_EXECUTABLE} ${doxyfile})
    if(USE_PROJECT_FOLDERS)
      set_target_properties(${doc_subsys} PROPERTIES FOLDER "Documentation")
    endif(USE_PROJECT_FOLDERS)
  endif(DOXYGEN_FOUND)
endmacro(RW_ADD_DOC)
