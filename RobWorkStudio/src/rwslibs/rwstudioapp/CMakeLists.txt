SET(SUBSYS_NAME rws_rwstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
SET(SUBSYS_DEPS rws)

IF( RWS_USE_STATIC_LINK_PLUGINS )
    LIST(APPEND SUBSYS_DEPS rws_jog rws_log rws_planning rws_playback rws_sensors rws_treeview rws_propertyview)
	IF ( RWS_HAVE_LUA )
    	LIST(APPEND SUBSYS_DEPS rws_luaeditor rws_luapl)
    ENDIF() 
ENDIF()

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles RobWorkStudioApp.cpp)
    SET(SRC_FILES_HPP RobWorkStudioApp.hpp)
    
    RWS_ADD_COMPONENT(${SUBSYS_NAME} plugins ${SrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/rwstudioapp" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()

	IF(WIN32 AND MSVC AND PROJECT_LIB_TYPE AND CMAKE_VERSION VERSION_LESS 3.4)
		MESSAGE(FATAL_ERROR "Building shared libraries on Windows requires CMake 3.4 or newer.")
	ENDIF()
ENDIF()


