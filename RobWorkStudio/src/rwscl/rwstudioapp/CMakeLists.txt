SET(SUBSYS_NAME rwscl_rwstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
SET(SUBSYS_DEPS rwstudio rwscl_jog rwscl_log rwscl_planning rwsc_playback rwscl_sensors rwsc_treeview)

IF( RWS_HAVE_LUA )
    LIST(APPEND SUBSYS_DEPS rwscl_luaeditor rwscl_lua rws_lua_s) 
ENDIF()

SET(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles RobWorkStudioApp.cpp)
    SET(SRC_FILES_HPP RobWorkStudioApp.hpp)
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles )
        
    RWS_ADD_COMPONENT(${SUBSYS_NAME} plugins ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwscl/rwstudioapp" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()


