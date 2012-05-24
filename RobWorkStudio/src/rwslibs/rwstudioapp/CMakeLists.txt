SET(SUBSYS_NAME rws_rwstudioapp)
set(SUBSYS_DESC "A rwstudio application that may be started in a thread.")
SET(SUBSYS_DEPS rws rws_jog rws_log rws_planning rws_playback rws_sensors rws_treeview)

IF( RWS_HAVE_LUA )
    LIST(APPEND SUBSYS_DEPS rws_luaeditor rws_lua_s) 
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
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles )
        
    RWS_ADD_COMPONENT(${SUBSYS_NAME} plugins ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/rwstudioapp" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()


