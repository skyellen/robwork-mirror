SET(SUBSYS_NAME rws_luapl)
set(SUBSYS_DESC "A lua plugin")
SET(SUBSYS_DEPS rws rws_luaeditor rws_lua_s)

SET(build ${RWS_HAVE_LUA})
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles Lua.cpp)
    SET(SRC_FILES_HPP Lua.hpp)
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
        
    # Set extra linker flags for apple macos computers    
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwslibs/lua" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
