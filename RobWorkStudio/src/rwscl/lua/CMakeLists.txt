SET(SUBSYS_NAME rwscl_lua)
set(SUBSYS_DESC "A jog plugin")
SET(SUBSYS_DEPS rwstudio rwscl_luaeditor rws_lua_s)

SET(build ${RWS_HAVE_LUA})
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)    
    # Standard cpp files to compile:
    SET(SrcFiles Lua.cpp)
    SET(SRC_FILES_HPP Lua.hpp)
    
    # Header files to moc and compile:
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
        
    # Set extra linker flags for apple macos computers    
    RWS_ADD_PLUGIN(${SUBSYS_NAME} plugins ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
    RW_ADD_INCLUDES(plugins "rwscl/lua" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})
ENDIF()
