SET(SUBSYS_NAME rws_luaeditor)
set(SUBSYS_DESC "A jog plugin")
SET(SUBSYS_DEPS rws rws_lua_s)

SET(build ${RWS_HAVE_LUA})
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    # if we want to use ui files add them here
    SET(UIS_FILES LuaEditorWindow.ui )
    RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    
    SET(SrcFiles LuaEditorWindow.cpp 
        LuaHighlighter.cpp 
        CodeEditor.cpp
        TreeModelCompleter.cpp
        LuaExecutionThread.cpp
        LuaConsoleWidget.cpp)
    SET(SRC_FILES_HPP LuaEditorWindow.hpp 
        LuaHighlighter.hpp 
        CodeEditor.hpp 
        TreeModelCompleter.hpp
        LuaExecutionThread.hpp
        LuaConsoleWidget.hpp)
    
    set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
    
    QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
    # Rcc the files:
    QT4_ADD_RESOURCES(RccSrcFiles luaresources.qrc)
            
    RWS_ADD_COMPONENT(${SUBSYS_NAME} components ${RWS_DEFAULT_LIB_TYPE} ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(components "rwslibs/luaeditor" ${SRC_FILES_HPP}) 
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

ENDIF()
