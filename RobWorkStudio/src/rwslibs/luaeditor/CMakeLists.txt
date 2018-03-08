SET(SUBSYS_NAME rws_luaeditor)
set(SUBSYS_DESC "A lua editor plugin")
SET(SUBSYS_DEPS rws)

SET(build ${RWS_HAVE_LUA})
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${RWS_HAVE_LUA})
RW_SUBSYS_DEPEND(build ${SUBSYS_NAME} DEPS ${SUBSYS_DEPS})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    # if we want to use ui files add them here
    SET(UIS_FILES LuaEditorWindow.ui )
    IF(RWS_USE_QT5)
      QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ELSE()
      QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
    ENDIF()
	#Need to add the current binary dir to the include directory because the generated source files are placed here
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
	
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
    
    IF(RWS_USE_QT5)
    	QT5_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP})
    ELSE()
    # Header files to moc and compile:
    	QT4_WRAP_CPP(MocSrcFiles ${SRC_FILES_HPP} OPTIONS -DBOOST_TT_HAS_PLUS_HPP_INCLUDED -DBOOST_TT_HAS_PLUS_ASSIGN_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_HPP_INCLUDED -DBOOST_TT_HAS_MINUS_ASSIGN_HPP_INCLUDED)
    ENDIF()
            
    RWS_ADD_COMPONENT(${SUBSYS_NAME} components ${SrcFiles} ${MocSrcFiles} ${UIS_OUT_H})
    RW_ADD_INCLUDES(components "rwslibs/luaeditor" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()

	IF(WIN32 AND MSVC AND PROJECT_LIB_TYPE AND CMAKE_VERSION VERSION_LESS 3.4)
		MESSAGE(FATAL_ERROR "Building shared libraries on Windows requires CMake 3.4 or newer.")
	ENDIF()

ENDIF()
