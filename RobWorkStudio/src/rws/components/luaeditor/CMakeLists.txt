IF( RWS_HAVE_LUA )

#----------------------------------------------------------------------
# Define the following variables:

# The shared library to build:
set(TargetName LuaEditorWindow)

# if we want to use ui files add them here
SET(UIS_FILES LuaEditorWindow.ui )
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

# MESSAGE("LUA UI FILES ${UIS_OUT_H}")

SET(SrcFiles LuaEditorWindow.cpp 
    LuaHighlighter.cpp 
    CodeEditor.cpp
    TreeModelCompleter.cpp
    LuaExecutionThread.cpp
    LuaConsoleWidget.cpp
    ${UIS_OUT_H})

QT4_WRAP_CPP(MocSrcFiles LuaEditorWindow.hpp 
    LuaHighlighter.hpp 
    CodeEditor.hpp 
    TreeModelCompleter.hpp
    LuaExecutionThread.hpp
    LuaConsoleWidget.hpp)

# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles )

#----------------------------------------------------------------------
# Everything below should be common for all components:

# The library to build:
IF (RWS_USE_STATIC_LINK_COMPONENTS)
  ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_LIB_INSTALL_DIR})
ELSE ()
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  # Link the standard static libraries with with the shared library:
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_BIN_INSTALL_DIR})
ENDIF ()

ADD_DEPENDENCIES(${TargetName} rwstudio)

SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}${TargetName};")

ENDIF()


