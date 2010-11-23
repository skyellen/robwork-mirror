
IF( RWS_HAVE_LUA )

#----------------------------------------------------------------------
# Define the following variables:

SET(TargetName rws_plugin_lua)

# Standard cpp files to compile:
SET(SrcFiles Lua.cpp LuaConsoleWidget.cpp)

# Header files to moc and compile:
QT4_WRAP_CPP(MocSrcFiles Lua.hpp LuaConsoleWidget.hpp)

# Rcc the files:
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

#----------------------------------------------------------------------
# The below should be the same for most plugins
#

# Set extra linker flags for apple macos computers
SET(CMAKE_SHARED_LINKER_FLAGS "${ExtraLinkerFlags} ${CMAKE_SHARED_LINKER_FLAGS}")

# The library to build:
IF (RWS_USE_STATIC_LINK_PLUGINS)
  ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_LIB_INSTALL_DIR})
ELSE ()
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RWS_LIBRARY_OUT_DIR})
  # Link the standard static libraries with with the shared library:
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_BIN_INSTALL_DIR})
ENDIF ()

ADD_DEPENDENCIES(${TargetName} rwstudio)
ADD_DEPENDENCIES(${TargetName} LuaEditorWindow)
ADD_DEPENDENCIES(${TargetName} rwstudio_lua)

SET(ENV{RWS_PLUGIN_LIBRARIES} "$ENV{RWS_PLUGIN_LIBRARIES}${TargetName};")

ENDIF()
