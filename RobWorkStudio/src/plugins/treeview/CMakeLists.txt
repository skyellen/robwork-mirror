#----------------------------------------------------------------------
# Define the following variables:

SET(TargetName rwstudio_treeview)

# Standard cpp files to compile:
SET(SrcFiles TreeView.cpp)

# Header files to moc and compile:
QT4_WRAP_CPP(MocSrcFiles TreeView.hpp)

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
ELSE ()
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
ENDIF ()

# Link the standard static libraries with with the shared library:
INSTALL(TARGETS ${TargetName} DESTINATION bin)
SET(ENV{RWS_PLUGIN_LIBRARIES} "$ENV{RWS_PLUGIN_LIBRARIES}${TargetName};")