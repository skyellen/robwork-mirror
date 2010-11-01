#----------------------------------------------------------------------
# Define the following variables:

SET(TargetName rws_plugin_sensors)

# if we want to use ui files add them here
SET(UIS_FILES SensorsPlugin.ui )
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

# Standard cpp files to compile:
SET(SrcFiles Sensors.cpp SensorView.cpp ${UIS_OUT_H})




# Header files to moc and compile:
QT4_WRAP_CPP(MocSrcFiles Sensors.hpp SensorView.hpp)

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
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  # Link the standard static libraries with with the shared library:
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES})
  message("ROBWORKSTUDIO_LIBRARIES: ${ROBWORKSTUDIO_LIBRARIES}")
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_BIN_INSTALL_DIR})
ENDIF ()
ADD_DEPENDENCIES(${TargetName} rwstudio)

SET(ENV{RWS_PLUGIN_LIBRARIES} "$ENV{RWS_PLUGIN_LIBRARIES}${TargetName};")
