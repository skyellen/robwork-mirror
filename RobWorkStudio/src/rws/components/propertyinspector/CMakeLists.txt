#----------------------------------------------------------------------
# Define the following variables:

# The shared library to build:
set(TargetName rwstudio_propertyinspector)

# if we want to use ui files add them here
SET(UIS_FILES PropertyViewEditor.ui )
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

SET(SrcFiles PropertyInspector.cpp PropertyViewEditor.cpp ${UIS_OUT_H})

QT4_WRAP_CPP(MocSrcFiles PropertyInspector.hpp PropertyViewEditor.hpp ${UIS_OUT_H})

# Rcc the files:
#QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)



#----------------------------------------------------------------------
# Everything below should be common for all components:

# The library to build:
IF (RWS_USE_STATIC_LINK_COMPONENTS)
  #ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  ADD_LIBRARY(${TargetName} STATIC ${SrcFiles} ${MocSrcFiles})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_LIB_INSTALL_DIR})
ELSE ()
  #ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles} ${RccSrcFiles})
  ADD_LIBRARY(${TargetName} SHARED ${SrcFiles} ${MocSrcFiles})
  # Link the standard static libraries with with the shared library:
  TARGET_LINK_LIBRARIES(${TargetName} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
  INSTALL(TARGETS ${TargetName} DESTINATION ${RWS_BIN_INSTALL_DIR})
ENDIF ()

#ADD_DEPENDENCIES(${TargetName} rwstudio)

SET(ENV{RWS_COMPONENT_LIBRARIES} "$ENV{RWS_COMPONENT_LIBRARIES}${TargetName};")
