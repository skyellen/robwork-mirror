

# if we want to use ui files add them here
SET(UIS_FILES AboutBox.ui propertyview/PropertyViewEditor.ui propertyview/PropertyViewDialog.ui)

RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})


# Standard files to compile:
SET(SRC_FILES_CPP
  ArcBallController.cpp
  #FixedAxisController.cpp
  RWSImageLoaderPlugin.cpp
  ImageUtil.cpp
  RobWorkStudio.cpp
  RobWorkStudioApp.cpp
  RWStudioView3D.cpp
  SceneOpenGLViewer.cpp
  RobWorkStudioPlugin.cpp
  AboutBox.cpp
  HelpAssistant.cpp
  propertyview/PropertyViewEditor.cpp
  propertyview/PropertyViewDialog.cpp 
  propertyview/PropertyInspector.cpp
  propertyview/VariantManager.cpp)

SET(SRC_FILES_HPP
  ArcBallController.hpp
  RWSImageLoaderPlugin.hpp
  ImageUtil.hpp
  #FixedAxisController.hpp
  RobWorkStudio.hpp
  RobWorkStudioApp.hpp
  SceneOpenGLViewer.hpp
  RobWorkStudioPlugin.hpp
  RWStudioView3D.hpp
  AboutBox.hpp
  HelpAssistant.hpp
  propertyview/PropertyViewEditor.hpp
  propertyview/PropertyViewDialog.hpp 
  propertyview/PropertyInspector.hpp
  propertyview/VariantManager.hpp
  )

# Files to moc and compile:
SET(MocHeaderFiles
  RobWorkStudioPlugin.hpp
  RobWorkStudio.hpp
  SceneOpenGLViewer.hpp
  AboutBox.hpp
  RWStudioView3D.hpp
  HelpAssistant.hpp
  propertyview/PropertyViewEditor.hpp 
  propertyview/PropertyViewDialog.hpp )


set_source_files_properties(${SRC_FILES_CPP} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

SET(MocSrcFiles)
QT4_WRAP_CPP(MocSrcFiles ${MocHeaderFiles} ${UIS_OUT_H})
MESSAGE( ${UIS_OUT_H} )
# Resource files to rcc and compile:
IF(RWS_BUILD_SANDBOX)
    #SET(SandboxResourceFiles rwstudio_sandbox_resources.qrc)
ENDIF ()
# Rcc the files:
QT4_ADD_RESOURCES(ResourceFiles rwstudio_resources.qrc propertyview/resources.qrc)

# We create a static library named rwstudio that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
ADD_LIBRARY(rwstudio
            ${SRC_FILES_CPP}
            ${MocSrcFiles}
            ${ResourceFiles}
)

# Install stuff
INSTALL(TARGETS rwstudio DESTINATION ${RWS_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_CPP} DESTINATION ${RWS_CPP_INSTALL_DIR}/rws)
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RWS_HPP_INSTALL_DIR}/rws)

#ADD_SUBDIRECTORY(lua)
