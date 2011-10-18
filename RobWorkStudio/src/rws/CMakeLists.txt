# compile the components 
SET(ENV{RWS_COMPONENT_LIBRARIES} "")
ADD_SUBDIRECTORY(components)

# compile the plugins 
SET(ENV{RWS_PLUGIN_LIBRARIES} "")
ADD_SUBDIRECTORY(plugins)
ADD_SUBDIRECTORY(swig)

SET(UIS_FILES AboutBox.ui )
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})

# Standard files to compile:
SET(SRC_FILES_CPP
  ArcBallController.cpp
  #FixedAxisController.cpp
  RWSImageLoaderPlugin.cpp
  ImageUtil.cpp
  RobWorkStudio.cpp
  RWStudioView3D.cpp
  SceneOpenGLViewer.cpp
  RobWorkStudioPlugin.cpp
  AboutBox.cpp
  HelpAssistant.cpp
  ${UIS_OUT_H})

SET(SRC_FILES_HPP
  ArcBallController.hpp
  RWSImageLoaderPlugin.hpp
  ImageUtil.hpp
  #FixedAxisController.hpp
  RobWorkStudio.hpp
  SceneOpenGLViewer.hpp
  RobWorkStudioPlugin.hpp
  RWStudioView3D.hpp
  AboutBox.hpp
  HelpAssistant.hpp)

# Files to moc and compile:
SET(MocHeaderFiles
  RobWorkStudioPlugin.hpp
  RobWorkStudio.hpp
  SceneOpenGLViewer.hpp
  AboutBox.hpp
  RWStudioView3D.hpp
  HelpAssistant.hpp)


SET(MocSrcFiles)
QT4_WRAP_CPP(MocSrcFiles ${MocHeaderFiles})

# Resource files to rcc and compile:
IF(RWS_BUILD_SANDBOX)
    #SET(SandboxResourceFiles rwstudio_sandbox_resources.qrc)
ENDIF ()
QT4_ADD_RESOURCES(ResourceFiles rwstudio_resources.qrc)

# We create a static library named rwstudio that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
set(TargetName rwstudio)

ADD_LIBRARY(${TargetName}
            ${SRC_FILES_CPP}
            ${MocSrcFiles}
            ${ResourceFiles}
)

# Install stuff
INSTALL(TARGETS rwstudio DESTINATION ${RWS_LIB_INSTALL_DIR})
#INSTALL(FILES ${SRC_FILES_CPP} DESTINATION ${RWS_CPP_INSTALL_DIR}/rws)
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${RWS_HPP_INSTALL_DIR}/rws)

ADD_DEPENDENCIES(rwstudio rwstudio_propertyinspector)

ADD_SUBDIRECTORY(lua)
