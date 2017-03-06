
# if we want to use ui files add them here
SET(UIS_FILES AboutBox.ui propertyview/PropertyViewEditor.ui propertyview/PropertyViewDialog.ui)

IF(RWS_USE_QT5)
	QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ELSE()
	QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ENDIF()

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
  ImageView.cpp
  propertyview/PropertyViewEditor.cpp
  propertyview/PropertyViewDialog.cpp)

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
  HelpAssistant.hpp
  ImageView.hpp
  propertyview/PropertyViewEditor.hpp
  propertyview/PropertyViewDialog.hpp 
  )

# Files to moc and compile:
SET(MocHeaderFiles
  RobWorkStudioPlugin.hpp
  RobWorkStudio.hpp
  SceneOpenGLViewer.hpp
  AboutBox.hpp
  RWStudioView3D.hpp
  ImageView.hpp
  propertyview/PropertyViewEditor.hpp 
  propertyview/PropertyViewDialog.hpp )


set_source_files_properties(${SRC_FILES_CPP} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

#Add the cmake binary dir because this is where the ui files are placed
#MESSAGE(STATUS "CMAKE_CURRENT_BINARY_DIR:  ${CMAKE_CURRENT_BINARY_DIR}")
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 


SET(MocSrcFiles)
IF(RWS_USE_QT5)
	QT5_WRAP_CPP(MocSrcFiles ${MocHeaderFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ELSE()
	QT4_WRAP_CPP(MocSrcFiles ${MocHeaderFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
ENDIF()
set_source_files_properties(${MocSrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")

# Resource files to rcc and compile:
IF(RWS_BUILD_SANDBOX)
    #SET(SandboxResourceFiles rwstudio_sandbox_resources.qrc)
ENDIF ()
# Rcc the files:
IF(RWS_USE_QT5)
	QT5_ADD_RESOURCES(ResourceFiles rwstudio_resources.qrc)
ELSE()
	QT4_ADD_RESOURCES(ResourceFiles rwstudio_resources.qrc)
ENDIF()

# We create a static library named rwstudio that all plugins should link to. Our
# RobWorkStudio executable links to this library also.
ADD_LIBRARY(rws
            ${UIS_OUT_H}
            ${SRC_FILES_CPP}
            ${MocSrcFiles}
            ${ResourceFiles}
)
TARGET_LINK_LIBRARIES(rws qtpropertybrowser ${QT_LIBRARIES} ${ROBWORK_LIBRARIES})

# Install stuff
INSTALL(TARGETS rws DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${SRC_FILES_HPP} DESTINATION ${INCLUDE_INSTALL_DIR}/rws)

ADD_LIBRARY(rws_plugin.rwplugin MODULE RWSImageLoaderPlugin.cpp RWSImageLoaderPlugin.hpp)
TARGET_LINK_LIBRARIES(rws_plugin.rwplugin rws ${QT_LIBRARIES} ${ROBWORK_LIBRARIES})
