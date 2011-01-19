
if( OPENGL_FOUND AND OPENGL_GLU_FOUND )

SET(SRC_FILES
  ${RW_DRAWABLE_USER_FILES}
  Drawable.cpp
  DrawableFactory.cpp
  DrawableUtil.cpp
  RenderFrame.cpp
  RenderGeometry.cpp
  RenderLines.cpp
  RenderMatrix.cpp
  SceneOpenGL.cpp
  TactileArrayRender.cpp
  RWGLTexture.cpp
  RenderImage.cpp
  RenderModel3D.cpp
  RenderUtil.cpp
  RenderScan.cpp
  RenderPointCloud.cpp
  RenderCameraFrustum.cpp
  DrawableGeometry.cpp
)

SET(SRC_FILES_HPP
)

ADD_LIBRARY(rw_opengl ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_opengl DESTINATION ${RW_LIB_INSTALL_DIR})

else ()
  message(SEND_WARNING "Opengl an Glu is needed for compiling the drawable library!")
endif()



