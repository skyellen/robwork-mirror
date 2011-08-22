
if( OPENGL_FOUND AND OPENGL_GLU_FOUND )

SET(SRC_FILES
  ${RW_DRAWABLE_USER_FILES}
  Drawable.cpp
  RWGLFrameBuffer.cpp
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
  Drawable.hpp
  RWGLFrameBuffer.hpp
  DrawableFactory.hpp
  DrawableUtil.hpp
  RenderFrame.hpp
  RenderGeometry.hpp
  RenderLines.hpp
  RenderMatrix.hpp
  SceneOpenGL.hpp
  TactileArrayRender.hpp
  RWGLTexture.hpp
  RenderImage.hpp
  RenderModel3D.hpp
  RenderUtil.hpp
  RenderScan.hpp
  RenderPointCloud.hpp
  RenderCameraFrustum.hpp
  DrawableGeometry.hpp
)

ADD_LIBRARY(rw_opengl ${SRC_FILES} ${SRC_FILES_HPP})

INSTALL(TARGETS rw_opengl DESTINATION ${RW_LIB_INSTALL_DIR})

else ()
  message(SEND_WARNING "Opengl an Glu is needed for compiling the drawable library!")
endif()



